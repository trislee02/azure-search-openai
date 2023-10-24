from typing import Any, Sequence

import openai
import tiktoken
import json
from azure.search.documents import SearchClient
from azure.search.documents.models import QueryType
from approaches.approach import Approach
from text import nonewlines

from core.messagebuilder import MessageBuilder
from core.modelhelper import get_token_limit

from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed

from approaches.promptutils import ChatRAGPrompt, ChatVectorComparePrompt, CheckerFeedbackPrompt
from scipy.spatial.distance import cosine

from approaches.checker.codechecker import CodeChecker
from approaches.checker.embedtextchecker import EmbedTextChecker
from approaches.checker.codeattributionchecker import CodeAttributionChecker

class ChatRAGCompareTextAndCodeApproach(Approach):
    """
    This approach includes following steps:
        1. RAG
            a. Refine search query based on new message and chat history
            b. Search (Semantic search + Vector search)
            c. Generate first answer based on retrieved documents
        2. Iterative vector and answer comparison
            a. Extract main ideas from previous answer
            b. Compare previous answer embedding with each retrieved document embedding
            c. Repeatedly refine the answer with max try
    """

    # Chat roles
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"

    THRESHOLD_NO_ANSWER = 0.25

    MAX_TRY = 3

    def __init__(self, 
                 search_client: SearchClient, 
                 sourcepage_field: str, 
                 content_field: str,
                 embedding_field: str,
                 chatgpt_model = "", 
                 embed_model = "", 
                 chatgpt_deployment = "", 
                 embedding_deployment = ""):
        super().__init__()
        
        self.search_client = search_client
        self.sourcepage_field = sourcepage_field
        self.content_field = content_field
        self.embedding_field = embedding_field
        self.embed_model = embed_model
        self.chatgpt_deployment = chatgpt_deployment
        self.chatgpt_model = chatgpt_model
        self.embedding_deployment = embedding_deployment
        self.chatgpt_token_limit = get_token_limit(chatgpt_model)

    def before_retry_sleep(retry_state):
        print(f"Rate limited on the OpenAI API, sleeping 60s before retrying...")

    # @retry(wait=wait_random_exponential(min=15, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
    def __compute_chat_completion(self, messages, temperature, max_tokens, n):
        completion = None
        if self.chatgpt_deployment != "":
            completion = openai.ChatCompletion.create(
                deployment_id=self.chatgpt_deployment,
                model=self.chatgpt_model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens,
                n=n)
        else:
            completion = openai.ChatCompletion.create(
                model=self.chatgpt_model,
                messages=messages, 
                temperature=temperature, 
                max_tokens=max_tokens, 
                n=n)
        
        return completion

    def __compute_embedding(self, text): 
        if self.embedding_deployment != "":
            vector = openai.Embedding.create(engine=self.embedding_deployment, input=text)["data"][0]["embedding"]                
        else:
            vector = openai.Embedding.create(input=text, model=self.embed_model)["data"][0]["embedding"]
        return vector

    def run(self, history: Sequence[dict[str, str]], overrides: dict[str, Any]) -> Any:
        has_text = overrides.get("retrieval_mode") in ["text", "hybrid", None]
        has_vector = overrides.get("retrieval_mode") in ["vectors", "hybrid", None]
        use_semantic_captions = True if overrides.get("semantic_captions") and has_text else False
        top = overrides.get("top") or 3
        exclude_category = overrides.get("exclude_category") or None
        filter = "category ne '{}'".format(exclude_category.replace("'", "''")) if exclude_category else None

        user_q = 'Generate search query for: ' + history[-1]["user"]

        # STEP 1: Generate an optimized keyword search query based on the chat history and the last question
        messages = self.get_messages_from_history(
            ChatRAGPrompt.query_prompt_template,
            self.chatgpt_model,
            history,
            user_q,
            ChatRAGPrompt.query_prompt_few_shots,
            self.chatgpt_token_limit - len(user_q)
            )
        
        chat_completion = self.__compute_chat_completion(messages=messages, temperature=0.0, max_tokens=32, n=1)
                
        query_text = chat_completion.choices[0].message.content
        if query_text.strip() == "0":
            query_text = history[-1]["user"] # Use the last user input if we failed to generate a better query

        # STEP 2: Retrieve relevant documents from the search index with the GPT optimized query

        # If retrieval mode includes vectors, compute an embedding for the query
        if has_vector:
            query_vector = self.__compute_embedding(query_text)
        else:
            query_vector = None

        # Only keep the text query if the retrieval mode uses text, otherwise drop it
        if not has_text:
            query_text = None

        # Use semantic L2 reranker if requested and if retrieval mode is text or hybrid (vectors + text)
        if overrides.get("semantic_ranker") and has_text:
            r = self.search_client.search(query_text, 
                                          filter=filter,
                                          query_type=QueryType.SEMANTIC, 
                                          query_language="en-us", 
                                          query_speller="lexicon", 
                                          semantic_configuration_name="default", 
                                          top=top, 
                                          query_caption="extractive|highlight-false" if use_semantic_captions else None,
                                          vector=query_vector, 
                                          top_k=50 if query_vector else None,
                                          vector_fields="embedding" if query_vector else None)
        else:
            r = self.search_client.search(query_text, 
                                          filter=filter, 
                                          top=top, 
                                          vector=query_vector, 
                                          top_k=50 if query_vector else None, 
                                          vector_fields="embedding" if query_vector else None)
        retrieved_docs = []
        retrieved_doc_embeds = []
        for doc in r:
            if use_semantic_captions:
                doc_content = f"[{doc[self.sourcepage_field]}]" + ": " + nonewlines(" . ".join([c.text for c in doc['@search.captions']]))
            else:
                doc_content = f"[{doc[self.sourcepage_field]}]" + ": " + nonewlines(doc[self.content_field])
            doc_vector = doc[self.embedding_field]
            
            retrieved_docs.append(doc_content)
            retrieved_doc_embeds.append(doc_vector)
        
        supporting_content = "\n".join(retrieved_docs)

        follow_up_questions_prompt = ChatRAGPrompt.follow_up_questions_prompt_content if overrides.get("suggest_followup_questions") else ""
        
        # STEP 3: Generate a contextual and content specific answer using the search results and chat history

        # Allow client to replace the entire prompt, or to inject into the existing prompt using >>>
        prompt_override = overrides.get("prompt_override")
        if prompt_override is None:
            system_message = ChatRAGPrompt.system_message_chat_conversation.format(injected_prompt="", follow_up_questions_prompt=follow_up_questions_prompt)
        elif prompt_override.startswith(">>>"):
            system_message = ChatRAGPrompt.system_message_chat_conversation.format(injected_prompt=prompt_override[3:] + "\n", follow_up_questions_prompt=follow_up_questions_prompt)
        else:
            system_message = prompt_override.format(follow_up_questions_prompt=follow_up_questions_prompt)
        
        user_conv = ChatRAGPrompt.user_chat_template.format(content=supporting_content, question=history[-1]["user"])
        
        messages = self.get_messages_from_history(
            system_message,
            self.chatgpt_model,
            history,
            user_conv,
            max_tokens=self.chatgpt_token_limit)

        chat_completion = self.__compute_chat_completion(messages=messages, 
                                                         temperature=overrides.get("temperature") or 0.0, 
                                                         max_tokens=1024, 
                                                         n=1)
        
        chat_content = chat_completion.choices[0].message.content

        check_logs = ""
        def debug_callback(log):
            nonlocal check_logs
            check_logs += f"\n{log}"

        ## STEP 4: Run checks on LLM's answer
        # An answer is valid only if it satisfies all checkers
        is_valid = False
        code_valid = False
        tries = 0
        while not is_valid and tries <= self.MAX_TRY:
            ### If not valid, refine it
            if not is_valid and tries > 0:
                response_to_revise = ChatVectorComparePrompt.user_message_revise_template.format(source=supporting_content,
                                                                            question=history[-1]["user"],
                                                                            previous_answer=previous_answer)
                if not code_valid:
                    messages = [{"role":"system","content": ChatVectorComparePrompt.system_message_revise.format(injected_prompt=CheckerFeedbackPrompt.revise_code_hallucination)},
                            {"role":"user","content": response_to_revise}]
                else:
                    messages = [{"role":"system","content": ChatVectorComparePrompt.system_message_revise},
                            {"role":"user","content": response_to_revise}]

                chat_completion = self.__compute_chat_completion(messages=messages, 
                                                                temperature=overrides.get("temperature") or 0.0, 
                                                                max_tokens=1024, 
                                                                n=1)
                chat_content = chat_completion.choices[0].message.content
                print(f"Revised answer #{tries}: {chat_content}")

            previous_answer = chat_content
            
            # Check whether LLM's answer is "I don't know"
            answer_vector = self.__compute_embedding(previous_answer)
            no_answer_distance = cosine(ChatVectorComparePrompt.no_answer_embed, answer_vector)
            print(f"No answer distance: {no_answer_distance}")
            # If LLM says it knows the answer, then run checks
            if (no_answer_distance > self.THRESHOLD_NO_ANSWER):
                # Run checks
                code_checker = CodeAttributionChecker()
                embed_text_checker = EmbedTextChecker(embedding_deployment=self.embedding_deployment)
                
                code_valid = code_checker.check(previous_answer, retrieved_docs, debug_callback)
                embed_valid = embed_text_checker.check(previous_answer, retrieved_docs, retrieved_doc_embeds, debug_callback)
                toxicity_valid = self.selfcheck_toxicity(previous_answer)
                print("Toxicity valid:", toxicity_valid)
                is_valid = code_valid and embed_valid and toxicity_valid
                tries += 1
            else:
                # All "I don't know" answer is valid
                is_valid = True

        
        ## Can't find valid answer after a lot of tries, respond "I don't know"
        if not is_valid:
            chat_content = ChatVectorComparePrompt.no_answer_message
        
        print(f"Final answer: {chat_content}")

        msg_to_display = self.format_display_message(messages)
        # msg_to_display = self.format_display_message(text=msg_to_display)
        msg_to_display = self.format_display_message(text=check_logs)
        return {"data_points": retrieved_docs, 
                "answer": chat_content, 
                "thoughts": f"Searched for:<br>{query_text}<br><br>Conversations:<br>{msg_to_display}"}
    
    def get_messages_from_history(self, system_prompt: str, model_id: str, history: Sequence[dict[str, str]], user_conv: str, few_shots = [], max_tokens: int = 4096) -> []:
        message_builder = MessageBuilder(system_prompt, model_id)

        # Add examples to show the chat what responses we want. It will try to mimic any responses and make sure they match the rules laid out in the system message.
        for shot in few_shots[::-1]:
            message_builder.append_message(shot.get('role'), shot.get('content'))

        user_content = user_conv
        append_index = len(few_shots) + 1

        message_builder.append_message(self.USER, user_content, index=append_index)

        for h in reversed(history[:-1]):
            if h.get("bot"):
                message_builder.append_message(self.ASSISTANT, h.get('bot'), index=append_index)
            message_builder.append_message(self.USER, h.get('user'), index=append_index)
            if message_builder.token_length > max_tokens:
                break
        
        messages = message_builder.messages
        return messages
    
    def format_display_message(self, list_messages: list = None, text: str = "") -> str:
        msg_to_display = text
        if list_messages:
            msg_to_display = '\n=========================================\n\n'.join([str(message) for message in list_messages])
        msg_to_display = msg_to_display.replace('\\n', '<br>').replace('\n', '<br>')
        return msg_to_display