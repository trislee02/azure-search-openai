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

from approaches.promptutils import ChatRAGPrompt, ChatVectorComparePrompt, ChatMultiSearchPrompt, ChatGeneralPrompt
from scipy.spatial.distance import cosine

from approaches.checker.codechecker import CodeChecker
from approaches.checker.embedtextchecker import EmbedTextChecker
from approaches.checker.codeattributionchecker import CodeAttributionChecker

from core.modelhelper import num_tokens_from_chat_messages

class ChatRAGMultiSearchApproach(Approach):
    """
    This approach includes following steps:
        1. Extract main requests of message based on new message and chat history
        2. For each request, run RAG: 
            a. Search (Semantic search + Vector search)
            b. Generate response for the request based on retrieved documents
            c. Compare response vector and retrieved document vectors
                - Repeatedly refine the response with max try
        3. Merge all the responses to generate the final response
    """

    # Chat roles
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"

    LOG_ANSWER_FOR_REQUEST = """\nRequest: {query_text}

Answer: {chat_content}
------------------
"""

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
                 embedding_deployment = "",
                 completion_deployment = "",
                 completion_model = ""):
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
        self.completion_deployment = completion_deployment
        self.completion_model = completion_model

    def before_retry_sleep(retry_state):
        print(f"Rate limited on the OpenAI API, sleeping before retrying...")

    @retry(wait=wait_random_exponential(min=15, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
    def __compute_chat_completion(self, messages, temperature, max_tokens, n):
        completion = None
        max_tokens = max_tokens - num_tokens_from_chat_messages(messages=messages, model=self.chatgpt_model)
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
    
    def __count_tokens(self, gpt_response):
        # return gpt_response.usage["prompt_tokens"]
        # return gpt_response.usage["completion_tokens"]
        return gpt_response.usage["total_tokens"]
    
    def __compute_completion(self, prompt, temperature, max_tokens=4097):
        completion = None
        if self.completion_deployment != "":
            completion = openai.Completion.create(
                deployment_id=self.completion_deployment,
                model=self.completion_model,
                prompt=prompt,
                temperature=temperature,
                max_tokens=max_tokens)
        else:
            completion = openai.Completion.create(
                model=self.completion_model,
                prompt=prompt, 
                temperature=temperature, 
                max_tokens=max_tokens)
        return completion

    def __compute_embedding(self, text): 
        if self.embedding_deployment != "":
            vector = openai.Embedding.create(engine=self.embedding_deployment, input=text)["data"][0]["embedding"]                
        else:
            vector = openai.Embedding.create(input=text, model=self.embed_model)["data"][0]["embedding"]
        return vector

    def __extract_main_requests(self, history: Sequence[dict[str, str]]) -> list[str]:
        messages = self.get_messages_from_history(
            ChatMultiSearchPrompt.system_message,
            self.chatgpt_model,
            history,
            ChatMultiSearchPrompt.user_message_template.format(message=history[-1]["user"]),
            ChatMultiSearchPrompt.few_shots
        )
        
        chat_completion = self.__compute_chat_completion(messages=messages, 
                                                         temperature=0.0, 
                                                         max_tokens=self.chatgpt_token_limit, 
                                                         n=1)
        tokens_count = self.__count_tokens(chat_completion)
        
        print(f"Requests: {chat_completion.choices[0].message.content}")
        extracted_requests = chat_completion.choices[0].message.content.strip().split("\n")

        # Check whether no questions found
        if len(extracted_requests) == 1 and extracted_requests[0] == "0":
            return tokens_count, []
        return tokens_count, extracted_requests

    def __retrieve_documents(self, query_text: str, overrides: dict[str, Any]):
        has_text = overrides.get("retrieval_mode") in ["text", "hybrid", None]
        has_vector = overrides.get("retrieval_mode") in ["vectors", "hybrid", None]
        use_semantic_captions = True if overrides.get("semantic_captions") and has_text else False
        top = overrides.get("top") or 3
        exclude_category = overrides.get("exclude_category") or None
        filter = "category ne '{}'".format(exclude_category.replace("'", "''")) if exclude_category else None

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
        
        return retrieved_docs, retrieved_doc_embeds

    def run(self, history: Sequence[dict[str, str]], overrides: dict[str, Any]) -> Any:
        raw_answer = "" # Raw answer contains separated responses for requests
        check_logs = ""
        all_supporting_contents = ""
        tokens_count = 0
        retrieved_docs = []
        final_answer = ""
        contexts = []
        query_texts = []
        thoughts = ""

        # STEP 1: Extract main requests in message
        num_tokens, query_texts = self.__extract_main_requests(history=history)
        
        tokens_count += num_tokens

        # Check whether no questions found
        if len(query_texts) == 0:
            user_msg = ChatGeneralPrompt.user_message_template.format(message=history[-1]["user"])  

            messages = self.get_messages_from_history(
                system_prompt=ChatGeneralPrompt.system_message,
                model_id=self.chatgpt_model,
                history=history,
                user_conv=user_msg,
                max_tokens=self.chatgpt_token_limit - len(user_msg)
            )
            
            chat_completion = self.__compute_chat_completion(messages=messages, temperature=0.7, max_tokens=200, n=1)
            
            chat_content = chat_completion.choices[0].message.content
            
            msg_to_display = self.format_display_message(messages)
            
            thoughts = msg_to_display
            final_answer = chat_content
        else:
            
            for query_text in query_texts:
                query_text = query_text.strip()
                if query_text == "":
                    continue
                # STEP 2: Retrieve relevant documents from the search index with the GPT optimized query
                retrieved_docs, retrieved_doc_embeds = self.__retrieve_documents(query_text=query_text, overrides=overrides)
                            
                supporting_content = "\n".join(retrieved_docs)
                all_supporting_contents += f"##########################\nRequest: {query_text}\n{supporting_content}\n\n"

                follow_up_questions_prompt = ChatRAGPrompt.follow_up_questions_prompt_content if overrides.get("suggest_followup_questions") else ""
                
                # STEP 3: Generate a contextual and content specific answer using the search results and chat history

                # Allow client to replace the entire prompt, or to inject into the existing prompt using >>>
                prompt_override = overrides.get("prompt_override")
                if prompt_override is None:
                    system_message = ChatRAGPrompt.system_message_chat_conversation.format(injected_prompt="", 
                                                                                        follow_up_questions_prompt=follow_up_questions_prompt)
                elif prompt_override.startswith(">>>"):
                    system_message = ChatRAGPrompt.system_message_chat_conversation.format(injected_prompt=prompt_override[3:] + "\n", follow_up_questions_prompt=follow_up_questions_prompt)
                else:
                    system_message = prompt_override.format(follow_up_questions_prompt=follow_up_questions_prompt)
                
                user_conv = ChatRAGPrompt.user_chat_template.format(content=supporting_content, 
                                                                    question=query_text)
                
                messages = self.get_messages_from_history(
                    system_message,
                    self.chatgpt_model,
                    history,
                    user_conv,
                    max_tokens=self.chatgpt_token_limit)

                chat_completion = self.__compute_chat_completion(messages=messages, 
                                                                temperature=overrides.get("temperature") or 0.0, 
                                                                max_tokens=self.chatgpt_token_limit, 
                                                                n=1)
                tokens_count += self.__count_tokens(chat_completion)

                chat_content = chat_completion.choices[0].message.content

                check_logs += f"\nRequest: {query_text}"
                def debug_callback(log):
                    nonlocal check_logs
                    check_logs += f"\n{log}"

                ## STEP 4: Run checks on LLM's answer
                # An answer is valid only if it satisfies all checkers
                is_valid = False
                tries = 0
                while not is_valid and tries <= self.MAX_TRY:
                    ### If not valid, refine it
                    if not is_valid and tries > 0:
                        response_to_revise = ChatVectorComparePrompt.user_message_revise_template.format(source=supporting_content,
                                                                                    question=history[-1]["user"],
                                                                                    previous_answer=previous_answer)
                        messages = [{"role":"system","content": ChatVectorComparePrompt.system_message_revise},
                                    {"role":"user","content": response_to_revise}]

                        chat_completion = self.__compute_chat_completion(messages=messages, 
                                                                        temperature=overrides.get("temperature") or 0.0, 
                                                                        max_tokens=self.chatgpt_token_limit, 
                                                                        n=1)
                        tokens_count += self.__count_tokens(chat_completion)
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
                        embed_text_checker = EmbedTextChecker(embedding_deployment=self.embedding_deployment)
                        
                        embed_valid = embed_text_checker.check(previous_answer, retrieved_docs, retrieved_doc_embeds, debug_callback)
                        toxicity_valid = self.selfcheck_toxicity(previous_answer)
                        print("Toxicity valid:", toxicity_valid)
                        is_valid = embed_valid and toxicity_valid
                        tries += 1
                    else:
                        # All "I don't know" answer is valid
                        is_valid = True

                
                ## Can't find valid answer after a lot of tries, respond "I don't know"
                if not is_valid:
                    # chat_content = ChatVectorComparePrompt.no_answer_message
                    chat_content = ChatVectorComparePrompt.book_a_meeting_message
                
                answer_for_request = self.LOG_ANSWER_FOR_REQUEST.format(query_text=query_text, chat_content=chat_content)
                check_logs += answer_for_request

                raw_answer += answer_for_request
                print(f"Answer for request: {answer_for_request}")

            user_conv = ChatMultiSearchPrompt.user_content_merge_answer.format(customer_message=history[-1]['user'],
                                                                            raw_answer=raw_answer)

            messages = [{"role":"system","content": ChatMultiSearchPrompt.system_message_merge_answer},
                        {"role":"user","content": user_conv}]
            
            print(f"Messages: {messages}")

            chat_completion = self.__compute_chat_completion(messages=messages, 
                                                            temperature=overrides.get("temperature") or 0.0, 
                                                            max_tokens=self.chatgpt_token_limit, 
                                                            n=1)
            tokens_count += self.__count_tokens(chat_completion)
            final_answer = chat_completion.choices[0].message.content
            print(f"Final answer: {final_answer}")

            contexts = [raw_answer]

            # msg_to_display = self.format_display_message(messages)
            # msg_to_display = self.format_display_message(text=msg_to_display)
            thoughts = self.format_display_message(text=check_logs)

        return {"data_points": retrieved_docs, 
                "answer": final_answer,
                "contexts": contexts,
                "supporting_contents": all_supporting_contents,
                "token_usage": tokens_count,
                "extracted_requests": query_texts,
                "thoughts": thoughts}
    
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