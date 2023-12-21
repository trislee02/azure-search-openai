from typing import Any, Sequence
from approaches.subrag.subrag import SubRAG

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

from approaches.promptutils import ChatRAGPrompt, ChatVectorComparePrompt, ChatRAGCodePrompt
from scipy.spatial.distance import cosine

from approaches.checker.codechecker import CodeChecker
from approaches.checker.embedtextchecker import EmbedTextChecker
from approaches.checker.codeattributionchecker import CodeAttributionChecker

from core.modelhelper import num_tokens_from_chat_messages, count_tokens

class SubRAGCode(SubRAG):

    # Chat roles
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"

    LOG_ANSWER_FOR_REQUEST = """
Request: {query_text}

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
        # print(messages)
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

    def __compute_embedding(self, text): 
        if self.embedding_deployment != "":
            vector = openai.Embedding.create(engine=self.embedding_deployment, input=text)["data"][0]["embedding"]                
        else:
            vector = openai.Embedding.create(input=text, model=self.embed_model)["data"][0]["embedding"]
        return vector

    def __retrieve_documents(self, query_text: str, overrides: dict[str, Any]):
        """
        Retrieve relevant documents
        Return:
            list[str]: list of document content
            list[list[float]]: list of embeddings of documents
        """
        has_text = False
        use_semantic_captions = True if overrides.get("semantic_captions") and has_text else False
        top = 3

        # Retrieval mode includes vectors, compute an embedding for the query
        query_vector = self.__compute_embedding(query_text)

        results = self.search_client.search(query_text, 
                                            top=top, 
                                            vector=query_vector, 
                                            top_k=50 if query_vector else None, 
                                            vector_fields="embedding" if query_vector else None)
        retrieved_docs = []
        retrieved_doc_embeds = []
        for doc in results:
            if "url" in doc.keys():
                doc_content = f"[{doc['url']}]" + ": " + nonewlines(doc[self.content_field])
            else:
                doc_content = f"[{doc[self.sourcepage_field]}]" + ": " + nonewlines(doc[self.content_field])
            doc_vector = doc[self.embedding_field]
            
            retrieved_docs.append(doc_content)
            retrieved_doc_embeds.append(doc_vector)
        
        return retrieved_docs, retrieved_doc_embeds

    def run(self, history: Sequence[dict[str, str]], overrides: dict[str, Any], query_text: str = None) -> Any:
        check_logs = ""
        all_supporting_contents = ""
        tokens_count = 0
        retrieved_docs = []
        thoughts = ""

        if query_text is None:
            query_text = history[-1]["user"]

        # STEP 1: Retrieve relevant documents from the search index
        retrieved_docs, retrieved_doc_embeds = self.__retrieve_documents(query_text=query_text, overrides=overrides)

        supporting_content = "\n-------\n".join(retrieved_docs)
        all_supporting_contents += f"Request: {query_text}\n{supporting_content}\n\n"
        
        # STEP 2: Generate a contextual and content specific answer using the search results and chat history
        system_message = ChatRAGCodePrompt.system_message
        user_conv = ChatRAGCodePrompt.user_content_template.format(context=supporting_content, 
                                                            question=query_text)

        # system_message = ChatRAGPrompt.system_message_chat_conversation
        # user_conv = ChatRAGPrompt.user_chat_template.format(content=supporting_content, 
        #                                                     question=query_text)
        
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
        tokens_count += count_tokens(chat_completion)
        answer = chat_completion.choices[0].message.content

        check_logs += f"\nRequest: {query_text}"
        def debug_callback(log):
            nonlocal check_logs
            check_logs += f"\n{log}"

        ## STEP 3: Run checks on LLM's answer
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
                tokens_count = count_tokens(chat_completion)
                answer = chat_completion.choices[0].message.content

            previous_answer = answer
            
            # Check whether LLM's answer is "I don't know"
            answer_vector = self.__compute_embedding(previous_answer)
            no_answer_distance = cosine(ChatVectorComparePrompt.no_answer_embed, answer_vector)

            # If LLM says it knows the answer, then run checks
            if (no_answer_distance > self.THRESHOLD_NO_ANSWER):
                # Run checks
                embed_text_checker = EmbedTextChecker(embedding_deployment=self.embedding_deployment)
                
                embed_valid = embed_text_checker.check(previous_answer, retrieved_docs, retrieved_doc_embeds, debug_callback)
                is_valid = embed_valid
                tries += 1
            else:
                # All "I don't know" answer is valid
                is_valid = True
                answer = ""
        
        ## Can't find valid answer after a lot of tries, respond "I don't know"
        if not is_valid:
            answer = ""
        
        thoughts = self.format_display_message(text=check_logs)

        return {"data_points": retrieved_docs, 
                "answer": answer,
                "supporting_contents": all_supporting_contents,
                "token_usage": tokens_count,
                "thoughts": thoughts, }
    
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