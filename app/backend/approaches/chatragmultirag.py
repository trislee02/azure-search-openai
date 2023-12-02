from typing import Any, Sequence
from collections import defaultdict

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

from approaches.promptutils import ChatRAGPrompt, ChatVectorComparePrompt, ChatMultiSearchPrompt, ChatGeneralPrompt, IntentClassificationPrompt
from scipy.spatial.distance import cosine

from core.modelhelper import num_tokens_from_chat_messages

from approaches.subrag.subragcode import SubRAGCode
from approaches.subrag.subragtext import SubRAGText


class ChatRAGMultiRAGApproach(Approach):
    """
    This approach includes following steps:
        1. Extract main requests of message based on new message and chat history
        2. For each request, run intent classification
        3. Run sub-RAG for each request
        4. Merge all the responses to generate the final response
    """

    class RAGIntent:
        TEXT = "text"
        CODE = "code"
        ROS = "ros"

    # Chat roles
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"

    LOG_ANSWER_FOR_REQUEST = """\nRequest: {query_text}

Answer: {chat_content}
------------------
"""

    THRESHOLD_NO_ANSWER = 0.25
    THRESHOLD_REQUEST_CODE = 0.23

    MAX_TRY = 3

    DISCLAIMER_FOR_PLAIN_LLM = """
This is LLM-self generated answer based on pre-existing knowledge, not directly related to LuxAI document.
"""

    def __init__(self, 
                 search_clients: defaultdict[SearchClient],
                 sourcepage_field: str, 
                 content_field: str,
                 embedding_field: str,
                 chatgpt_model = "", 
                 embed_model = "", 
                 chatgpt_deployment = "", 
                 embedding_deployment = "",
                 completion_deployment = "",
                 completion_model = "",
                 ):
        super().__init__()
        
        self.search_client_code = search_clients["code"]
        self.search_client_text = search_clients["text"]
        self.search_client_ros = search_clients["ros"]

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

        self.sub_rags = {
            self.RAGIntent.CODE: SubRAGCode(search_client=self.search_client_code, 
                               sourcepage_field=sourcepage_field,
                               content_field=content_field,
                               embedding_field=embedding_field,
                               chatgpt_model=chatgpt_model,
                               embed_model=embed_model,
                               chatgpt_deployment=chatgpt_deployment,
                               embedding_deployment=embedding_deployment,
                               completion_deployment=completion_deployment,
                               completion_model=completion_model),
            self.RAGIntent.TEXT: SubRAGText(search_client=self.search_client_text, 
                               sourcepage_field=sourcepage_field,
                               content_field=content_field,
                               embedding_field=embedding_field,
                               chatgpt_model=chatgpt_model,
                               embed_model=embed_model,
                               chatgpt_deployment=chatgpt_deployment,
                               embedding_deployment=embedding_deployment,
                               completion_deployment=completion_deployment,
                               completion_model=completion_model),
            self.RAGIntent.ROS: SubRAGText(search_client=self.search_client_ros, 
                               sourcepage_field=sourcepage_field,
                               content_field=content_field,
                               embedding_field=embedding_field,
                               chatgpt_model=chatgpt_model,
                               embed_model=embed_model,
                               chatgpt_deployment=chatgpt_deployment,
                               embedding_deployment=embedding_deployment,
                               completion_deployment=completion_deployment,
                               completion_model=completion_model),
        }

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

        history_list = self.__parse_history(history)

        history_list_str = json.dumps(history_list)

        messages = self.get_messages_from_history(
            ChatMultiSearchPrompt.system_message,
            self.chatgpt_model,
            [],
            ChatMultiSearchPrompt.user_message_template.format(message=history[-1]["user"], chat_history=history_list_str),
            ChatMultiSearchPrompt.few_shots
        )

        print(messages)

        chat_completion = self.__compute_chat_completion(messages=messages, 
                                                         temperature=0.0, 
                                                         max_tokens=self.chatgpt_token_limit, 
                                                         n=1)

        tokens_count = self.__count_tokens(chat_completion)
        extracted_requests = chat_completion.choices[0].message.content.strip().split("\n")

        # Check whether no questions found
        if len(extracted_requests) == 1 and extracted_requests[0] == "0":
            return tokens_count, []
        return tokens_count, extracted_requests

    def __classify_request_intent(self, request: str):
        # Check whether the query is asking for code generation
        request_vector = self.__compute_embedding(request)
        request_code_distance = cosine(IntentClassificationPrompt.request_code_vector, request_vector)
        print(f"Request code distance: {request_code_distance}")
        # If the request is a normal text generation
        if (request_code_distance > self.THRESHOLD_REQUEST_CODE):
            return self.RAGIntent.TEXT
        else:
            return self.RAGIntent.CODE

    def run(self, history: Sequence[dict[str, str]], overrides: dict[str, Any]) -> Any:
        all_answers = "" # Raw answer contains separated responses for requests
        check_logs = ""
        all_supporting_contents = ""
        tokens_count = 0
        retrieved_docs = []
        final_answer = ""
        contexts = []
        extracted_requests = []
        thoughts = ""

        has_answer = False

        # STEP 1: Extract requests in message
        num_tokens, extracted_requests = self.__extract_main_requests(history=history)
        tokens_count += num_tokens

        # Check whether no questions found
        if len(extracted_requests) > 0:
            for request in extracted_requests:
                print(f"Length of extracted_request: {len(extracted_requests)}")
                request = request.strip()
                if request == "":
                    continue
                
                sub_rags_stack = []
                request_intent = self.__classify_request_intent(request)

                # If it is more likely to be a code-related request, then try code stores first, otherwise do the reverse.
                if request_intent == self.RAGIntent.CODE:
                    sub_rags_stack.append(self.sub_rags[self.RAGIntent.CODE])
                    sub_rags_stack.append(self.sub_rags[self.RAGIntent.TEXT])
                    sub_rags_stack.append(self.sub_rags[self.RAGIntent.ROS])
                else:
                    sub_rags_stack.append(self.sub_rags[self.RAGIntent.TEXT])
                    sub_rags_stack.append(self.sub_rags[self.RAGIntent.ROS])
                    sub_rags_stack.append(self.sub_rags[self.RAGIntent.CODE])

                sub_answer = {}
                while sub_answer == {} or (len(sub_rags_stack) > 0 and sub_answer["answer"] == ""):
                    print(f"Number of rags in stack: {len(sub_rags_stack)}")
                    print(f"Request: {request}")
                    sub_rag = sub_rags_stack.pop(0)

                    sub_answer = sub_rag.run(history, overrides, query_text=request)

                if sub_answer["answer"] == "":
                    sub_answer["answer"] = ChatVectorComparePrompt.book_a_meeting_message
                else:
                    has_answer = True

                request_answer = self.LOG_ANSWER_FOR_REQUEST.format(query_text=request, chat_content=sub_answer["answer"])
                all_answers += request_answer

                retrieved_docs.extend(sub_answer["data_points"])
                thoughts += f"\n\n{request_answer}\n\n{sub_answer['supporting_contents']}"
                tokens_count += sub_answer["token_usage"]
                all_supporting_contents += f"\n---------------------\n{sub_answer['supporting_contents']}"

            # Combine answers to generate the final answer
            user_conv = ChatMultiSearchPrompt.user_content_merge_answer.format(customer_message=history[-1]['user'],
                                                                            raw_answer=all_answers)

            messages = [{"role":"system","content": ChatMultiSearchPrompt.system_message_merge_answer},
                        {"role":"user","content": user_conv}]
            
            chat_completion = self.__compute_chat_completion(messages=messages, 
                                                            temperature=overrides.get("temperature") or 0.0, 
                                                            max_tokens=self.chatgpt_token_limit, 
                                                            n=1)
                    
            tokens_count += self.__count_tokens(chat_completion)
            final_answer = chat_completion.choices[0].message.content
            # print(f"Final answer: {final_answer}")

            contexts = [all_answers]

            # msg_to_display = self.format_display_message(messages)
            # msg_to_display = self.format_display_message(text=msg_to_display)
            thoughts = self.format_display_message(text=thoughts)

        if len(extracted_requests) == 0 or not has_answer:
            print("No request or answer found!")
            user_msg = ChatGeneralPrompt.user_message_template.format(message=history[-1]["user"])  

            messages = self.get_messages_from_history(
                system_prompt=ChatGeneralPrompt.system_message,
                model_id=self.chatgpt_model,
                history=history,
                user_conv=user_msg,
                max_tokens=self.chatgpt_token_limit - len(user_msg))            

            chat_completion = self.__compute_chat_completion(messages=messages, temperature=0.7, max_tokens=self.chatgpt_token_limit, n=1)
            final_answer = chat_completion.choices[0].message.content            
            final_answer += "\n" + self.DISCLAIMER_FOR_PLAIN_LLM
            thoughts = self.format_display_message(messages)

        return {"data_points": retrieved_docs, 
                "answer": final_answer,
                "contexts": contexts,
                "supporting_contents": all_supporting_contents,
                "token_usage": tokens_count,
                "extracted_requests": extracted_requests,
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
    
    def __parse_history(self, history: Sequence[dict[str, str]]) -> list[dict]:
        history_list = []
        for h in reversed(history[:-1]):
            history_list.append({self.USER: h.get("user")})
            if h.get("bot"):
                history_list.append({self.ASSISTANT: h.get("bot")})
        return history_list

    def format_display_message(self, list_messages: list = None, text: str = "") -> str:
        msg_to_display = text
        if list_messages:
            msg_to_display = '\n=========================================\n\n'.join([str(message) for message in list_messages])
        msg_to_display = msg_to_display.replace('\\n', '<br>').replace('\n', '<br>')
        return msg_to_display