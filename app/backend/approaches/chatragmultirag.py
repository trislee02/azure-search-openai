from typing import Any, Sequence
from collections import defaultdict

import openai
import tiktoken
import ast
import json
from azure.search.documents import SearchClient
from azure.search.documents.models import QueryType
from approaches.approach import Approach
from text import nonewlines

from core.messagebuilder import MessageBuilder
from core.modelhelper import get_token_limit

from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed

from approaches.promptutils import (
    ChatRAGPrompt, 
    ChatVectorComparePrompt, 
    ChatMultiSearchPrompt, 
    ChatGeneralPrompt, 
    IntentClassificationPrompt,
    RequestExtractionRefinePrompt
)

from scipy.spatial.distance import cosine

from core.modelhelper import num_tokens_from_chat_messages, count_tokens

from approaches.subrag.subragcode import SubRAGCode
from approaches.subrag.subragtext import SubRAGText
from approaches.subrag.subragcomposite import SubRAGComposite

class ChatRAGMultiRAGApproach(Approach):
    """
    This approach includes following steps:
        1. Extract main requests of message based on new message and chat history
        2. For each request, run intent classification
        3. Run sub-RAG stack for each request
        4. Merge all the responses to generate the final response
    """

    class SubRAGList:
        LUXAI = "luxai"
        REPO = "repo"
        ROS = "ros"
        EMAIL = "email"
        ISSUE = "issue"
        COMPOSITE_TEXT = "composite_text"
        COMPOSITE_CODE = "composite_code"

    class RequestIntent:
        NO_REQUEST = 0
        CODE_GENERATION = 1
        SCHEDULE_MEETING = 2
        OTHERS = 3

    # Chat roles
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"

    LOG_ANSWER_FOR_REQUEST = """
Question: {query_text}
Answer: {chat_content}
------------------
"""

    THRESHOLD_NO_ANSWER = 0.25

    MAX_TRY_REFINE_REQUESTS = 3

    DISCLAIMER_FOR_PLAIN_LLM = {"DISCLAIMER_FOR_PLAIN_LLM": """
This is LLM-self generated answer based on pre-existing knowledge, not directly related to LuxAI document.
"""}
    DISCLAIMER_SCHEDULING_MEETING = {"DISCLAIMER_SCHEDULING_MEETING": """
The customer requests to schedule a meeting which should be handled by human.
"""}

    def __init__(self, 
                 search_clients: defaultdict[SearchClient],
                 sourcepage_field: str, 
                 content_field: str,
                 embedding_field: str,
                 prefix_field: str,
                 chatgpt_model = "", 
                 embed_model = "", 
                 chatgpt_deployment = "", 
                 embedding_deployment = "",
                 completion_deployment = "",
                 completion_model = "",
                 ):
        super().__init__()
        
        self.search_client_repo = search_clients["repo"]
        self.search_client_issue = search_clients["issue"]
        self.search_client_luxai = search_clients["luxai"]
        self.search_client_ros = search_clients["ros"]
        self.search_client_email = search_clients["email"]
        
        search_components_code = [SubRAGComposite.SearchComponent(search_client=self.search_client_luxai),
                                  SubRAGComposite.SearchComponent(search_client=self.search_client_repo, use_reranker=False)]
        search_components_text = [SubRAGComposite.SearchComponent(search_client=self.search_client_luxai),
                                  SubRAGComposite.SearchComponent(search_client=self.search_client_email),
                                  SubRAGComposite.SearchComponent(search_client=self.search_client_issue),]

        self.sourcepage_field = sourcepage_field
        self.content_field = content_field
        self.embedding_field = embedding_field
        self.prefix_field = prefix_field
        self.embed_model = embed_model
        self.chatgpt_deployment = chatgpt_deployment
        self.chatgpt_model = chatgpt_model
        self.embedding_deployment = embedding_deployment
        self.chatgpt_token_limit = get_token_limit(chatgpt_model)
        self.completion_deployment = completion_deployment
        self.completion_model = completion_model

        self.sub_rags = {
            self.SubRAGList.REPO: SubRAGCode(search_client=self.search_client_repo, 
                               sourcepage_field=sourcepage_field,
                               content_field=content_field,
                               embedding_field=embedding_field,
                               chatgpt_model=chatgpt_model,
                               embed_model=embed_model,
                               chatgpt_deployment=chatgpt_deployment,
                               embedding_deployment=embedding_deployment,
                               completion_deployment=completion_deployment,
                               completion_model=completion_model),
            self.SubRAGList.ISSUE: SubRAGText(search_client=self.search_client_issue, 
                               sourcepage_field=sourcepage_field,
                               content_field=content_field,
                               embedding_field=embedding_field,
                               prefix_field=prefix_field,
                               chatgpt_model=chatgpt_model,
                               embed_model=embed_model,
                               chatgpt_deployment=chatgpt_deployment,
                               embedding_deployment=embedding_deployment,
                               completion_deployment=completion_deployment,
                               completion_model=completion_model),
            self.SubRAGList.LUXAI: SubRAGText(search_client=self.search_client_luxai, 
                               sourcepage_field=sourcepage_field,
                               content_field=content_field,
                               embedding_field=embedding_field,
                               prefix_field=prefix_field,
                               chatgpt_model=chatgpt_model,
                               embed_model=embed_model,
                               chatgpt_deployment=chatgpt_deployment,
                               embedding_deployment=embedding_deployment,
                               completion_deployment=completion_deployment,
                               completion_model=completion_model),
            self.SubRAGList.ROS: SubRAGText(search_client=self.search_client_ros, 
                               sourcepage_field=sourcepage_field,
                               content_field=content_field,
                               embedding_field=embedding_field,
                               prefix_field=prefix_field,
                               chatgpt_model=chatgpt_model,
                               embed_model=embed_model,
                               chatgpt_deployment=chatgpt_deployment,
                               embedding_deployment=embedding_deployment,
                               completion_deployment=completion_deployment,
                               completion_model=completion_model),
            self.SubRAGList.EMAIL: SubRAGText(search_client=self.search_client_email, 
                               sourcepage_field=sourcepage_field,
                               content_field=content_field,
                               embedding_field=embedding_field,
                               prefix_field=prefix_field,
                               chatgpt_model=chatgpt_model,
                               embed_model=embed_model,
                               chatgpt_deployment=chatgpt_deployment,
                               embedding_deployment=embedding_deployment,
                               completion_deployment=completion_deployment,
                               completion_model=completion_model),
            self.SubRAGList.COMPOSITE_TEXT: SubRAGComposite(search_components=search_components_text,
                                                        sourcepage_field=sourcepage_field,
                                                        content_field=content_field,
                                                        embedding_field=embedding_field,
                                                        prefix_field=prefix_field,
                                                        chatgpt_model=chatgpt_model,
                                                        embed_model=embed_model,
                                                        chatgpt_deployment=chatgpt_deployment,
                                                        embedding_deployment=embedding_deployment,
                                                        completion_deployment=completion_deployment,
                                                        completion_model=completion_model),
            self.SubRAGList.COMPOSITE_CODE: SubRAGComposite(search_components=search_components_code,
                                                        sourcepage_field=sourcepage_field,
                                                        content_field=content_field,
                                                        embedding_field=embedding_field,
                                                        prefix_field=prefix_field,
                                                        chatgpt_model=chatgpt_model,
                                                        embed_model=embed_model,
                                                        chatgpt_deployment=chatgpt_deployment,
                                                        embedding_deployment=embedding_deployment,
                                                        completion_deployment=completion_deployment,
                                                        completion_model=completion_model),
        }

    def before_retry_sleep(retry_state):
        try:
            print(f"Rate limited on the OpenAI API, sleeping before retrying...")
            retry_state.outcome.result()
        except Exception as e:
            print(e)

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
    
    def __check_request(self, request: str):
        """
        Check whether a request provides sufficient context using LLM
        Return:
            bool: whether a request provides sufficient context or not
            int: token usage
        """
        messages = self.get_messages_from_history(
            RequestExtractionRefinePrompt.system_message_check,
            self.chatgpt_model,
            [],
            RequestExtractionRefinePrompt.user_content_template_check.format(request=request)
        )

        chat_completion = self.__compute_chat_completion(messages=messages, 
                                                         temperature=0.0, 
                                                         max_tokens=self.chatgpt_token_limit, 
                                                         n=1)

        tokens_count = count_tokens(chat_completion)
        reasoning = chat_completion.choices[0].message.content

        if RequestExtractionRefinePrompt.SUFFICIENT_TOKEN in reasoning:
            return True, tokens_count
        else:
            return False, tokens_count
    
    def __refine_request(self, request: str, message: str, chat_history: str):
        """
        Refine a request by elaborating on the context
        Return:
            str: refined request
            int: token usage
        """
        messages = self.get_messages_from_history(
            RequestExtractionRefinePrompt.system_message_refine,
            self.chatgpt_model,
            [],
            RequestExtractionRefinePrompt.user_content_template_refine.format(original_request=request,
                                                                              current_message=message,
                                                                              conversation_history=chat_history)
        )

        chat_completion = self.__compute_chat_completion(messages=messages, 
                                                         temperature=0.0, 
                                                         max_tokens=self.chatgpt_token_limit, 
                                                         n=1)

        tokens_count = count_tokens(chat_completion)
        refined_request = chat_completion.choices[0].message.content
        return refined_request, tokens_count

    def __extract_main_requests(self, history: Sequence[dict[str, str]]):
        """
        Extract main requests from the latest message based on the conversation history.
        Also, refine the initial generated request if it provides insufficient context.
        Return:
            int: token usage
            list[str]: list of extracted requests
        """
        tokens_count = 0

        history_list = self.__parse_history(history)

        history_list_str = json.dumps(history_list)

        messages = self.get_messages_from_history(
            ChatMultiSearchPrompt.system_message,
            self.chatgpt_model,
            [],
            ChatMultiSearchPrompt.user_message_template.format(message=history[-1]["user"], chat_history=history_list_str),
            ChatMultiSearchPrompt.few_shots
        )

        chat_completion = self.__compute_chat_completion(messages=messages, 
                                                         temperature=0.0, 
                                                         max_tokens=self.chatgpt_token_limit, 
                                                         n=1)

        tokens_count += count_tokens(chat_completion)
        chat_content = chat_completion.choices[0].message.content.strip()
        # chat_content should be a string of list[string]
        try:
            extracted_requests = ast.literal_eval(chat_content)
        except:
            extracted_requests = chat_content.split("\n")

        # Self-refining request
        # Check understandability and refine extracted requests
        extracted_requests, tokens = self.self_refine_request(history[-1]["user"], history_list_str, extracted_requests)
        tokens_count += tokens

        # Check whether no questions found
        if len(extracted_requests) == 1 and extracted_requests[0] == "0":
            return tokens_count, []
        return tokens_count, extracted_requests

    def self_refine_request(self, message: str, history_list_str: str, extracted_requests: list[str]) -> list[str]:
        tokens_count = 0
        for i, request in enumerate(extracted_requests):
            understandable = False # A request that is understandable has sufficient context in that.
            tries = 0
            while not understandable and tries < self.MAX_TRY_REFINE_REQUESTS:
                if tries > 0:
                    response, tokens = self.__refine_request(request=request,
                                                    message=message,
                                                    chat_history=history_list_str)   
                    tokens_count += tokens
                    try:
                        # Response should be a dictionry
                        response = ast.literal_eval(response)
                        request = response["refined_request"]
                    except:
                        # Because temperature is 0, so need a different request for a different response later
                        request = request + "."

                understandable, tokens = self.__check_request(request)
                tokens_count += tokens
                tries += 1
            extracted_requests[i] = request
        return extracted_requests, tokens_count

    def __classify_request_intent(self, request: str):
        """
        Classifiy the intent of request by using LLM
        Return:
            str: intent of request
            int: token usage
        """
        user_conv = IntentClassificationPrompt.user_content_template.format(request=request)
        messages = [{"role":"system","content": IntentClassificationPrompt.system_message},
                    {"role":"user","content": user_conv}]
        chat_completion = self.__compute_chat_completion(messages=messages, 
                                                        temperature=0.0, 
                                                        max_tokens=self.chatgpt_token_limit, 
                                                        n=1)
        tokens_count = count_tokens(chat_completion)
        chat_content = chat_completion.choices[0].message.content.strip()
        try:
            intent = int(chat_content.split("[Intent]:")[-1])
        except:
            intent = 3
        
        return intent, tokens_count   

    def run(self, history: Sequence[dict[str, str]], overrides: dict[str, Any]) -> Any:
        all_answers = "" # Raw answer contains separated responses for requests
        all_supporting_contents = ""
        tokens_count = 0
        retrieved_docs = []
        final_answer = ""
        contexts = []
        extracted_requests = []
        thoughts = ""
        followup_message = {}

        overrides["retrieval_mode"] = "hybrid" # Use vectors for retrieval only. Choices: "hybrid", "text", "vectors"

        has_answer = False # Whether the LLM finds answer to respond

        # STEP 1: Extract requests in message
        num_tokens, extracted_requests = self.__extract_main_requests(history=history)
        tokens_count += num_tokens

        # Check whether no questions found
        if len(extracted_requests) > 0:
            for index, request in enumerate(extracted_requests):
                request = request.strip()
                if request == "":
                    continue
                
                sub_rags_stack = []
                request_intent, num_tokens = self.__classify_request_intent(request)
                tokens_count += num_tokens
                # If it is more likely to be a code-related request, then try code stores first, otherwise do the reverse.
                if request_intent == self.RequestIntent.CODE_GENERATION:
                    # sub_rags_stack.append(self.sub_rags[self.SubRAGList.COMPOSITE_CODE])
                    sub_rags_stack.append(self.sub_rags[self.SubRAGList.REPO])
                    sub_rags_stack.append(self.sub_rags[self.SubRAGList.LUXAI])
                elif request_intent == self.RequestIntent.OTHERS:
                    # sub_rags_stack.append(self.sub_rags[self.SubRAGList.EMAIL])
                    sub_rags_stack.append(self.sub_rags[self.SubRAGList.COMPOSITE_TEXT])
                    # sub_rags_stack.append(self.sub_rags[self.SubRAGList.ISSUE])
                    sub_rags_stack.append(self.sub_rags[self.SubRAGList.ROS])
                elif request_intent == self.RequestIntent.SCHEDULE_MEETING:
                    followup_message.update(self.DISCLAIMER_SCHEDULING_MEETING)
                    continue
                else: # No request
                    continue         

                # Try to find answer using sub-rags
                sub_answer = {}
                while sub_answer == {} or (len(sub_rags_stack) > 0 and sub_answer["answer"] == ""):
                    sub_rag = sub_rags_stack.pop(0)
                    sub_answer = sub_rag.run(history, overrides, query_text=request)

                if sub_answer["answer"] == "":
                    sub_answer["answer"] = ChatVectorComparePrompt.book_a_meeting_message
                    followup_message[f"Request {index}"] = f"""Cannot respond to the request: {request}"""""
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
                                                            temperature=ChatMultiSearchPrompt.temperature_merge_answer, 
                                                            max_tokens=self.chatgpt_token_limit, 
                                                            n=1)
            
            tokens_count += count_tokens(chat_completion)
            final_answer = chat_completion.choices[0].message.content

            contexts = [all_answers]
            thoughts = self.format_display_message(text=thoughts)

        # Use plain LLM to generate answer if no answer found or no request extracted
        if len(extracted_requests) == 0 or not has_answer:
            user_msg = ChatGeneralPrompt.user_message_template.format(message=history[-1]["user"])  

            messages = self.get_messages_from_history(
                system_prompt=ChatGeneralPrompt.system_message,
                model_id=self.chatgpt_model,
                history=history,
                user_conv=user_msg,
                max_tokens=self.chatgpt_token_limit - len(user_msg))            

            chat_completion = self.__compute_chat_completion(messages=messages, temperature=0.7, max_tokens=self.chatgpt_token_limit, n=1)
            final_answer = chat_completion.choices[0].message.content  
            followup_message.update(self.DISCLAIMER_FOR_PLAIN_LLM)       
            thoughts = self.format_display_message(messages)
            contexts = []
            all_supporting_contents = ""

        if len(followup_message) != 0:
            followup_message_str = "\n".join(followup_message.values())
            final_answer += "\n---------------\n" + followup_message_str

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