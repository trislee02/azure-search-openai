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

from approaches.promptutils import ChatGeneralPrompt

class ChatGeneral(Approach):
    # Chat roles
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"

    def __init__(self, 
                 chatgpt_model = "", 
                 chatgpt_deployment = "", 
                 embedding_deployment = ""):
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

    def run(self, history: Sequence[dict[str, str]], overrides: dict[str, Any]) -> Any:
        
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
        
        return {"data_points": {}, 
                "answer": chat_content, 
                "thoughts": f"Conversations:<br>" + msg_to_display}
    
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
    
    def format_display_message(self, list_messages: list) -> str:
        msg_to_display = '\n=========================================\n\n'.join([str(message) for message in list_messages])
        msg_to_display = msg_to_display.replace('\n', '<br>').replace('\\n', '<br>')
        return msg_to_display