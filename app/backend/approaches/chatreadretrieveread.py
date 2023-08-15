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

from approaches.promptutils import ChatPrompt

class ChatReadRetrieveReadApproach(Approach):
    # Chat roles
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"

    def __init__(self, 
                 search_client: SearchClient, 
                 sourcepage_field: str, 
                 content_field: str,
                 chatgpt_model = "", 
                 embed_model = "", 
                 chatgpt_deployment = "", 
                 embedding_deployment = ""):
        self.search_client = search_client
        self.sourcepage_field = sourcepage_field
        self.content_field = content_field
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
            ChatPrompt.query_prompt_template,
            self.chatgpt_model,
            history,
            user_q,
            ChatPrompt.query_prompt_few_shots,
            self.chatgpt_token_limit - len(user_q)
            )
        msg_need_display = messages # Debugging

        chat_completion = self.__compute_chat_completion(messages=messages, temperature=0.0, max_tokens=32, n=1)
                
        query_text = chat_completion.choices[0].message.content
        if query_text.strip() == "0":
            query_text = history[-1]["user"] # Use the last user input if we failed to generate a better query

        # STEP 2: Retrieve relevant documents from the search index with the GPT optimized query

        # If retrieval mode includes vectors, compute an embedding for the query
        if has_vector:
            if self.embedding_deployment != "":
                query_vector = openai.Embedding.create(engine=self.embedding_deployment, input=query_text)["data"][0]["embedding"]                
            else:
                query_vector = openai.Embedding.create(input=query_text, model=self.embed_model)["data"][0]["embedding"]
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
        if use_semantic_captions:
            results = [f"[{doc[self.sourcepage_field]}]" + ": " + nonewlines(" . ".join([c.text for c in doc['@search.captions']])) for doc in r]
        else:
            results = [f"[{doc[self.sourcepage_field]}]" + ": " + nonewlines(doc[self.content_field]) for doc in r]
        content = "\n".join(results)

        follow_up_questions_prompt = ChatPrompt.follow_up_questions_prompt_content if overrides.get("suggest_followup_questions") else ""
        
        # STEP 3: Generate a contextual and content specific answer using the search results and chat history

        # Allow client to replace the entire prompt, or to inject into the existing prompt using >>>
        prompt_override = overrides.get("prompt_override")
        if prompt_override is None:
            system_message = ChatPrompt.system_message_chat_conversation.format(injected_prompt="", follow_up_questions_prompt=follow_up_questions_prompt)
        elif prompt_override.startswith(">>>"):
            system_message = ChatPrompt.system_message_chat_conversation.format(injected_prompt=prompt_override[3:] + "\n", follow_up_questions_prompt=follow_up_questions_prompt)
        else:
            system_message = prompt_override.format(follow_up_questions_prompt=follow_up_questions_prompt)
        
        user_conv = f"""Source: 
<source>
    {content}
</source>

Question: {history[-1]["user"]}
"""

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

        ## STEP 4: Post-checking bot response for hallucinations, attitude
        response_to_check = f"Text message:\n```\n{chat_content}\n```\n\nSupporting content:\n```\n{content}\n```"
                
        chat_completion = self.__compute_chat_completion(messages=[{"role":"system","content": ChatPrompt.system_message_check_response},
                                                                   {"role":"user","content": response_to_check}], 
                                                         temperature=overrides.get("temperature") or 0.0, 
                                                         max_tokens=1024, 
                                                         n=1)
        check_result = chat_completion.choices[0].message.content
        print(f"Check result: {check_result}")

        msg_to_display = '=========================================\n\n'.join([str(message) for message in messages])
        msg_to_display = msg_need_display
        msg_to_display = msg_to_display.replace('\n', '<br>').replace('\\n', '<br>').replace('\\', '')

        return {"data_points": results, "answer": chat_content, "thoughts": f"Searched for:<br>{query_text}<br><br>Conversations:<br>" + msg_to_display}
    
    def get_messages_from_history(self, system_prompt: str, model_id: str, history: Sequence[dict[str, str]], user_conv: str, few_shots = [], max_tokens: int = 4096) -> []:
        message_builder = MessageBuilder(system_prompt, model_id)

        # Add examples to show the chat what responses we want. It will try to mimic any responses and make sure they match the rules laid out in the system message.
        for shot in few_shots:
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