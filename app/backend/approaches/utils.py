import openai

from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed
from core.modelhelper import num_tokens_from_chat_messages

def before_retry_sleep(retry_state):
    try:
        print(f"Rate limited on the OpenAI API, sleeping before retrying...")
        retry_state.outcome.result()
    except Exception as e:
        print(e)

@retry(wait=wait_random_exponential(min=15, max=60), stop=stop_after_attempt(5), before_sleep=before_retry_sleep)
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