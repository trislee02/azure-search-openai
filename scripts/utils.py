import openai
import re
import base64
from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed

def before_retry_sleep(retry_state):
    print(f"Rate limited on the OpenAI embeddings API, sleeping before retrying...")

@retry(wait=wait_random_exponential(min=20, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
def compute_embedding(azure_openai_embed_deployment: str, text: str):
    embed = openai.Embedding.create(engine=azure_openai_embed_deployment, input=text)["data"][0]["embedding"]
    if embed: print("Successfully embedded")
    return embed

def filename_to_id(filename: str):
    filename_ascii = re.sub("[^0-9a-zA-Z_-]", "_", filename)
    filename_hash = base64.b16encode(filename.encode('utf-8')).decode('ascii')
    return f"file-{filename_ascii}-{filename_hash}"