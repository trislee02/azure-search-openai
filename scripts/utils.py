import re
import os
import hashlib
import base64
from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed
from openai import AzureOpenAI
from dotenv import load_dotenv

load_dotenv()

AZURE_OPENAI_GPT_DEPLOYMENT = os.environ.get("AZURE_OPENAI_GPT_35_DEPLOYMENT") or ""
AZURE_OPENAI_EMB_DEPLOYMENT = os.environ.get("AZURE_OPENAI_EMB_DEPLOYMENT") or ""
AZURE_OPENAI_KEY = os.environ.get("AZURE_OPENAI_KEY") or ""
OPENAI_API_VERSION = os.environ.get("OPENAI_API_VERSION") or ""
AZURE_OPENAI_SERVICE = os.environ.get("AZURE_OPENAI_SERVICE") or ""
AZURE_ENDPOINT = f"https://{AZURE_OPENAI_SERVICE}.openai.azure.com"

openai_client = AzureOpenAI(api_key=AZURE_OPENAI_KEY,  
                            api_version=OPENAI_API_VERSION,
                            azure_endpoint=AZURE_ENDPOINT)

def before_retry_sleep(retry_state):
    try:
        print(f"Rate limited on the OpenAI API, sleeping before retrying...")
        retry_state.outcome.result()
    except Exception as e:
        print(e)

@retry(wait=wait_random_exponential(min=20, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
def compute_embedding(text: str):
    embed = openai_client.embeddings.create(model=AZURE_OPENAI_EMB_DEPLOYMENT, input=[text]).data[0].embedding
    if embed: print("Successfully embedded")
    return embed

def filename_to_id(filename: str):
    filename_ascii = re.sub("[^0-9a-zA-Z_-]", "_", filename)
    filename_hash = base64.b16encode(filename.encode('utf-8')).decode('ascii')
    return f"file-{filename_ascii}-{filename_hash}"

def compute_chatcompletion(messages: list, temperature: float = 0.7) -> str:
    response = openai_client.chat.completions.create(model=AZURE_OPENAI_GPT_DEPLOYMENT,
                                                            messages=messages,
                                                            temperature=temperature,
                                                            top_p=0.95,
                                                            frequency_penalty=0,
                                                            presence_penalty=0,
                                                            stop=None)
    
    return response.choices[0].message.content

def url_to_hash(url: str) -> str:
    data = url.encode()
    sha256_hash = hashlib.sha256(data).hexdigest()
    return sha256_hash
