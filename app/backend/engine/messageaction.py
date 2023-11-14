from typing import Any, Sequence

import os
import io
import mimetypes
import time
import logging
import openai
from azure.identity import DefaultAzureCredential
from azure.core.credentials import AzureKeyCredential
from azure.search.documents import SearchClient
from approaches.chatgeneral import ChatGeneral
from approaches.chatragteacherstudent import ChatRAGTeacherStudentApproach
from approaches.chatragvectorcompare import ChatRAGVectorCompareApproach
from approaches.chatragcomparetextncode import ChatRAGCompareTextAndCodeApproach
from approaches.chatragcomparetext import ChatRAGCompareTextApproach
from approaches.chatragmultisearch import ChatRAGMultiSearchApproach
from azure.storage.blob import BlobServiceClient

from .messageclassifier import ChatMessageClassifier

class ChatMessageAction:
    # Replace these with your own values, either in environment variables or directly here
    AZURE_STORAGE_ACCOUNT = os.environ.get("AZURE_STORAGE_ACCOUNT") or "mystorageaccount"
    AZURE_STORAGE_CONTAINER = os.environ.get("AZURE_STORAGE_CONTAINER") or "content"
    AZURE_SEARCH_SERVICE = os.environ.get("AZURE_SEARCH_SERVICE") or "gptkb"
    AZURE_SEARCH_INDEX = os.environ.get("AZURE_SEARCH_INDEX") or "gptkbindex"

    KB_FIELDS_CONTENT = os.environ.get("KB_FIELDS_CONTENT") or "content"
    KB_FIELDS_EMBEDDING = os.environ.get("KB_FIELDS_EMBEDDING") or "embedding"
    KB_FIELDS_CATEGORY = os.environ.get("KB_FIELDS_CATEGORY") or "category"
    KB_FIELDS_SOURCEPAGE = os.environ.get("KB_FIELDS_SOURCEPAGE") or "sourcepage"

    # Constants for OPENAI API
    # Add these azd environment in Configuration section of the App service on Azure Portal
    OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY") or ""
    OPENAI_GPT_MODEL = os.environ.get("OPENAI_GPT_MODEL") or "gpt-3.5-turbo"
    OPENAI_EMBED_MODEL = os.environ.get("OPENAI_EMBED_MODEL") or "text-embedding-ada-002"

    # Constants for AZURE OPENAI SERVICE
    # Add these azd environment in Configuration section of the App service on Azure Portal
    AZURE_OPENAI_KEY = os.environ.get("AZURE_OPENAI_KEY") or ""
    AZURE_OPENAI_SERVICE = os.environ.get("AZURE_OPENAI_SERVICE") or ""
    AZURE_OPENAI_GPT_DEPLOYMENT = os.environ.get("AZURE_OPENAI_GPT_35_DEPLOYMENT") or ""
    AZURE_OPENAI_CHATGPT_DEPLOYMENT = os.environ.get("AZURE_OPENAI_CHATGPT_35_DEPLOYMENT") or ""
    AZURE_OPENAI_CHATGPT_MODEL = os.environ.get("AZURE_OPENAI_CHATGPT_35_MODEL") or "gpt-3.5-turbo"
    AZURE_OPENAI_EMB_DEPLOYMENT = os.environ.get("AZURE_OPENAI_EMB_DEPLOYMENT") or ""
    AZURE_OPENAI_COMPLETION_DEPLOYMENT = os.environ.get("AZURE_OPENAI_COMPLETION_DEPLOYMENT") or ""
    AZURE_OPENAI_COMPLETION_MODEL = os.environ.get("AZURE_OPENAI_COMPLETION_DEPLOYMENT") or "gpt-3.5-turbo-instruct"

    # KEY
    AZURE_SEARCH_KEY = os.environ.get("AZURE_SEARCH_SERVICE_KEY") or ""
    AZURE_STORAGE_KEY = os.environ.get("AZURE_STORAGE_ACCOUNT_KEY") or ""

    # Use the current user identity to authenticate with Cognitive Search and Blob Storage (no secrets needed, 
    # just use 'az login' locally, and managed identity when deployed on Azure). If you need to use keys, use separate AzureKeyCredential instances with the 
    # keys for each service
    # If you encounter a blocking error during a DefaultAzureCredntial resolution, you can exclude the problematic credential by using a parameter (ex. exclude_shared_token_cache_credential=True)
    azure_credential = DefaultAzureCredential(exclude_shared_token_cache_credential = True)
    search_creds = azure_credential if AZURE_SEARCH_KEY == "" else AzureKeyCredential(AZURE_SEARCH_KEY)
    storage_creds = azure_credential if AZURE_STORAGE_KEY == "" else AZURE_STORAGE_KEY

    if OPENAI_API_KEY != "":
        openai.api_key = OPENAI_API_KEY
    else:
        # Used by the OpenAI SDK
        openai.api_type = "azure"
        openai.api_base = f"https://{AZURE_OPENAI_SERVICE}.openai.azure.com"
        openai.api_version = "2023-05-15"

        if AZURE_OPENAI_KEY != "":
            openai.api_key = AZURE_OPENAI_KEY
        else:
            openai.api_type = "azure_ad"
            openai_token = azure_credential.get_token("https://cognitiveservices.azure.com/.default")
            openai.api_key = openai_token.token

    # Set up clients for Cognitive Search and Storage
    search_client = SearchClient(
        endpoint=f"https://{AZURE_SEARCH_SERVICE}.search.windows.net",
        index_name=AZURE_SEARCH_INDEX,
        credential=search_creds)
    blob_client = BlobServiceClient(
        account_url=f"https://{AZURE_STORAGE_ACCOUNT}.blob.core.windows.net", 
        credential=storage_creds)
    blob_container = blob_client.get_container_client(AZURE_STORAGE_CONTAINER)

    chat_approaches = {
        "ts": ChatRAGTeacherStudentApproach(search_client, 
                                            KB_FIELDS_SOURCEPAGE, 
                                            KB_FIELDS_CONTENT,
                                            chatgpt_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                                            chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                                            embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT),
        "vc": ChatRAGVectorCompareApproach(search_client, 
                                    KB_FIELDS_SOURCEPAGE, 
                                    KB_FIELDS_CONTENT,
                                    KB_FIELDS_EMBEDDING,
                                    chatgpt_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                                    chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                                    embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT),
        "ctc": ChatRAGCompareTextAndCodeApproach(search_client, 
                                    KB_FIELDS_SOURCEPAGE, 
                                    KB_FIELDS_CONTENT,
                                    KB_FIELDS_EMBEDDING,
                                    chatgpt_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                                    chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                                    embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT),
        "ct": ChatRAGCompareTextApproach(search_client, 
                                    KB_FIELDS_SOURCEPAGE, 
                                    KB_FIELDS_CONTENT,
                                    KB_FIELDS_EMBEDDING,
                                    chatgpt_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                                    chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                                    embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT),
        "ms": ChatRAGMultiSearchApproach(search_client, 
                                    KB_FIELDS_SOURCEPAGE, 
                                    KB_FIELDS_CONTENT,
                                    KB_FIELDS_EMBEDDING,
                                    chatgpt_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                                    chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                                    embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT,
                                    completion_deployment=AZURE_OPENAI_COMPLETION_DEPLOYMENT,
                                    completion_model=AZURE_OPENAI_COMPLETION_MODEL),
        "g": ChatGeneral(chatgpt_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                         chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                         embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT)
    }

    def run(self, msg_type: str, history: Sequence[dict[str, str]], overrides: dict[str, Any]) -> Any:
        # if msg_type == ChatMessageClassifier.TYPE_OTHER:
        #     approach = self.chat_approaches["g"]
        # elif msg_type == ChatMessageClassifier.TYPE_QUESTION:
        #     approach = self.chat_approaches["ms"]
        approach = self.chat_approaches["ms"]
        r = approach.run(history, overrides)
        return r