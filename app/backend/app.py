import os
import io
import mimetypes
import time
import logging
import openai
from flask import Flask, request, jsonify, send_file, abort, redirect
from azure.identity import DefaultAzureCredential
from azure.core.credentials import AzureKeyCredential
from azure.search.documents import SearchClient
from approaches.retrievethenread import RetrieveThenReadApproach
from approaches.readretrieveread import ReadRetrieveReadApproach
from approaches.readdecomposeask import ReadDecomposeAsk
from approaches.chatgeneral import ChatGeneral
from approaches.chatragteacherstudent import ChatRAGTeacherStudentApproach
from approaches.chatragvectorcompare import ChatRAGVectorCompareApproach
from approaches.chatragcomparetextncode import ChatRAGCompareTextAndCodeApproach
from approaches.chatragcomparetext import ChatRAGCompareTextApproach
from approaches.chatragmultisearch import ChatRAGMultiSearchApproach
from approaches.chatragmultirag import ChatRAGMultiRAGApproach
from azure.storage.blob import BlobServiceClient

# Replace these with your own values, either in environment variables or directly here
AZURE_STORAGE_ACCOUNT = os.environ.get("AZURE_STORAGE_ACCOUNT") or "mystorageaccount"
AZURE_STORAGE_CONTAINER = os.environ.get("AZURE_STORAGE_CONTAINER") or "content"
AZURE_SEARCH_SERVICE = os.environ.get("AZURE_SEARCH_SERVICE") or "gptkb"

AZURE_SEARCH_INDEX_LUXAI = os.environ.get("AZURE_SEARCH_INDEX_LUXAI") or "gptkbindex-luxai"
AZURE_SEARCH_INDEX_EMAIL = os.environ.get("AZURE_SEARCH_INDEX_EMAIL") or "gptkbindex-email"
AZURE_SEARCH_INDEX_CODE = os.environ.get("AZURE_SEARCH_INDEX_CODE") or "gptkbindex-code"
AZURE_SEARCH_INDEX_REPO = os.environ.get("AZURE_SEARCH_INDEX_REPO") or "gptkbindex-repo"
AZURE_SEARCH_INDEX_ISSUE = os.environ.get("AZURE_SEARCH_INDEX_ISSUE") or "gptkbindex-issue"
AZURE_SEARCH_INDEX_ROS = os.environ.get("AZURE_SEARCH_INDEX_ROS") or "gptkbindex-ros"

KB_FIELDS_CONTENT = os.environ.get("KB_FIELDS_CONTENT") or "content"
KB_FIELDS_EMBEDDING = os.environ.get("KB_FIELDS_EMBEDDING") or "embedding"
KB_FIELDS_PREFIX = os.environ.get("KB_FIELDS_PREFIX") or "prefix"
KB_FIELDS_SOURCEPAGE = os.environ.get("KB_FIELDS_SOURCEPAGE") or "sourcefile"

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

OPENAI_API_VERSION = os.environ.get("OPENAI_API_VERSION")
# Use the current user identity to authenticate with Cognitive Search and Blob Storage (no secrets needed, 
# just use 'az login' locally, and managed identity when deployed on Azure). If you need to use keys, use separate AzureKeyCredential instances with the 
# keys for each service
# If you encounter a blocking error during a DefaultAzureCredntial resolution, you can exclude the problematic credential by using a parameter (ex. exclude_shared_token_cache_credential=True)
azure_credential = DefaultAzureCredential(exclude_shared_token_cache_credential = True)
search_creds = azure_credential if AZURE_SEARCH_KEY == "" else AzureKeyCredential(AZURE_SEARCH_KEY)
storage_creds = azure_credential if AZURE_STORAGE_KEY == "" else AZURE_STORAGE_KEY

openai_token = None
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
    index_name=AZURE_SEARCH_INDEX_LUXAI,
    credential=search_creds)
search_client_repo = SearchClient(
    endpoint=f"https://{AZURE_SEARCH_SERVICE}.search.windows.net",
    index_name=AZURE_SEARCH_INDEX_REPO,
    credential=search_creds)
search_client_issue = SearchClient(
    endpoint=f"https://{AZURE_SEARCH_SERVICE}.search.windows.net",
    index_name=AZURE_SEARCH_INDEX_ISSUE,
    credential=search_creds)
search_client_ros = SearchClient(
    endpoint=f"https://{AZURE_SEARCH_SERVICE}.search.windows.net",
    index_name=AZURE_SEARCH_INDEX_ROS,
    credential=search_creds)
search_client_email = SearchClient(
    endpoint=f"https://{AZURE_SEARCH_SERVICE}.search.windows.net",
    index_name=AZURE_SEARCH_INDEX_EMAIL,
    credential=search_creds)

blob_client = BlobServiceClient(
    account_url=f"https://{AZURE_STORAGE_ACCOUNT}.blob.core.windows.net", 
    credential=storage_creds)
blob_container = blob_client.get_container_client(AZURE_STORAGE_CONTAINER)

search_clients = {
    "luxai": search_client,
    "repo": search_client_repo,
    "issue": search_client_issue,
    "ros": search_client_ros,
    "email": search_client_email
}

# Various approaches to integrate GPT and external knowledge, most applications will use a single one of these patterns
# or some derivative, here we include several for exploration purposes
ask_approaches = {
    "rtr": RetrieveThenReadApproach(search_client, KB_FIELDS_SOURCEPAGE, KB_FIELDS_CONTENT,
                                    openai_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                                    chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                                    embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT),
    "rrr": ReadRetrieveReadApproach(search_client, KB_FIELDS_SOURCEPAGE, KB_FIELDS_CONTENT,
                                    openai_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                                    embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT),
    "rda": ReadDecomposeAsk(search_client, OPENAI_GPT_MODEL, OPENAI_EMBED_MODEL, KB_FIELDS_SOURCEPAGE, KB_FIELDS_CONTENT)
}

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
    "mr": ChatRAGMultiRAGApproach(search_clients,
                                    KB_FIELDS_SOURCEPAGE, 
                                    KB_FIELDS_CONTENT,
                                    KB_FIELDS_EMBEDDING,
                                    KB_FIELDS_PREFIX,
                                    chatgpt_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                                    chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                                    embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT,
                                    completion_deployment=AZURE_OPENAI_COMPLETION_DEPLOYMENT,
                                    completion_model=AZURE_OPENAI_COMPLETION_MODEL),
    "g": ChatGeneral(chatgpt_deployment=AZURE_OPENAI_CHATGPT_DEPLOYMENT,
                        chatgpt_model=AZURE_OPENAI_CHATGPT_MODEL,
                        embedding_deployment=AZURE_OPENAI_EMB_DEPLOYMENT)
}

app = Flask(__name__)

@app.route("/", defaults={"path": "index.html"})
@app.route("/<path:path>")
def static_file(path):
    return app.send_static_file(path)

# Serve content files from blob storage from within the app to keep the example self-contained. 
# *** NOTE *** this assumes that the content files are public, or at least that all users of the app
# can access all the files. This is also slow and memory hungry.
@app.route("/content/<path>")
def content_file(path):
    blob = blob_container.get_blob_client(path).download_blob()
    if not blob.properties or not blob.properties.has_key("content_settings"):
        # abort(404)
        return redirect(path)
    mime_type = blob.properties["content_settings"]["content_type"]
    if mime_type == "application/octet-stream":
        mime_type = mimetypes.guess_type(path)[0] or "application/octet-stream"
    blob_file = io.BytesIO()
    blob.readinto(blob_file)
    blob_file.seek(0)
    return send_file(blob_file, mimetype=mime_type, as_attachment=False, download_name=path)
    
@app.route("/ask", methods=["POST"])
def ask():
    ensure_openai_token()
    if not request.json:
        return jsonify({"error": "request must be json"}), 400
    approach = request.json["approach"]
    try:
        impl = ask_approaches.get(approach)
        if not impl:
            return jsonify({"error": "unknown approach"}), 400
        r = impl.run(request.json["question"], request.json.get("overrides") or {})
        return jsonify(r)
    except Exception as e:
        logging.exception("Exception in /ask")
        return jsonify({"error": str(e)}), 500
    
@app.route("/chat", methods=["POST"])
def chat():
    ensure_openai_token()
    if not request.json:
        return jsonify({"error": "request must be json"}), 400
    try:
        approach = chat_approaches["mr"]
        r = approach.run(request.json["history"], request.json.get("overrides") or {})
        
        return jsonify(r)
    except Exception as e:
        logging.exception("Exception in /chat")
        return jsonify({"error": str(e)}), 500

def ensure_openai_token():
    global openai_token
    if openai_token and openai_token.expires_on < int(time.time()) - 60:
        openai_token = azure_credential.get_token("https://cognitiveservices.azure.com/.default")
        openai.api_key = openai_token.token
    
if __name__ == "__main__":
    app.run()
