import argparse
import base64
import glob
import html
import io
import os
import re
import time

import openai
from azure.ai.formrecognizer import DocumentAnalysisClient
from azure.core.credentials import AzureKeyCredential
from azure.identity import AzureDeveloperCliCredential, DefaultAzureCredential
from azure.search.documents import SearchClient
from azure.search.documents.indexes import SearchIndexClient
from azure.search.documents.indexes.models import (
    HnswParameters,
    PrioritizedFields,
    SearchableField,
    SearchField,
    SearchFieldDataType,
    SearchIndex,
    SemanticConfiguration,
    SemanticField,
    SemanticSettings,
    SimpleField,
    VectorSearch,
    VectorSearchAlgorithmConfiguration,
)
from azure.storage.blob import BlobServiceClient
from pypdf import PdfReader, PdfWriter
from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed

from splitter.splitter import Splitter
from splitter.markdownsplitter import MarkdownSplitter
from splitter.codesplitter import CodeSplitter
from splitter.emailsplitter import EmailSplitter

MAX_SECTION_LENGTH = 1000
SENTENCE_SEARCH_LIMIT = 100
SECTION_OVERLAP = 100

def blob_name_from_file_page(filename, page = 0):
    if os.path.splitext(filename)[1].lower() == ".pdf":
        return os.path.splitext(os.path.basename(filename))[0] + f"-{page}" + ".pdf"
    else:
        return os.path.basename(filename)

def upload_blobs(filename):
    blob_service = BlobServiceClient(account_url=f"https://{args.storageaccount}.blob.core.windows.net", credential=storage_creds)
    blob_container = blob_service.get_container_client(args.container)
    if not blob_container.exists():
        blob_container.create_container()

    # if file is PDF split into pages and upload each page as a separate blob
    if os.path.splitext(filename)[1].lower() == ".pdf":
        reader = PdfReader(filename)
        pages = reader.pages
        for i in range(len(pages)):
            blob_name = blob_name_from_file_page(filename, i)
            if args.verbose: print(f"\tUploading blob for page {i} -> {blob_name}")
            f = io.BytesIO()
            writer = PdfWriter()
            writer.add_page(pages[i])
            writer.write(f)
            f.seek(0)
            blob_container.upload_blob(blob_name, f, overwrite=True)
    else:
        blob_name = blob_name_from_file_page(filename)
        with open(filename,"rb") as data:
            if args.verbose: print(f"\tUploading blob for file {filename} -> {blob_name}")
            blob_container.upload_blob(blob_name, data, overwrite=True)

def remove_blobs(filename):
    if args.verbose: print(f"Removing blobs for '{filename or '<all>'}'")
    blob_service = BlobServiceClient(account_url=f"https://{args.storageaccount}.blob.core.windows.net", credential=storage_creds)
    blob_container = blob_service.get_container_client(args.container)
    if blob_container.exists():
        if filename == None:
            blobs = blob_container.list_blob_names()
        else:
            prefix = os.path.splitext(os.path.basename(filename))[0]
            blobs = filter(lambda b: re.match(f"{prefix}-\d+\.pdf", b), blob_container.list_blob_names(name_starts_with=os.path.splitext(os.path.basename(prefix))[0]))
        for b in blobs:
            if args.verbose: print(f"\tRemoving blob {b}")
            blob_container.delete_blob(b)

def load_and_split(filename: str, splitters: dict[str, Splitter]) -> dict:
    

    return splitter.load_and_split(filename)

def before_retry_sleep(retry_state):
    if args.verbose: print(f"Rate limited on the OpenAI embeddings API, sleeping before retrying...")

@retry(wait=wait_random_exponential(min=20, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
def compute_embedding(text):
    if args.openaiapikey:
        embed = openai.Embedding.create(input=text, model=args.openaiembedmodel)["data"][0]["embedding"]
    else:
        embed = openai.Embedding.create(engine=args.openaiembeddingdeployment, input=text)["data"][0]["embedding"]
    if embed and args.verbose: print("Successfully embedded")
    return embed

def check_index_exist(index_client, index_name):
    if index_name not in index_client.list_index_names():
        index = SearchIndex(
            name=index_name,
            fields=[
                SimpleField(name="id", type="Edm.String", key=True),
                SearchableField(name="content", type="Edm.String", analyzer_name="en.microsoft"),
                SearchField(name="embedding", type=SearchFieldDataType.Collection(SearchFieldDataType.Single), 
                            hidden=False, searchable=True, filterable=False, sortable=False, facetable=False,
                            vector_search_dimensions=1536, vector_search_configuration="default"),
                SimpleField(name="category", type="Edm.String", filterable=True, facetable=True),
                SimpleField(name="sourcepage", type="Edm.String", filterable=True, facetable=True),
                SimpleField(name="sourcefile", type="Edm.String", filterable=True, facetable=True),
                SimpleField(name="metadata", type="Edm.String", filterable=True, facetable=True)
            ],
            semantic_settings=SemanticSettings(
                configurations=[SemanticConfiguration(
                    name='default',
                    prioritized_fields=PrioritizedFields(
                        title_field=None, prioritized_content_fields=[SemanticField(field_name='content')]))]),
                vector_search=VectorSearch(
                    algorithm_configurations=[
                        VectorSearchAlgorithmConfiguration(
                            name="default",
                            kind="hnsw",
                            hnsw_parameters=HnswParameters(metric="cosine") 
                        )
                    ]
                )        
            )
        if args.verbose: print(f"Creating {index_name} search index")
        index_client.create_index(index)
    else:
        if args.verbose: print(f"Search index {index_name} already exists")

def create_search_index():
    if args.verbose: print(f"Ensuring search index {args.indextext} and {args.indexcode} exists")
    index_client = SearchIndexClient(endpoint=f"https://{args.searchservice}.search.windows.net/",
                                     credential=search_creds)
    check_index_exist(index_client, args.indextext)
    check_index_exist(index_client, args.indexcode)    

def index_sections(filename, sections):
    if filename.endswith(".py"):
        index_name = args.indexcode
    else:
        index_name = args.indextext

    search_client = SearchClient(endpoint=f"https://{args.searchservice}.search.windows.net/",
                                    index_name=index_name,
                                    credential=search_creds)
    if args.verbose: print(f"Indexing sections from '{filename}' into search index '{index_name}'")
    
    i = 0
    batch = []
    for s in sections:
        batch.append(s)
        i += 1
        if i % 1000 == 0:
            results = search_client.upload_documents(documents=batch)
            succeeded = sum([1 for r in results if r.succeeded])
            if args.verbose: print(f"\tIndexed {len(results)} sections, {succeeded} succeeded")
            batch = []

    if len(batch) > 0:
        results = search_client.upload_documents(documents=batch)
        succeeded = sum([1 for r in results if r.succeeded])
        if args.verbose: print(f"\tIndexed {len(results)} sections, {succeeded} succeeded")

def remove_from_index(filename):
    if args.verbose: print(f"Removing sections from '{filename or '<all>'}' from search index '{args.indextext}'")
    search_client = SearchClient(endpoint=f"https://{args.searchservice}.search.windows.net/",
                                    index_name=args.indextext,
                                    credential=search_creds)
    while True:
        filter = None if filename == None else f"sourcefile eq '{os.path.basename(filename)}'"
        r = search_client.search("", filter=filter, top=1000, include_total_count=True)
        if r.get_count() == 0:
            break
        r = search_client.delete_documents(documents=[{ "id": d["id"] } for d in r])
        if args.verbose: print(f"\tRemoved {len(r)} sections from index")
        # It can take a few seconds for search results to reflect changes, so wait a bit
        time.sleep(2)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Prepare documents by extracting content from PDFs, splitting content into sections, uploading to blob storage, and indexing in a search index.",
        epilog="Example: prepdocs.py '..\data\*' --storageaccount myaccount --container mycontainer --searchservice mysearch --index myindex -v"
        )
    parser.add_argument("files", help="Files to be processed")
    parser.add_argument("--category", help="Value for the category field in the search index for all sections indexed in this run")
    parser.add_argument("--skipblobs", action="store_true", help="Skip uploading individual pages to Azure Blob Storage")
    parser.add_argument("--storageaccount", help="Azure Blob Storage account name")
    parser.add_argument("--container", help="Azure Blob Storage container name")
    parser.add_argument("--storagekey", required=False, help="Optional. Use this Azure Blob Storage account key instead of the current user identity to login (use az login to set current user for Azure)")
    parser.add_argument("--tenantid", required=False, help="Optional. Use this to define the Azure directory where to authenticate)")
    parser.add_argument("--searchservice", help="Name of the Azure Cognitive Search service where content should be indexed (must exist already)")
    parser.add_argument("--indextext", help="Name of the Azure Cognitive Search index where text content should be indexed (will be created if it doesn't exist)")
    parser.add_argument("--indexcode", help="Name of the Azure Cognitive Search index where code content should be indexed (will be created if it doesn't exist)")
    parser.add_argument("--searchkey", required=False, help="Optional. Use this Azure Cognitive Search account key instead of the current user identity to login (use az login to set current user for Azure)")
    parser.add_argument("--novectors", action="store_true", help="Don't compute embeddings for the sections (e.g. don't call the OpenAI embeddings API during indexing)")
    # Arguments for OpenAI
    parser.add_argument("--openaiapikey", required=False, help="Optional. OpenAI API key, not Azure OpenAI key")
    parser.add_argument("--openaiembedmodel", required=False, default='text-embedding-ada-002', help="Optional. OpenAI API Embedding model name")
    # Arguments for Azure OpenAI
    parser.add_argument("--openaiservice", help="Name of the Azure OpenAI service used to compute embeddings")
    parser.add_argument("--openaiembeddingdeployment", help="Name of the Azure OpenAI model deployment for an embedding model ('text-embedding-ada-002' recommended)")
    parser.add_argument("--openaigptdeployment", help="Name of the Azure OpenAI model deployment for a gpt model ('gpt-3.5-turbo' recommended)")
    parser.add_argument("--openaikey", required=False, help="Optional. Use this Azure OpenAI account key instead of the current user identity to login (use az login to set current user for Azure)")
    
    parser.add_argument("--remove", action="store_true", help="Remove references to this document from blob storage and the search index")
    parser.add_argument("--removeall", action="store_true", help="Remove all blobs from blob storage and documents from the search index")
    parser.add_argument("--localpdfparser", action="store_true", help="Use PyPdf local PDF parser (supports only digital PDFs) instead of Azure Form Recognizer service to extract text, tables and layout from the documents")
    parser.add_argument("--formrecognizerservice", required=False, help="Optional. Name of the Azure Form Recognizer service which will be used to extract text, tables and layout from the documents (must exist already)")
    parser.add_argument("--formrecognizerkey", required=False, help="Optional. Use this Azure Form Recognizer account key instead of the current user identity to login (use az login to set current user for Azure)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    args = parser.parse_args()

    # Use the current user identity to connect to Azure services unless a key is explicitly set for any of them
    azd_credential = AzureDeveloperCliCredential() if args.tenantid == None else AzureDeveloperCliCredential(tenant_id=args.tenantid, process_timeout=60)
    default_creds = azd_credential if args.searchkey == None or args.storagekey == None else None
    search_creds = default_creds if args.searchkey == None else AzureKeyCredential(args.searchkey)
    use_vectors = not args.novectors

    # Document splitter / Chunking strategy
    markdown_splitter = MarkdownSplitter(openaikey=args.openaikey,
                                openaiservice=args.openaiservice,
                                gptdeployment=args.openaigptdeployment)
    
    code_splitter = CodeSplitter()
    email_splitter = EmailSplitter()
    
    if not args.skipblobs:
        storage_creds = default_creds if args.storagekey == None else args.storagekey
    if not args.localpdfparser:
        # check if Azure Form Recognizer credentials are provided
        if args.formrecognizerservice == None:
            print("Error: Azure Form Recognizer service is not provided. Please provide formrecognizerservice or use --localpdfparser for local pypdf parser.")
            exit(1)
        formrecognizer_creds = default_creds if args.formrecognizerkey == None else AzureKeyCredential(args.formrecognizerkey)

    ### Comment out all things related to Azure OpenAI service
    if use_vectors:
        # If OpenAI API key provided, use OpenAI API directly
        if args.openaiapikey:
            openai.api_key = args.openaiapikey
        else:
            # Otherwise, use Azure OpenAI service
            if args.openaikey == None:
                openai.api_key = azd_credential.get_token("https://cognitiveservices.azure.com/.default").token
                openai.api_type = "azure_ad"
            else:
                openai.api_type = "azure"
                openai.api_key = args.openaikey

            openai.api_base = f"https://{args.openaiservice}.openai.azure.com"
            openai.api_version = "2023-05-15"

    if args.removeall:
        remove_blobs(None)
        remove_from_index(None)
    else:
        if not args.remove:
            create_search_index()
        
        print(f"Processing files...")
        for filename in glob.glob(args.files):
            if args.verbose: print(f"Processing '{filename}'")
            if args.remove:
                remove_blobs(filename)
                remove_from_index(filename)
            elif args.removeall:
                remove_blobs(None)
                remove_from_index(None)
            else:
                # if not args.skipblobs:
                #     upload_blobs(filename)
                # page_map = get_document_text(filename)
                if filename.endswith(".py"):
                    splitter = code_splitter
                elif filename.endswith(".md"):
                    splitter = markdown_splitter
                else:
                    splitter = email_splitter
                
                splits = splitter.load_and_split(filename)
                for s in splits:
                    print(s.content)
                    print("++++ END CONTENT ++++")
                    print(s.metadata)
                    print("===========END SPLIT=========")
                # sections = splitter.create_section(filename, splits)
                # index_sections(os.path.basename(filename), sections)
