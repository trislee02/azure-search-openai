import argparse
import os
import openai
import pickle
import utils
from typing import Any, List, Sequence

from dotenv import load_dotenv
from llama_index import download_loader, VectorStoreIndex, ServiceContext, set_global_service_context
from llama_index.llms import AzureOpenAI
from llama_index.embeddings import AzureOpenAIEmbedding
from llama_index.node_parser import SentenceSplitter
from llama_hub.github_repo import GithubRepositoryReader, GithubClient
from llama_index.schema import (
    BaseNode
)

from azure.core.credentials import AzureKeyCredential
from azure.identity import AzureDeveloperCliCredential
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

def create_search_index(indexes: dict[str, str]):
    index_client = SearchIndexClient(endpoint=f"https://{args.searchservice}.search.windows.net/",
                                     credential=search_creds)
    for index_name in indexes.values():
        check_index_exist(index_client, index_name)

def check_index_exist(index_client, index_name):
    if args.verbose: print(f"Ensuring search index {index_name} exists")
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
                SimpleField(name="url", type="Edm.String", filterable=False, facetable=True)
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

def create_section(nodes: List[BaseNode]):
    for i, doc in enumerate(nodes):
        if doc.get_content().strip() != "":
            section = {
                "id": f"{doc.hash}",
                "content": doc.get_content(),
                "sourcefile": doc.extra_info["file_path"],
                "embedding": utils.compute_embedding(doc.get_content()),
                "url": doc.extra_info["url"]
            }
            yield section  

def get_index_name():
    return indexes['repo']

def index_sections(sections):
    index_name = get_index_name()
    search_client = SearchClient(endpoint=f"https://{args.searchservice}.search.windows.net/",
                                    index_name=index_name,
                                    credential=search_creds)
    if args.verbose: print(f"Indexing sections...")
    
    i = 0
    batch = []
    for s in sections:
        batch.append(s)
        i += 1
        if i % 10 == 0:
            results = search_client.upload_documents(documents=batch)
            succeeded = sum([1 for r in results if r.succeeded])
            if args.verbose: print(f"\tIndexed {len(results)} sections, {succeeded} succeeded")
            batch = []

    if len(batch) > 0:
        results = search_client.upload_documents(documents=batch)
        succeeded = sum([1 for r in results if r.succeeded])
        if args.verbose: print(f"\tIndexed {len(results)} sections, {succeeded} succeeded")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--githubtoken", required=True, help="Github token")
    parser.add_argument("--owner", required=True, help="Github repo owner")
    parser.add_argument("--repo", required=True, help="Repository name")
    parser.add_argument("--branch", default="main", help="Branch name")
    parser.add_argument("--azuregptdeployment", help="Azure OpenAI GPT deployment name")
    parser.add_argument("--azureopenaikey", help="Azure OpenAI API key")
    parser.add_argument("--azureopenaibase", help="Azure OpenAI API base")
    parser.add_argument("--azureopenaiversion", help="Azure OpenAI API version")
    parser.add_argument("--azureembeddingdeployment", help="Azure OpenAI Embedding deployment name")
    parser.add_argument("--docs", help="Path to pickle file of saved documents")
    parser.add_argument("--nodes", help="Path to pickle file of saved nodes")
    parser.add_argument("--tenantid", required=False, help="Optional. Use this to define the Azure directory where to authenticate)")
    parser.add_argument("--searchkey", required=False, help="Optional. Use this Azure Cognitive Search account key instead of the current user identity to login (use az login to set current user for Azure)")
    parser.add_argument("--searchservice", help="Name of the Azure Cognitive Search service where content should be indexed (must exist already)")
    parser.add_argument("--indexrepo", help="Name of the Azure Cognitive Search index where ROS document should be indexed (will be created if it doesn't exist)")
    parser.add_argument("-v", "--verbose", default=False, action="store_true", help="Verbose")
    args = parser.parse_args()

    # Use the current user identity to connect to Azure services unless a key is explicitly set for any of them
    azd_credential = AzureDeveloperCliCredential() if args.tenantid == None else AzureDeveloperCliCredential(tenant_id=args.tenantid, process_timeout=60)
    default_creds = azd_credential if args.searchkey == None else None
    search_creds = default_creds if args.searchkey == None else AzureKeyCredential(args.searchkey)

    # Set up Azure OpenAI API
    llm = AzureOpenAI(
        model="gpt-35-turbo",
        deployment_name=args.azuregptdeployment,
        api_key=args.azureopenaikey,
        azure_endpoint=args.azureopenaibase,
        api_version=args.azureopenaiversion,
    )

    embed_model = AzureOpenAIEmbedding(
        model="text-embedding-ada-002",
        deployment_name=args.azureembeddingdeployment,
        api_key=args.azureopenaikey,
        azure_endpoint=args.azureopenaibase,
        api_version=args.azureopenaiversion,
    )

    service_context = ServiceContext.from_defaults(
        llm=llm,
        embed_model=embed_model,
    )

    set_global_service_context(service_context)

    # Setup Search Index

    indexes = {
        "repo": args.indexrepo,
    }

    create_search_index(indexes)

    docs = None
    nodes = None
    if args.docs and os.path.exists(args.docs):
        with open(args.docs, "rb") as f:
            docs = pickle.load(f)

    if docs is None:
        github_client = GithubClient(args.githubtoken)
        loader = GithubRepositoryReader(
            github_client,
            owner =                  args.owner,
            repo =                   args.repo,
            # filter_directories =     (["demos", "examples"], GithubRepositoryReader.FilterType.INCLUDE),
            filter_file_extensions = ([".py", ".cpp", ".md"], GithubRepositoryReader.FilterType.INCLUDE),
            verbose =                args.verbose,
            concurrent_requests =    10,
        )

        docs = loader.load_data(branch=args.branch)
        
        if args.docs:
            with open(args.docs, "wb") as f:
                pickle.dump(docs, f)

    
    if args.nodes and os.path.exists(args.nodes):
        with open(args.nodes, "rb") as f:
            nodes = pickle.load(f)

    if nodes is None:
        parser = SentenceSplitter()
        nodes = parser.get_nodes_from_documents(docs)
        if args.nodes:
            with open(args.nodes, "wb") as f:
                pickle.dump(nodes, f)

    sections = create_section(nodes)
    index_sections(sections)

    
    