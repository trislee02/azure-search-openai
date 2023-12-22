import copy
import utils
from indexer.indexer import Indexer
from document.Document import Document
from llama_index.node_parser import SentenceSplitter
from llama_hub.github_repo import GithubRepositoryReader, GithubClient
from customized_lib.github import GitHubIssuesLoader
from customized_lib.text_splitter import SummaryRecursiveCharacterTextSplitter

class GithubRepoIndexer(Indexer):

    def __init__(self, index_name: str = None, githubtoken: str = None, owner: str = None, repo: str = None, branch: str = None):
        super().__init__(index_name)
        self.githubtoken = githubtoken
        self.branch = branch
        self.owner = owner
        self.repo = repo

    def load(self) -> list[Document]:
        final_docs = []
        github_client = GithubClient(self.githubtoken)
        loader = GithubRepositoryReader(
            github_client,
            owner =                  self.owner,
            repo =                   self.repo,
            filter_file_extensions = ([".py", ".cpp", ".md"], GithubRepositoryReader.FilterType.INCLUDE),
            concurrent_requests =    10,
        )
        docs = loader.load_data(branch=self.branch)
        for doc in docs:
            content = doc.get_content()
            metadata = doc.extra_info
            new_doc = Document(content=content, metadata=metadata)
            final_docs.append(new_doc)
        return final_docs
            
    def split(self, documents: list[Document]) -> list[Document]:
        final_docs = []
        parser = SentenceSplitter()
        for doc in documents:
            splits = parser.split_text(doc.content)
            for split in splits:
                metadata = copy.deepcopy(doc.metadata)
                new_doc = Document(content=split, metadata=metadata)
                final_docs.append(new_doc)
        return final_docs
    
    def create_section(self, documents: list[Document]):
        for i, doc in enumerate(documents[:2]):
            if doc.content != "":
                doc_id = utils.url_to_hash(doc.metadata["url"])
                section = {
                    "id": f"{doc_id}",
                    "content": doc.content,
                    "sourcefile": doc.metadata["file_path"],
                    "embedding": utils.compute_embedding(doc.content),
                    "url": doc.metadata["url"]
                }
                yield section  

class GithubIssuesIndexer(Indexer):

    def __init__(self, index_name: str = None, githubtoken: str = None, repo: str = None, include_prs: bool = False, chunk_size: int = 2000, chunk_overlap: int = 200):
        super().__init__(index_name)

        self.loader = GitHubIssuesLoader(
            repo=repo,
            access_token=githubtoken,  # delete/comment out this argument if you've set the access token as an env var.
            include_prs=include_prs,
        )

        self.text_splitter = SummaryRecursiveCharacterTextSplitter(
            chunk_size = chunk_size,
            chunk_overlap  = chunk_overlap,
            length_function = len,
            is_separator_regex = False,
        )

    def load(self) -> list[Document]:
        final_docs = []
        docs = self.loader.load()
        for doc in docs:
            new_doc = Document(content=doc.page_content, metadata=doc.metadata)
            final_docs.append(new_doc)
        return final_docs
            
    def split(self, documents: list[Document]) -> list[Document]:
        final_docs = []
        for doc in documents:
            splits = self.text_splitter.split_text(doc.content)
            for split in splits:
                metadata = copy.deepcopy(doc.metadata)
                new_doc = Document(content=split, metadata=metadata)
                final_docs.append(new_doc)
        return final_docs
    
    def create_section(self, documents: list[Document]):
        for i, doc in enumerate(documents):
            if doc.content != "":
                doc_id = utils.url_to_hash(doc.metadata["url"])
                section = {
                    "id": f"{doc_id}",
                    "content": doc.content,
                    "sourcefile": doc.metadata["url"],
                    "embedding": utils.compute_embedding(doc.content),
                    "url": doc.metadata["url"]
                }
                yield section  
