from splitter.splitter import Splitter
from document.Document import Document
import warnings
import utils

warnings.filterwarnings("ignore")
from pprint import pprint
from langchain.document_loaders.blob_loaders import Blob
from langchain.document_loaders.generic import GenericLoader

from langchain.document_loaders.parsers import LanguageParser
from langchain.text_splitter import (
    Language,
    RecursiveCharacterTextSplitter,
)

class CodeSplitter(Splitter):
    def __init__(self):
        pass

    def load(self, filename: str) -> list[Document]:
        with open(filename, "r", encoding="utf-8") as f:
            content = f.read()
            return [Document(content=content)]

    def split(self, documents: list[Document]) -> list[Document]:
        splits = []
        
        all_text = "".join(doc.content for doc in documents)

        blob = Blob.from_data(all_text)

        parser = LanguageParser(language=Language.PYTHON)
        docs = list(parser.lazy_parse(blob))

        python_splitter = RecursiveCharacterTextSplitter.from_language(
            language=Language.PYTHON, chunk_size=2000, chunk_overlap=200
        )
        result = python_splitter.split_documents(docs)

        for document in result:
            splits.append(Document(content=document.page_content))
                    
        return splits


    def create_section(self, filename: str, documents: list[Document]):
        file_id = utils.filename_to_id(filename)

        for i, doc in enumerate(documents):
            section = {
                "id": f"{file_id}-page-{i}",
                "content": doc.content,
                "sourcefile": filename,
                "embedding": utils.compute_embedding(self.azure_openai_embed_deployment, doc.content)
            }
            yield section  
    

