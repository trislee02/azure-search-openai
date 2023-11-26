from splitter.splitter import Splitter
import warnings

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

    def split_text(self, page_map):
        all_text = "".join(p[2] for p in page_map)

        blob = Blob.from_data(all_text)

        parser = LanguageParser(language=Language.PYTHON)
        docs = list(parser.lazy_parse(blob))

        # for document in docs:
        #     yield(document.page_content, 0)

        python_splitter = RecursiveCharacterTextSplitter.from_language(
            language=Language.PYTHON, chunk_size=2000, chunk_overlap=200
        )
        result = python_splitter.split_documents(docs)

        for document in result:
            yield(document.page_content, 0)

    

