from splitter.splitter import Splitter
from abc import ABC, abstractmethod
from document.Document import Document

class Indexer:
    """Online indexer"""
    index_name: str = None

    def __init__(self, index_name: str = None):
        self.index_name = index_name

    @abstractmethod
    def load(self) -> list[Document]:
        ...

    @abstractmethod
    def split(self, documents: list[Document]) -> list[Document]:
        ...

    @abstractmethod
    def create_section(self, documents: list[Document]):
        ...

    def load_and_split(self) -> list[Document]:
        docs = self.load()
        splits = self.split(docs)
        return splits