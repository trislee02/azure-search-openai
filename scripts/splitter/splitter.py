
from abc import ABC, abstractmethod
from document.Document import Document

class Splitter:

    @abstractmethod
    def load(self, filename: str) -> list[Document]:
        ...

    @abstractmethod
    def split(self, documents: list[Document]) -> list[Document]:
        ...

    @abstractmethod
    def create_section(self, filename: str, documents: list[Document]):
        ...

    def load_and_split(self, filename: str) -> list[Document]:
        docs = self.load(filename)
        splits = self.split(docs)
        return splits