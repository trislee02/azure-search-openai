from abc import ABC, abstractmethod
from typing import Any, Sequence

class SubRAG(ABC):

    @abstractmethod
    def run(self, history: Sequence[dict[str, str]], overrides: dict[str, Any], query_text: str = None) -> Any:
        """Run RAG pipeline"""