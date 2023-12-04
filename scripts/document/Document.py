from dataclasses import dataclass, field
from collections import defaultdict


@dataclass
class Document:
    """String text"""
    content: str = None

    """Associated metadata"""
    metadata: dict = field(default_factory=defaultdict)