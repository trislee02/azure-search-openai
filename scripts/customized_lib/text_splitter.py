import re

from langchain.text_splitter import TextSplitter, RecursiveCharacterTextSplitter, _split_text_with_regex
from typing import (
    Optional,
    List,
    Any
)
from utils import compute_chatcompletion

def _generate_summary(preceding_split: str, split: str) -> str:
    system_message = """Given the summary of the preceding parts and the complete text of the current part, write a concise introduction that links the current part to the preceding parts."""
    content_template = """Summary of the preceding parts:
```
{summary_previous_parts}
```
The current part:
```
{text}
```
Introduction for the current part:"""
    messages = [{"role": "system",  "content": system_message},
                {"role": "user",    "content": content_template.format(text=split,
                                                                        summary_previous_parts=preceding_split)}]

    return compute_chatcompletion(messages=messages)

class SummaryRecursiveCharacterTextSplitter(RecursiveCharacterTextSplitter):
    """Splitting text by recursively look at characters. Add a summary at the beginning of each split to link with preceding splits.

    Recursively tries to split by different characters to find one
    that works.
    """

    def split_text(self, text: str) -> List[str]:
        final_chunks = []
        chunks = super().split_text(text)
        preceding_split = ""
        for chunk in chunks:
            if preceding_split != "":
                summary = _generate_summary(preceding_split=preceding_split, split=chunk)
                chunk = f"{summary}\n{chunk}"

            final_chunks.append(chunk)
            preceding_split = chunk
        return final_chunks
