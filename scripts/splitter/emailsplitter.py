from splitter.splitter import Splitter
from document.Document import Document
import warnings
import utils
import re

warnings.filterwarnings("ignore")
from pprint import pprint

class EmailSplitter(Splitter):
    def __init__(self):
        pass

    def load(self, filename: str) -> list[Document]:
        with open(filename, "r", encoding="utf-8") as f:
            content = f.read()
            return [Document(content=content)]

    def split(self, documents: list[Document]) -> list[Document]:
        CUSTOMER = "Customer"
        LUXAI_SUPPORT = "LuxAI Support"

        segments = []
        
        all_text = "".join(doc.content for doc in documents)

        pattern = r"(?<!`)(?:^|\n)(?:#|##|###)(?![^`]*```[^`]*$)(.*)"
        splits = re.split(pattern, all_text, 0, re.MULTILINE)

        turn = ""
        customer_message = ""
        for index, segment in enumerate(splits):
            segment = segment.strip()
            if len(segment) == 0:
                continue
            if segment.startswith(CUSTOMER):
                turn = CUSTOMER
            elif segment.startswith(LUXAI_SUPPORT):
                turn = LUXAI_SUPPORT
            else:
                if turn == CUSTOMER:
                    customer_message = segment
                else:
                    seg = Document(content=segment,
                                   metadata={
                                    "customer_message": customer_message   
                                   })
                    segments.append(seg)      
        return segments


    def create_section(self, filename: str, documents: list[Document]):
        file_id = utils.filename_to_id(filename)

        for i, doc in enumerate(documents):
            section = {
                "id": f"{file_id}-page-{i}",
                "content": doc.content,
                "sourcefile": filename,
                "customer_message": doc.metadata["customer_message"],
                "embedding": utils.compute_embedding(self.azure_openai_embed_deployment, doc.content)
            }
            yield section  
    

