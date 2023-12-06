import re
import openai
import html

from azure.ai.formrecognizer import DocumentAnalysisClient
from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed
from splitter.splitter import Splitter
from document.Document import Document
import utils

def table_to_html(table):
    table_html = "<table>"
    rows = [sorted([cell for cell in table.cells if cell.row_index == i], key=lambda cell: cell.column_index) for i in range(table.row_count)]
    for row_cells in rows:
        table_html += "<tr>"
        for cell in row_cells:
            tag = "th" if (cell.kind == "columnHeader" or cell.kind == "rowHeader") else "td"
            cell_spans = ""
            if cell.column_span > 1: cell_spans += f" colSpan={cell.column_span}"
            if cell.row_span > 1: cell_spans += f" rowSpan={cell.row_span}"
            table_html += f"<{tag}{cell_spans}>{html.escape(cell.content)}</{tag}>"
        table_html +="</tr>"
    table_html += "</table>"
    return table_html

class PdfSplitter(Splitter):
    def __init__(self, azure_openai_embed_deployment: str, form_recognizer_service: str, form_recognizer_credential):
        self.form_recognizer_service = form_recognizer_service
        self.form_recognizer_credential = form_recognizer_credential
        self.azure_openai_embed_deployment = azure_openai_embed_deployment

    def load(self, filename: str) -> list[Document]:
        documents = []
        page_map = []
        form_recognizer_client = DocumentAnalysisClient(endpoint=f"https://{self.form_recognizer_service}.cognitiveservices.azure.com/", 
                                                        credential=self.form_recognizer_credential, 
                                                        headers={"x-ms-useragent": "azure-search-chat-demo/1.0.0"})
        with open(filename, "rb") as f:
            poller = form_recognizer_client.begin_analyze_document("prebuilt-layout", document = f)
        form_recognizer_results = poller.result()

        for page_num, page in enumerate(form_recognizer_results.pages):
            tables_on_page = [table for table in form_recognizer_results.tables if table.bounding_regions[0].page_number == page_num + 1]

            # mark all positions of the table spans in the page
            page_offset = page.spans[0].offset
            page_length = page.spans[0].length
            table_chars = [-1]*page_length
            for table_id, table in enumerate(tables_on_page):
                for span in table.spans:
                    # replace all table spans with "table_id" in table_chars array
                    for i in range(span.length):
                        idx = span.offset - page_offset + i
                        if idx >=0 and idx < page_length:
                            table_chars[idx] = table_id

            # build page text by replacing charcters in table spans with table html
            page_text = ""
            added_tables = set()
            for idx, table_id in enumerate(table_chars):
                if table_id == -1:
                    page_text += form_recognizer_results.content[page_offset + idx]
                elif not table_id in added_tables:
                    page_text += table_to_html(tables_on_page[table_id])
                    added_tables.add(table_id)

            page_text += " "
            page_map.append((page_num, offset, page_text))
            document = Document(content=page_text, 
                                metadata= {
                                    "page_num": page_num,
                                    "offset": offset, })
            documents.append(document)

            offset += len(page_text)

        return documents

    def split(self, documents: list[Document]) -> list[Document]:
        splits = []

        MAX_SECTION_LENGTH = 1000
        SENTENCE_SEARCH_LIMIT = 100
        SECTION_OVERLAP = 100

        SENTENCE_ENDINGS = [".", "!", "?"]
        WORDS_BREAKS = [",", ";", ":", " ", "(", ")", "[", "]", "{", "}", "\t", "\n"]

        def find_page(offset):
            l = len(documents)
            for i in range(l - 1):
                if offset >= documents[i].metadata["offset"] and offset < documents[i + 1].metadata["offset"]:
                    return i
            return l - 1

        all_text = "".join(p.content for p in documents)
        length = len(all_text)
        start = 0
        end = length
        while start + SECTION_OVERLAP < length:
            last_word = -1
            end = start + MAX_SECTION_LENGTH

            if end > length:
                end = length
            else:
                # Try to find the end of the sentence
                while end < length and (end - start - MAX_SECTION_LENGTH) < SENTENCE_SEARCH_LIMIT and all_text[end] not in SENTENCE_ENDINGS:
                    if all_text[end] in WORDS_BREAKS:
                        last_word = end
                    end += 1
                if end < length and all_text[end] not in SENTENCE_ENDINGS and last_word > 0:
                    end = last_word # Fall back to at least keeping a whole word
            if end < length:
                end += 1

            # Try to find the start of the sentence or at least a whole word boundary
            last_word = -1
            while start > 0 and start > end - MAX_SECTION_LENGTH - 2 * SENTENCE_SEARCH_LIMIT and all_text[start] not in SENTENCE_ENDINGS:
                if all_text[start] in WORDS_BREAKS:
                    last_word = start
                start -= 1
            if all_text[start] not in SENTENCE_ENDINGS and last_word > 0:
                start = last_word
            if start > 0:
                start += 1

            section_text = all_text[start:end]
            document = Document(content=section_text,
                                metadata={
                                    "page_number": find_page(start)
                                })
            splits.append(document)

            last_table_start = section_text.rfind("<table")
            if (last_table_start > 2 * self.SENTENCE_SEARCH_LIMIT and last_table_start > section_text.rfind("</table")):
                # If the section ends with an unclosed table, we need to start the next section with the table.
                # If table starts inside SENTENCE_SEARCH_LIMIT, we ignore it, as that will cause an infinite loop for tables longer than MAX_SECTION_LENGTH
                # If last table starts inside SECTION_OVERLAP, keep overlapping
                start = min(end - self.SECTION_OVERLAP, start + last_table_start)
            else:
                start = end - self.SECTION_OVERLAP
            
        if start + self.SECTION_OVERLAP < end:
            document = Document(content=all_text[start:end],
                                metadata={
                                    "page_number": find_page(start)
                                })
            splits.append(document)
        return splits

    def create_section(self, filename: str, documents: list[Document]):
        file_id = utils.filename_to_id(filename)

        for i, doc in enumerate(documents):
            section = {
                "id": f"{file_id}-page-{i}",
                "content": doc.content,
                # "sourcepage": blob_name_from_file_page(filename, pagenum),
                "sourcefile": filename,
                "embedding": utils.compute_embedding(doc.content)
            }
            yield section