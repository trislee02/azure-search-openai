import re
import openai
import utils
from tenacity import retry, stop_after_attempt, wait_random_exponential, wait_fixed
from splitter.splitter import Splitter
from document.Document import Document

def clean_html_text(text):
    re_html = re.compile(r'<[^>]+>')
    cleaned_text = re_html.sub('', text)
    cleaned_text = cleaned_text.replace("&nbsp;", "")
    return cleaned_text

def clean_special_character(text: str) -> str:
    cleaned_text = text.replace("\\n", "\n").replace("\\t", "\t")
    return cleaned_text

def get_code_blocks(text: str) -> dict:
    """
    Find all code blocks inside triple backsticks (```) pairs.
    Return a dictionary with starting index as key and code block as value.
    """
    pattern = r"```([^`]+)```"
    matches = re.finditer(pattern, text)

    matches_with_indices = {}

    for match in matches:
        match_text = match.group()
        start_index = match.start()
        matches_with_indices[start_index] = match_text

    return matches_with_indices

class MarkdownSplitter(Splitter):
    MAX_SECTION_LENGTH = 1000
    SENTENCE_SEARCH_LIMIT = 100
    SECTION_OVERLAP = 100
        
    def __init__(self, openaiapikey: str = None, openaikey: str = "", openaiservice: str = "", gptdeployment: str = ""):
        self.gptdeployment = gptdeployment
        if openaiapikey:
            openai.api_key = openaiapikey
        else:
            openai.api_type = "azure"
            openai.api_key = openaikey
            openai.api_base = f"https://{openaiservice}.openai.azure.com"
            openai.api_version = "2023-07-01-preview"

    def load(self, filename: str) -> list[Document]:
        with open(filename, "r", encoding="utf-8") as f:
            content = f.read()
            content = clean_html_text(content)
            document = Document(content=content)
            return [document]

    def split(self, documents: list[Document]) -> list[Document]:
        splits = []
        
        summary = ""
        all_text = "".join(doc.content for doc in documents)
        all_text = clean_html_text(all_text)
        all_text = clean_special_character(all_text)

        segments = self.__split_markdown_heading(all_text)

        for seg in segments:    
            for part in self.__split_by_breaks(seg):
                if summary == "":
                    document = Document(content=part[0])
                    summary = self.__generate_summary(part[0], "")
                else:
                    summary = self.__generate_summary(part[0], summary)
                    document = Document(content=f"{summary}\n{part[0]}")
                splits.append(document)
                    
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

    def __split_markdown_heading(self, text: str) -> list:
        """
        Split a text by markdown heading symbol #
        Only # symbols at the beginning of line.
        """

        pattern = r"(?<!`)(?:^|\n)(?:#|##|###)(?![^`]*```[^`]*$)"
        segments = re.split(pattern, text, 0, re.MULTILINE)
        segments = [segment.strip() for segment in segments if segment.strip()]
#         segments_str = "\n=============\n".join(segments)
#         print(f"""Segments:
# {segments_str}

# *****************************
# """)

        return segments

    def __find_smaller_closest(self, sorted_array: list, k:  float) -> int:
        l = 0
        r = len(sorted_array) - 1
        mid = 0
        while l < r:
            mid = (l + r) // 2
            if sorted_array[mid] < k:
                l = mid + 1
            else:
                r = mid - 1
        return l

    def __split_by_breaks(self, all_text: str) -> list:
        SENTENCE_ENDINGS = [".", "!", "?"]
        WORDS_BREAKS = [",", ";", ":", " ", "(", ")", "[", "]", "{", "}", "\t", "\n"]

        code_blocks = get_code_blocks(all_text)
        code_block_indexes = sorted(code_blocks.keys())

        length = len(all_text)
        start = 0
        end = length
        while start + self.SECTION_OVERLAP < length:
            last_word = -1
            end = start + self.MAX_SECTION_LENGTH

            if end > length:
                end = length
            else:
                # Try to find the end of the sentence
                while end < length and (end - start - self.MAX_SECTION_LENGTH) < self.SENTENCE_SEARCH_LIMIT and all_text[end] not in SENTENCE_ENDINGS:
                    if all_text[end] in WORDS_BREAKS:
                        last_word = end
                    end += 1
                if end < length and all_text[end] not in SENTENCE_ENDINGS and last_word > 0:
                    end = last_word # Fall back to at least keeping a whole word
            if end < length:
                end += 1

            # Try to find the start of the sentence or at least a whole word boundary
            last_word = -1
            while start > 0 and start > end - self.MAX_SECTION_LENGTH - 2 * self.SENTENCE_SEARCH_LIMIT and all_text[start] not in SENTENCE_ENDINGS:
                if all_text[start] in WORDS_BREAKS:
                    last_word = start
                start -= 1
            if all_text[start] not in SENTENCE_ENDINGS and last_word > 0:
                start = last_word
            if start > 0:
                start += 1

            # Check if the document break down any snippet codes
            if len(code_block_indexes) > 0:
                closest_code_block = code_block_indexes[self.__find_smaller_closest(code_block_indexes, end)]
                code_block_len = len(code_blocks[closest_code_block])
                if end < closest_code_block + code_block_len:
                    end = closest_code_block + code_block_len

            section_text = all_text[start:end]
            yield (section_text, start)

            last_table_start = section_text.rfind("<table")
            if (last_table_start > 2 * self.SENTENCE_SEARCH_LIMIT and last_table_start > section_text.rfind("</table")):
                # If the section ends with an unclosed table, we need to start the next section with the table.
                # If table starts inside SENTENCE_SEARCH_LIMIT, we ignore it, as that will cause an infinite loop for tables longer than MAX_SECTION_LENGTH
                # If last table starts inside SECTION_OVERLAP, keep overlapping
                start = min(end - self.SECTION_OVERLAP, start + last_table_start)
            else:
                start = end - self.SECTION_OVERLAP
            
        if start + self.SECTION_OVERLAP < end:
            yield (all_text[start:end], start)


    def before_retry_sleep(retry_state):
        print(f"Rate limited on the OpenAI API, sleeping before retrying...")

    @retry(wait=wait_random_exponential(min=15, max=60), stop=stop_after_attempt(15), before_sleep=before_retry_sleep)
    def __generate_summary(self, text: str, previous_parts: str) -> str:
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
                    {"role": "user",    "content": content_template.format(text=text,
                                                                           summary_previous_parts=previous_parts)}]

        response = openai.ChatCompletion.create(engine=self.gptdeployment,
                                                messages = messages,
                                                temperature=0.7,
                                                max_tokens=800,
                                                top_p=0.95,
                                                frequency_penalty=0,
                                                presence_penalty=0,
                                                stop=None)

        return response.choices[0].message.content