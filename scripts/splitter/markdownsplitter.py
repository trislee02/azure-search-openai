import re
import openai

class MarkdownSplitter:
    def __init__(self, openaiapikey: str = None, openaikey: str = "", openaiservice: str = "", gptdeployment: str = ""):
        self.gptdeployment = gptdeployment
        if openaiapikey:
            openai.api_key = openaiapikey
        else:
            openai.api_type = "azure"
            openai.api_key = openaikey
            openai.api_base = f"https://{openaiservice}.openai.azure.com"
            openai.api_version = "2023-07-01-preview"

    MAX_SECTION_LENGTH = 1000
    SENTENCE_SEARCH_LIMIT = 100
    SECTION_OVERLAP = 100

    def __get_code_blocks(self, text: str) -> dict:
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

    def __clean_html_text(self, text: str) -> str:
        re_html = re.compile(r'<[^>]+>')
        cleaned_text = re_html.sub('', text)
        cleaned_text = cleaned_text.replace("&nbsp;", "")
        return cleaned_text

    def __split_markdown_heading(self, text: str) -> list:
        """
        Split a text by markdown heading symbol #
        Only # symbols at the beginning of line.
        """

        pattern = r"(?<!`)(?:^|\n)(?:#|##|###)(?![^`]*```[^`]*$)"
        segments = re.split(pattern, text, 0, re.MULTILINE)
        segments = [segment.strip() for segment in segments if segment.strip()]

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

        code_blocks = self.__get_code_blocks(all_text)
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

    def __generate_summary(self, text: str, previous_parts: str) -> str:
        system_message = """Given the summary of previous parts and full text of this part, write a summary of this part that shows how it relates to the previous parts"""
        content_template = """Summary of previous parts:
    ```
    {summary_previous_parts}
    ```
    This part:
    ```
    {text}
    ```
    Summary:"""
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

    def split_text(self, page_map):
        def find_page(offset):
            l = len(page_map)
            for i in range(l - 1):
                if offset >= page_map[i][1] and offset < page_map[i + 1][1]:
                    return i
            return l - 1

        summary = ""

        all_text = "".join(p[2] for p in page_map)

        all_text = self.__clean_html_text(all_text)

        segments = self.__split_markdown_heading(all_text)

        for seg in segments:    
            for part in self.__split_by_breaks(seg):
                if summary == "":
                    yield(part[0], find_page(part[1]))
                    summary = self.__generate_summary(part[0], "")
                else:
                    summary = self.__generate_summary(part[0], summary)
                    yield(f"{summary}\n{part[0]}", find_page(part[1]))
