from approaches.checker.checker import Checker
from javascript import require, globalThis
import re

class CodeAttributionChecker(Checker):

    THRESHOLD_CODE_SIMILARITY = 0.8

    LOG_TEMPLATE = """==============COMPARE CODE===============
Answer code: 
<pre>
    <code>{answer_code}</code>
</pre>
----------------
Source code: 
<pre>
    <code>{source_code}</code>
</pre>
----------------
@@@@@@> Similarity: {local_similarity}
"""                 

    def __remove_redundant_empty_lines(self, input_string):
        # Define the regular expression pattern to match multiple consecutive empty lines
        pattern = r'\n\s*\n+'
        
        # Replace the matched pattern with a single newline character
        clean_string = re.sub(pattern, '\n', input_string.strip())
        
        return clean_string

    def __calculate_percentage(self, similar_parts: list, text: str) -> float:
        lines = text.splitlines()
        total_lines = len(lines)
        
        events = []
        for part in similar_parts:
            events.append((part[0], True)) # True means starting point
            events.append((part[1], False))
        
        events.sort(key=lambda x: (x[0], not x[1]))

        count_similar = 0
        first_start = -1
        start_pos = 0
        for pos, is_start in events:
            if is_start:
                if start_pos == 0:
                    first_start = pos
                start_pos += 1
            else:
                start_pos -= 1
                if start_pos == 0:
                    count_similar += pos - first_start + 1
        return count_similar / total_lines

    def __clean_text(self, text: str) -> str:
        cleaned_text = self.__remove_redundant_empty_lines(text)
        cleaned_text = cleaned_text.replace("\\n", "\n").replace("\\'", "'").replace("```","")
        return cleaned_text

    def __compare_code(self, code_1: str, code_2: str):
        dolos = require("./jslib/codeplagiarism.js")

        compares = dolos.compareCode(code_1, code_2)

        code_1_similar_parts = [[p[0], p[1]] for p in compares]
        code_2_similar_parts = [[p[2], p[3]] for p in compares]
        
        return code_1_similar_parts, code_2_similar_parts

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

    def check(self, answer: str, supporting_content: list, callback: callable = None):
        answer_code_blocks = self.__get_code_blocks(answer)

        log = ""
        for answer_code in answer_code_blocks.values():
            answer_code = self.__clean_text(answer_code)
            attributed_parts = []
            for content in supporting_content:
                source_code_blocks = self.__get_code_blocks(content)
                for source_code in source_code_blocks.values():
                    source_code = self.__clean_text(source_code)
                    similar_parts, _ = self.__compare_code(answer_code, source_code)
                    attributed_parts.extend(similar_parts)
                    print("Similar parts:")
                    print(similar_parts)
                    local_similarity = self.__calculate_percentage(similar_parts, answer_code)
                    log += self.LOG_TEMPLATE.format(answer_code=answer_code,
                                                    source_code=source_code,
                                                    local_similarity=local_similarity)
            print("Attributed pards:")
            print(attributed_parts)
            attribution_score = self.__calculate_percentage(attributed_parts, answer_code)
            log += f"\n*******************************\n\
                Attribution score: {attribution_score}\n\
                    *******************************\n"

            # An answer is invalid if at least one of snippet code included in answer is invalid
            if attribution_score < self.THRESHOLD_CODE_SIMILARITY:
                if callback:
                    callback(log)
                return False
        if callback:
            callback(log)
        return True

if __name__ == '__main__':
    checker = CodeAttributionChecker()
    checker.check("```Hello, World```", ["```Hello, World```", "```Hello, World```"])