from approaches.checker.checker import Checker
from javascript import require, globalThis
import re

class CodeChecker(Checker):

    THRESHOLD_CODE_SIMILARITY = 0.8

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

    def __measure_similarity(self, code_1: str, code_2: str):
        dolos = require("./jslib/codeplagiarism.js")
        
        code_1 = self.__remove_redundant_empty_lines(code_1)
        code_2 = self.__remove_redundant_empty_lines(code_2)

        compares = dolos.compareCode(code_1, code_2)

        code_1_similar_parts = [[p[0], p[1]] for p in compares]
        code_2_similar_parts = [[p[2], p[3]] for p in compares]
        
        percent_1 = self.__calculate_percentage(code_1_similar_parts, code_1)
        percent_2 = self.__calculate_percentage(code_2_similar_parts, code_2)
        
        return percent_1, percent_2

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
            for content in supporting_content:
                source_code_blocks = self.__get_code_blocks(content)
                for source_code in source_code_blocks.values():
                    answer_code = answer_code.replace("```","")
                    source_code = source_code.replace("\\n", "\n").replace("\\'", "'").replace("```","")
                    similarity, _ = self.__measure_similarity(answer_code, source_code)
                    log += f"""==============COMPARE CODE===============
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
@@@@@@> Similarity: {similarity}
"""                 
                    
                    if similarity >= CodeChecker.THRESHOLD_CODE_SIMILARITY:
                        if callback:
                            callback(log)
                        return True
            if callback:
                callback(log)
            return False

if __name__ == '__main__':
    checker = CodeChecker()
    checker.check("```Hello, World```", ["```Hello, World```", "```Hello, World```"])