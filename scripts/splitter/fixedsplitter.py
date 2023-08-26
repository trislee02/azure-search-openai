from splitter.splitter import Splitter

class FixedSplitter(Splitter):
    MAX_SECTION_LENGTH = 1000
    SENTENCE_SEARCH_LIMIT = 100
    SECTION_OVERLAP = 100

    def split_text(self, page_map):
        SENTENCE_ENDINGS = [".", "!", "?"]
        WORDS_BREAKS = [",", ";", ":", " ", "(", ")", "[", "]", "{", "}", "\t", "\n"]

        def find_page(offset):
            l = len(page_map)
            for i in range(l - 1):
                if offset >= page_map[i][1] and offset < page_map[i + 1][1]:
                    return i
            return l - 1

        all_text = "".join(p[2] for p in page_map)
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

            section_text = all_text[start:end]
            yield (section_text, find_page(start))

            last_table_start = section_text.rfind("<table")
            if (last_table_start > 2 * self.SENTENCE_SEARCH_LIMIT and last_table_start > section_text.rfind("</table")):
                # If the section ends with an unclosed table, we need to start the next section with the table.
                # If table starts inside SENTENCE_SEARCH_LIMIT, we ignore it, as that will cause an infinite loop for tables longer than MAX_SECTION_LENGTH
                # If last table starts inside SECTION_OVERLAP, keep overlapping
                start = min(end - self.SECTION_OVERLAP, start + last_table_start)
            else:
                start = end - self.SECTION_OVERLAP
            
        if start + self.SECTION_OVERLAP < end:
            yield (all_text[start:end], find_page(start))