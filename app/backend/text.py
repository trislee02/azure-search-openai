def nonewlines(s: str) -> str:
    if s:
        return s.replace('\r', ' ')#.replace('\n', ' ')
    return ""
