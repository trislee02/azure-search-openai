from typing import Any

from engine.toxicclassifier import ToxicMessageClassifier

class Approach:
    def __init__(self):
        self.toxicity_checker = ToxicMessageClassifier()

    def run(self, q: str, overrides: dict[str, Any]) -> Any:
        raise NotImplementedError

    def selfcheck_toxicity(self, message: str):
        return self.toxicity_checker.run(message)[0]
        