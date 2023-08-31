import openai
from approaches.checker.checker import Checker

from scipy.spatial.distance import cosine

class EmbedTextChecker(Checker):
    THRESHOLD_ANSWER_CLOSE_TO_DOC = 0.15

    def __init__(self, embed_model = "", embedding_deployment = ""):
        self.embed_model = embed_model
        self.embedding_deployment = embedding_deployment

    def __compute_embedding(self, text): 
        if self.embedding_deployment != "":
            vector = openai.Embedding.create(engine=self.embedding_deployment, input=text)["data"][0]["embedding"]                
        else:
            vector = openai.Embedding.create(input=text, model=self.embed_model)["data"][0]["embedding"]
        return vector
    
    def check(self, answer: str, content_texts: list, content_vectors: list, callback: callable = None):
        answer_vector = self.__compute_embedding(answer)

        distances = [] # Debug
        for i in range(len(content_vectors)):
            doc_vector = content_vectors[i]
            distance = cosine(answer_vector, doc_vector)
            distances.append(f"{content_texts[i]}<br>=====>Distance: {str(distance)}")
            if distance < self.THRESHOLD_ANSWER_CLOSE_TO_DOC:
                if callback:
                    callback("\n================\n".join(distances))
                    return True
        if callback:
            callback("\n================\n".join(distances))
        return False