import pickle

class ChatMessageClassifier:
    TYPE_OTHER = "other"
    TYPE_QUESTION = "question"

    INDEX_LABEL = {
        0: TYPE_OTHER,
        1: TYPE_QUESTION,
    }
    
    def __init__(self):
        self.model = pickle.load(open("engine/model/message-model.pkl", 'rb'))

    def run(self, message: str, return_index: bool = False) -> int:
        pred = self.model.predict([message])
        label = self.INDEX_LABEL[pred[0]]

        if return_index:
            return pred[0]
        else:
            return label

