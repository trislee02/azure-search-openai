import pickle

class ToxicMessageClassifier:
    def __init__(self):
        self.model = pickle.load(open("engine/model/toxic-model.pkl", 'rb'))

    def run(self, message: str) -> bool:
        pred = self.model.predict([message])
        return pred