class Classifier:
    def __init__(self) -> None:
        self.method = None

    def setMethod(self, _method):
        self.method = _method

    def classify(self, data):
        
        return [1,1,1], [0.5,0.5,0.5]