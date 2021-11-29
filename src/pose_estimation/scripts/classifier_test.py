import os
import cv2
import numpy as np
import csv
import pickle
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_val_score
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis

class Classifier:
    def __init__(self):
        #self.path = "/home/user/workspace/src/pose_estimation/data/V0/data_v0.csv"
        self.path = "./matlab_data_v1.csv"
        self.files = []

        self.X_train = []
        self.X_test = []
        self.Y_train = []
        self.Y_test = []

        self.model = LinearDiscriminantAnalysis(solver='lsqr')

        self.width = []
        self.height = []
        self.center_x = []
        self.center_y = []
        self.angle = []
        self.id = []

        #self.setImgs()

    def loadCSV(self):
        with open(self.path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = ',')

            for row in csv_reader:
                self.center_x.append(float(row[1]))
                self.center_y.append(float(row[2]))
                self.width.append(float(row[3]))
                self.height.append(float(row[4]))
                self.id.append(int(row[5]))
                self.angle.append(float(row[6]))

    def split_data(self, X, Y, test_size):
        self.X_train, self.X_test, self.Y_train, self.Y_test = train_test_split(X, Y, test_size = test_size, random_state = 0)

    def fit_data(self):
        self.model.fit(self.X_train, self.Y_train)

    def predict_class(self, data):
        return self.model.predict(data)

    def predict_probability(self, data):
        return self.model.predict_proba(data)

    def train_model(self):
        x1 = np.resize(np.array(classifier.width), (-1,1))
        x2 = np.resize(np.array(classifier.height), (-1,1))

        X = np.hstack((x1, x2))
        Y = np.array(classifier.id[:-1])

        print("X shape: ", X.shape)
        print("Y shape: ", Y.shape)

        self.split_data(X, Y, 0.7)
        self.fit_data()
        self.save_model()

    def save_model(self):
        filename = 'model.sav'
        pickle.dump(self.model, open(filename, 'wb'))

    def load_model(self):
        filename = 'model.sav'
        loaded_model = pickle.load(open(filename, 'rb'))
        self.model = loaded_model

    def classify(self, data):
        self.load_model()
        label = self.predict_class(data)
        prob = np.max(self.predict_probability(data))
        return int(label), float(prob)

if __name__ =="__main__":
    classifier = Classifier()
    classifier.loadCSV()

    classifier.train_model()
    prediction = classifier.predict_class(classifier.X_test)
    probability = classifier.predict_probability(classifier.X_test)
    score = cross_val_score(classifier.model, classifier.X_test, classifier.Y_test, cv=10)

    print(score.mean())
    print(score.std())

    test_array = classifier.X_test[1,:]
    print("test array size: ", test_array.shape)
    test = classifier.classify(test_array.reshape((-1,2)))
    print(test)
