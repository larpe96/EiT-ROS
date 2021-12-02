import csv
import os
from typing import Any
import cv2
import numpy as np
import csv
import pickle
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_val_score
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.neural_network import MLPClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn import svm
from matplotlib import pyplot as plt

def crop_rect(img, rect):
    #https://jdhao.github.io/2019/02/23/crop_rotated_rectangle_opencv/
    # get the parameter of the small rectangle
    center, size, angle = rect[0], rect[1], rect[2]
    center, size = tuple(map(int, center)), tuple(map(int, size))

    # get row and col num in img
    height, width = img.shape[0], img.shape[1]

    # calculate the rotation matrix
    M = cv2.getRotationMatrix2D(center, angle, 1)
    # rotate the original image
    img_rot = cv2.warpAffine(img, M, (width, height))

    # now rotated rectangle becomes vertical, and we crop it
    img_crop = cv2.getRectSubPix(img_rot, size, center)

    return img_crop, img_rot

class Classifier:
    def __init__(self) -> None:
        self.method = None
        self.path = "./matlab_data_v1.csv"
        self.huDatasetPath = "/home/user/workspace/src/pose_estimation/data/V4"
        self.files = []

        self.X_train = []
        self.X_test = []
        self.Y_train = []
        self.Y_test = []

        self.model = LinearDiscriminantAnalysis(solver='lsqr')
     
        # Parameters used for prediction
        self.width = []
        self.height = []
        self.center_x = []
        self.center_y = []
        self.angle = []
        self.id = []
        self.humoments = []

    # Calculates humoments from a binary image
    def bin_img2huMom(self, bin_img):
        moments = cv2.moments(bin_img)
        return cv2.HuMoments(moments)

    # Load the training dataset
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

    # Dataset used for humoments
    def loadHuDataset(self):
        all_files = os.listdir(self.huDatasetPath)
        for file in all_files:
            #Hu moments
            bin_img = cv2.imread(self.huDatasetPath + "/" + file, cv2.IMREAD_GRAYSCALE)#"RGB_" + name + ".png")
            hu_mom = self.bin_img2huMom(bin_img)
            log_mom = (-1)*np.log10(abs(hu_mom))*np.sign(hu_mom)
            
            self.humoments.append(log_mom)
            self.id.append(int(file.split("_")[1]))

    # Save humoments as csv
    def saveHuMom2csv(self):
        print("SAVING DATA")
        print(np.array(self.humoments).shape)
        np.savetxt("log_DATA.csv",np.array(self.humoments)[:,:,-1], delimiter=",", fmt='%d')
        np.savetxt("log_LABEL.csv",np.array(self.id), delimiter=",", fmt='%d')

    def split_data(self, X, Y, test_size):
        self.X_train, self.X_test, self.Y_train, self.Y_test = train_test_split(X, Y, test_size = test_size, random_state = 0)

    def fit_data(self):
        self.model.fit(self.X_train, self.Y_train)

    # Returns the label of the prediction
    def predict_class(self, data):
        return self.model.predict(data)

    # Returns the probability of the prediction
    def predict_probability(self, data):
        return self.model.predict_proba(data)

    # Train the model
    def train_model(self):
        x1 = np.resize(np.array(self.width), (-1,1))
        x2 = np.resize(np.array(self.height), (-1,1))

        X = np.hstack((x1, x2))
        Y = np.array(self.id[:-1])

        #X = np.array(self.humoments)[:,:,-1]
        #Y = np.array(self.id)

        self.split_data(X, Y, 0.3)
        self.fit_data()
        self.save_model()

    # Save the model
    def save_model(self):
        filename = 'model.sav'
        pickle.dump(self.model, open(filename, 'wb'))

    # Load the model
    def load_model(self):
        filename = '/home/user/workspace/src/classifier_pkg/scripts/model.sav'
        loaded_model = pickle.load(open(filename, 'rb'))
        self.model = loaded_model

    # Set the method used for classification
    def setMethod(self, _method):
        self.method = _method

    # Classifies the object and returns a label and a probability
    def classify(self, data):
        self.load_model()
        data = np.flip(np.sort(np.array(data).reshape(1,-1)))

        label = self.predict_class(data)
        prob = np.max(self.predict_probability(data))

        return int(label), float(prob)


if __name__ =="__main__":
    classifier = Classifier()
    classifier.loadCSV()
    #classifier.loadHuDataset()
    #classifier.saveHuMom2csv()

    classifier.train_model()
    prediction = classifier.predict_class(classifier.X_test)
    probability = classifier.predict_probability(classifier.X_test)
    score = cross_val_score(classifier.model, classifier.X_test, classifier.Y_test, cv=2)

    print(score.mean())
    print(score.std())