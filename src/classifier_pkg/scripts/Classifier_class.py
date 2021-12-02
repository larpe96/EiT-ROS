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


# obj_1_1.png
# obj_1_2.png
# obj_1_3.png
# obj_1_20.png
#
# obj_2_1.png
# obj_2_2.png
# obj_2_20.png

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
        #self.model = MLPClassifier(hidden_layer_sizes=100, activation='relu', solver='adam', learning_rate='constant', learning_rate_init=0.1, max_iter=1000)
        #self.model = GaussianNB(priors=None, var_smoothing=1e-5)
        #self.model = svm.SVC(kernel='poly')

        self.width = []
        self.height = []
        self.center_x = []
        self.center_y = []
        self.angle = []
        self.id = []
        self.humoments = []

    def bin_img2huMom(self, bin_img):
        moments = cv2.moments(bin_img)
        return cv2.HuMoments(moments)

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


    def loadHuDataset(self):
        all_files = os.listdir(self.huDatasetPath)
        for file in all_files:
            #Hu moments

            bin_img = cv2.imread(self.huDatasetPath + "/" + file, cv2.IMREAD_GRAYSCALE)#"RGB_" + name + ".png")
            print(bin_img.shape)
            # cv2.imshow("img", img)
            # cv2.waitKey(0)
            # #convert to
            # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            #threshold
            #bin_img = cv2.threshold(img_gray, 2, 255, cv2.THRESH_BINARY)[1]
            # print("bin img shape: ", np.array(bin_img).shape)
            hu_mom = self.bin_img2huMom(bin_img)
            #print(hu_mom, np.array(hu_mom))
            # Append humoments
            print(hu_mom)
            log_mom = (-1)*np.log10(abs(hu_mom))*np.sign(hu_mom)
            # #print(log_mom)
            self.humoments.append(log_mom)
            self.id.append(int(file.split("_")[1]))


    def saveHuMom2csv(self):
        print("SAVING DATA")
        # with open("DATA.csv") as csvFile:
        #     writer = csv.writer(csvFile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        #     for row in self.humoments:
        #         writer.writerow([row[0], row[1], row[2],[row[3], row[4], row[5], [row[6]])
        print(np.array(self.humoments).shape)
        np.savetxt("log_DATA.csv",np.array(self.humoments)[:,:,-1], delimiter=",", fmt='%d')
        np.savetxt("log_LABEL.csv",np.array(self.id), delimiter=",", fmt='%d')

    def split_data(self, X, Y, test_size):
        self.X_train, self.X_test, self.Y_train, self.Y_test = train_test_split(X, Y, test_size = test_size, random_state = 0)

    def fit_data(self):
        self.model.fit(self.X_train, self.Y_train)

    def predict_class(self, data):
        return self.model.predict(data)

    def predict_probability(self, data):
        return self.model.predict_proba(data)

    def train_model(self):
        x1 = np.resize(np.array(self.width), (-1,1))
        x2 = np.resize(np.array(self.height), (-1,1))

        X = np.hstack((x1, x2))
        Y = np.array(self.id[:-1])

        #X = np.array(self.humoments)[:,:,-1]
        #Y = np.array(self.id)

        #print("X shape: ", np.array(X)[:,:,-1].shape)
        #print("Y shape: ", np.array(Y).shape)

        #plt.scatter(X[:,0], X[:,1])
        #plt.show()

        self.split_data(X, Y, 0.3)
        self.fit_data()
        self.save_model()

    def save_model(self):
        filename = 'model.sav'
        pickle.dump(self.model, open(filename, 'wb'))

    def load_model(self):
        filename = '/home/user/workspace/src/classifier_pkg/scripts/model.sav'
        loaded_model = pickle.load(open(filename, 'rb'))
        self.model = loaded_model

    def setMethod(self, _method):
        self.method = _method

    def classify(self, data):
        self.load_model()

        data = np.flip(np.sort(np.array(data).reshape(1,-1)))
        print("data sorted: ", data)
        #data = np.array(data).reshape(1,-1)

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

    # test_array = classifier.X_test[1,:]
    # print("test array size: ", test_array.shape)
    # test = classifier.classify(test_array.reshape((-1,2)))
    # print(test)
    #
    #
    # data = [[30.405588150024414, 29.698482513427734],
    #         [32.16920852661133, 103.17975616455078],
    #         [33.58687973022461, 126.35648345947266]]
    #
    # for arr in data:
    #     print(arr)
    #     print(np.flip(np.sort(arr)))
    #     arr2 = np.flip(np.sort(arr))
    #     array = np.array(arr2).reshape(-1,1)
    #     label, prob = classifier.classify(array)
    #     print("label: ", label)
    #     print("prob: ", prob)
