import csv
import os
from typing import Any
import cv2
import numpy as np
from numpy.core.defchararray import center
import pandas as pd
import sklearn






class Obj_class:
    def __init__(self):
        self.path = "/home/user/workspace/src/pose_estimation/data/V0"
        #self.background = cv2.imread("/home/user/workspace/src/pose_estimation/src/backgroundEmpty.png")
        self.files = []
        #self.setImgs()
        self.name = "data_v0.csv"
        self.load_csv(self.path+"/"+ self.name)


        
    def load_csv(self, path):
        with open(path) as csv_file:
            csv.reader(csv_file, delimiter=",")
            self.img_names = []
            self.center_x = []
            self.center_y = []
            self.width = []
            self.height = []
            self.label = []
            self.angle = []
            for row in csv_file:
                self.img_names.append(row[0])
                self.center_x.append(float(row[1]))
                self.center_y.append(float(row[2]))
                self.width.append(float(row[3]))
                self.height.append(float(row[4]))
                self.label.append(row[5])
                self.angle.append(float(row[6]))
            

    def setImgs(self):
        files = os.listdir(self.path)
        for file in files:
            if file.endswith(".png"):
                self.files.append(file)

    def calc_huMoments(self):
        humoments_tot = np.zeros([8,7])
        i=0
        for number, file in enumerate(self.files):
            name = "rgb_obj1_"+str(number)+".png"
            path_name = self.path+name
            img = cv2.imread(path_name,cv2.IMREAD_GRAYSCALE)

            # new_img = DiffNorm(img, self.background)
            bin_img = cv2.threshold(img, 150, 255,cv2.THRESH_BINARY)[1]

            moments = cv2.moments(bin_img)
            hu_moments = cv2.HuMoments(moments)

            humoments_tot[i][0]= hu_moments[0]
            humoments_tot[i][1]= hu_moments[1]
            humoments_tot[i][2]= hu_moments[2]
            humoments_tot[i][3]= hu_moments[3]
            humoments_tot[i][4]= hu_moments[4]
            humoments_tot[i][5]= hu_moments[5]
            humoments_tot[i][6]= hu_moments[6]
            i +=1
            # print(humoments_tot)

            # # if img == None:
            # #     print("Could not find image in "+ path_name)
            # # else:
            im2, contours, hierachy = cv2.findContours(bin_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) >1:
                cv2.drawContours(img, contours, 1, (0,255,0),3)
            cv2.imshow(name,img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        #np.savetxt("test.csv", humoments_tot,delimiter=',')




if __name__ =="__main__":
    classifier = Obj_class()
