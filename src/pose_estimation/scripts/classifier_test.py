import os
import cv2
import numpy as np


def DiffNorm(img, background):
    
    absdiff = cv2.absdiff(img, background)
    # diff_square = absdiff.mul(absdiff)
    abc = []

    # print(absdiff.size)
    a, b, c = cv2.split(absdiff, [])

    diff_sum = a+b+c
    # img_ = cv2.imshow("HELLO", diff_sum)
    # cv2.waitKey(500)
    # diff_norm = None
    # diff_norm= cv2.sqrt(diff_sum)
    return diff_sum

def ErodeAndDilate(img, erodeSize, dilateSize):
    struct_element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))

    for i in range(erodeSize):
        cv2.erode(img, struct_element,dst=img)
    
    for i in range(dilateSize):
        cv2.dilate(img,struct_element, dst=img)
    return img

Good_list= [7, 4, 1, 8, 5, 2, 6, 9 ] # obj1



class Obj_class:
    def __init__(self):
        self.path = "/home/user/workspace/src/pose_estimation/data/obj_1/"
        #self.background = cv2.imread("/home/user/workspace/src/pose_estimation/src/backgroundEmpty.png") 
        self.files = []
        #self.setImgs()

    def setImgs(self):
        files = os.listdir(self.path)
        for file in files:
            if file.endswith(".png"):
                self.files.append(file)

    def run(self):
        humoments_tot = np.zeros([8,7])
        i=0
        for number in Good_list:
            name = "rgb_obj1_"+str(number)+".png"
            path_name = self.path+name
            img = cv2.imread(path_name,cv2.IMREAD_GRAYSCALE)

            # new_img = DiffNorm(img, self.background)
            bin_img = cv2.threshold(img, 150, 255,cv2.THRESH_BINARY)[1]
            bin_img = ErodeAndDilate(bin_img,3,2)

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

        np.savetxt("test.csv", humoments_tot,delimiter=',')




if __name__ =="__main__":
    classifier = Obj_class()
    classifier.run()