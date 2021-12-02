import json
import cv2 as cv
import numpy as np
import csv
import os
from os import walk
import random
from datetime import datetime


path = "../data/V1/2021-11-24_v1/"
output_file_name = "matlab_data_v1.csv"
output_folder = "data_vis_v1"


#os.mkdir(output_folder)

all_files = os.listdir(path)


annotation_files = []
for file in all_files:
    if file[-5:]  == '.json':
        annotation_files.append(file)

with open(output_file_name, 'w', newline='') as open_file:
    writer = csv.writer(open_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for anno in annotation_files:
        print(anno)
        with open(path + anno) as json_file:
            data = json.load(json_file)
            image_name = data['imagePath']
            img = cv.imread(path + image_name)

            if len(data['shapes']) != 0:
                for annotation in data['shapes']:
                    rot_rect = cv.minAreaRect(np.float32(annotation['points']))
                    box_points = cv.boxPoints(rot_rect)
                    #p1 = [rot_rect[0]+]
                    # make sure that width and height are consistent
                    if rot_rect[-2][0] < rot_rect[-2][1]:
                        rot_rect = (rot_rect[0],(rot_rect[-2][1],rot_rect[-2][0]),rot_rect[-1]+90)
                    # make sure that no angle is above 90 or below -90
                    if rot_rect[-1] > 90:
                        rot_rect = (rot_rect[0],rot_rect[1],rot_rect[-1]-180)
                    if rot_rect[-1] < -90:
                        rot_rect = (rot_rect[0],rot_rect[1],rot_rect[-1]+180)
                    print(rot_rect)
                    class_name = annotation['label']
                    class_name = int(class_name[-1])
                    img_id = int(str(image_name).split("_")[-1].split(".")[0])
                    cx, cy = rot_rect[0]
                    width, height = rot_rect[1]
                    angle = rot_rect[2]
                    writer.writerow([img_id, cx, cy, width, height, class_name,angle])

                    box = cv.boxPoints(rot_rect) #for OpenCV 3.x
                    box = np.int0(box)
                    cv.drawContours(img,[box],0,(0,0,255),2)
            cv.imwrite(output_folder +"/"+image_name, img)
            print("saved file: "+ output_folder +"/"+image_name)
