#USE python2 !!!!!
import rospy
import os
import csv
from cv_bridge import CvBridge
import cv2
from rospy.timer import sleep
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Int32, Float32, String
from pose_estimation.msg import groundTruth
from pose_estimation.srv import pose_est_srv

class streamNode:
    def __init__(self):
        self.img_names = []
        self.path = "/home/user/workspace/src/pose_estimation/data/V4/2021-12-07_v1/"
        self.converter = CvBridge()
        self.pose_est = rospy.ServiceProxy("/pose_est",pose_est_srv)
        self.pub = rospy.Publisher("/camera/rgb/image_raw",Image,queue_size=20)
        self.pub_d = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=20)
        self.pub_groundTruth = rospy.Publisher("GT_rects", groundTruth, queue_size=20)
        self.rate = rospy.Rate(1)
        self.csv = None
        self.setup()
        self._getCorrCSV()

    def _getCorrCSV(self):
        files = os.listdir(self.path+"/..")
        for file in files:
            if file.endswith(".csv") and file.startswith("matlab"):
                self.csv = file
        if self.csv is None:
            print("COULD NOT FIND CSV FILE")
            exit(-1)

    def run_fromCSV(self):
        count =0
        with open(self.path+"../"+self.csv, mode ="r") as file:
            csvFile = csv.reader(file)
            csvFile = np.array(list(csvFile))
            for row in range(0, csvFile.shape[0],4):
                i = csvFile[row,0]
                gt = groundTruth()
                for offset in range(4):
                    idx = row+offset
                    annotation = csvFile[idx,:]
                    gt.cx.append(float(annotation[1]))
                    gt.cy.append(float(annotation[2]))
                    gt.width.append(float(annotation[3]))
                    gt.height.append(float(annotation[4]))
                    gt.label.append(str("obj_"+str(annotation[5])))
                    gt.angle.append(float(annotation[6]))
                gt.rgbFile = self.path+"RGB_"+str(i)+".png"
                gt.depthFile = self.path+"DEPTH_"+str(i)+".png"
    
                # send msgs                
                print("image pair: "+str(count))
                self.pub_groundTruth.publish(gt)
                count+=1
                sleep(1)
                ans = self.pose_est()
            
                # rgb = cv2.imread(self.path+"RGB_"+str(i)+".png")
                # # rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                # rgb_msg = self.converter.cv2_to_imgmsg(rgb,"bgr8")
                # self.pub.publish(rgb_msg)

                # depth = cv2.imread(self.path+"DEPTH_"+str(i)+".png")
                # depth=cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
                # depth_msg = ans = raw_input("Press enter to SEND the next image pair. Press 'q' to exit")
                #self.converter.cv2_to_imgmsg(depth, "passthrough")
                # self.pub_d.publish(depth_msg)

    def setup(self):
        files = os.listdir(self.path)
        for file in files:
            if file.endswith(".png") and file.startswith("RGB"):
                self.img_names.append(file)

    def run(self, verbose = False ):
        for i in range(len(self.img_names)):
            rgb = cv2.imread(self.path+"RGB_"+str(i)+".png")
            # rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            rgb_msg = self.converter.cv2_to_imgmsg(rgb,"bgr8")
            self.pub.publish(rgb_msg)

            depth = cv2.imread(self.path+"DEPTH_"+str(i)+".png")
            depth=cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
            depth_msg = self.converter.cv2_to_imgmsg(depth, "passthrough")
            self.pub_d.publish(depth_msg)
            if verbose:
                cv2.imshow("depth_"+str(i),depth)
                cv2.imshow("rgb_"+str(i),rgb)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            ans = raw_input("Press enter to continue to next image pair. Press 'q' to exit")
            if "q" == str(ans):
                exit()
        cv2.destroyAllWindows()
        self.pub.unregister()



if __name__ == "__main__":
    
    rospy.init_node("dataset_streamer_node")
    print("RUNNING")
    #try:
    sN = streamNode()
    sN.run_fromCSV()
    # sN.run()
    # except:
    #     print("SOMETHING WRONG")
    #     exit(-1)