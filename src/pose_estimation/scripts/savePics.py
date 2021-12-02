import cv_bridge
import rospy
import cv2
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import os
from datetime import date


class SaveImgClass:
    def __init__(self):
        self.Pardir = "/home/user/workspace/src/pose_estimation/data/"
        self.conv = cv_bridge.CvBridge()
        rospy.wait_for_message("/camera/rgb/image_raw",Image)
        rospy.wait_for_message("/camera/depth/image_raw", Image)
        rospy.Subscriber(name="/camera/rgb/image_raw", data_class=Image, callback=self.set_rgb)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.set_depth)
        rospy.Service("saveImgs",Trigger, self.saveImg)
        self.rgb = None
        self.depth = None
        self.folder_name =date.today()
        self.noo =0
        self.path2Dir =""
        self.lock = False
        self.makeDir()
        os.chdir(self.path2Dir)
        print("Created folder: " +self.path2Dir)

    def makeDir(self):
        i=0
        while os.path.exists(self.Pardir+str(self.folder_name)+"_v"+str(i)):
            i= i+1
        self.path2Dir = self.Pardir+str(self.folder_name)+"_v"+str(i)
        os.mkdir(self.path2Dir)

    def set_rgb(self, data):
        if not self.lock:
            self.rgb = self.conv.imgmsg_to_cv2(data, desired_encoding="bgr8")

    def set_depth(self, data):
        if not self.lock:
            self.depth = self.conv.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def saveImg(self,msg):
        print("HELLO")
        if self.rgb is not None and self.depth is not None:
            self.lock = True
            RGBName = "RGB_"+str(self.noo)
            depthName = "DEPTH_"+str(self.noo)
            self.noo += 1
            # cv2.imshow(depthName, self.depth)
            # cv2.imshow(RGBName,self.rgb)
            # cv2.waitKey(500)
            # cv2.destroyAllWindows()
            print("saving imgs")
            cv2.imwrite(RGBName+".png",self.rgb)
            cv2.imwrite(depthName+".png",self.depth)
            self.lock = False
            return True, "saved imgs"
        else:
            return False, "Unable to save imgs"

if __name__ =="__main__":
    rospy.init_node("SaveImgs")
    imgGrapper = SaveImgClass()
    rospy.spin()

