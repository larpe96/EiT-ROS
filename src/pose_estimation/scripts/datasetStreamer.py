#USE python2 !!!!!
import rospy
import os
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

class streamNode:
    def __init__(self):
        self.img_names = []
        self.path = "/home/user/workspace/src/pose_estimation/data/V1/2021-11-24_v1/"
        self.converter = CvBridge()
        self.pub = rospy.Publisher("/camera/rgb/image_raw",Image,queue_size=20)
        self.pub_d = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=20)
        self.rate = rospy.Rate(1)
        self.setup()

    def setup(self):
        files = os.listdir(self.path)
        for file in files:
            if file.endswith(".png") and file.startswith("RGB"):
                self.img_names.append(file)

    def run(self, verbose = False ):
        for i in range(len(self.img_names)):
            rgb = cv2.imread(self.path+"RGB_"+str(i)+".png")
            depth = cv2.imread(self.path+"DEPTH_"+str(i)+".png")
            rgb_msg = self.converter.cv2_to_imgmsg(rgb,"bgr8")
            depth_msg = self.converter.cv2_to_imgmsg(depth, "passthrough")
            self.pub.publish(rgb_msg)
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
    
    print("hello world")
    rospy.init_node("dataset_streamer_node")
    print("a")
    #try:
    sN = streamNode()
    sN.run()
    # except:
    #     print("SOMETHING WRONG")
    #     exit(-1)