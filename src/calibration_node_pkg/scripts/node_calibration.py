#!/usr/bin/env python
import cv_bridge
import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import cv2

from ur_robot_pkg.srv import CurrTCPPose
from position_controller_pkg.srv import Pre_def_pose
from cv_bridge import CvBridge, CvBridgeError

from scipy.spatial.transform import Rotation as R

import time

def convertFromQuatToRotvec(pos_res):
    r = R.from_quat([pos_res.position.orientation.x,pos_res.position.orientation.y,pos_res.position.orientation.z,pos_res.position.orientation.w])
    return r.as_rotvec()

class CalibrationNode():

    def __init__(self):
        rospy.init_node("calibration_recording")
        self.conv = CvBridge()
        self.image_data = None

        self.img_path = "../data/"
        
        self.pose_list = ["cali_0","cali_1","cali_2","cali_3","cali_4","cali_5","cali_6","cali_7","cali_8","cali_9","cali_10","cali_11","cali_12","cali_13","cali_14","cali_15","cali_16"]
        
        rospy.wait_for_service("GET/tcp_pose_srv")
        self.get_tcp = rospy.ServiceProxy("GET/tcp_pose_srv", CurrTCPPose)
        
        rospy.wait_for_service("move2_def_pos_srv")
        self.move2_def_pose = rospy.ServiceProxy("move2_def_pos_srv", Pre_def_pose)

        rospy.Subscriber("/camera/rgb/image_raw", Image, self.onImage)

    def onImage(self,data):
        try:
            self.image_data = self.conv.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #self.image_data = self.conv.imgmsg_to_cv2(data.data,"bgr8")

    def run_collection(self):
        for name in self.pose_list:
            print("Moving robot to: ", name)
            self.move2_def_pose(name)

            
            pos_res = self.get_tcp()
            print("Tcp location: ",pos_res)

            file = open("../data/"+name+".csv", "w")
            file.write(str(pos_res.position.position.x)+"\n")
            file.write(str(pos_res.position.position.y)+"\n")
            file.write(str(pos_res.position.position.z)+"\n")
            rot_vec_ori = convertFromQuatToRotvec(pos_res)
            file.write(str(rot_vec_ori[0])+"\n")
            file.write(str(rot_vec_ori[1])+"\n")
            file.write(str(rot_vec_ori[2])+"\n")
            file.close()
            print("Capturing image")
            cv2.imwrite(self.img_path + name + ".png",self.image_data)

            time.sleep(1)

cn = CalibrationNode()

cn.run_collection()

        
        


