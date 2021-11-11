#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


class UR_recieve(RTDEReceive):
    def __init__(self, _ip):
        self.ip = _ip
        RTDEReceive.__init__(self, _ip)
    
    def get_robot_qs(self):
        qs = self.getActualQ()
        dim = MultiArrayDimension()
        dim.label="robot Qs"
        dim.size = 1
        dim.stride =6

        layout = MultiArrayLayout()
        layout.dim = [dim]
        layout.data_offset = 0
        qs_formatted = Float32MultiArray(layout, qs)
        return qs_formatted


class UR_recieve_node:
    def __init__(self):
        rospy.init_node("UR_Recieve_node",anonymous=True)
        ip = self.get_robot_ip()
        self.robotRecieve = UR_recieve(ip)
        print("UR_Recieve connected successfully")
        self.datacollectionFreq = 60 # Hz
        self.pub_qs_msg = rospy.Publisher("robot_joint_vals",Float32MultiArray, queue_size=20)
        self.stream_data()


    def get_robot_ip(self):
        rospy.wait_for_service("GET/robot_ip")
        getIP = rospy.ServiceProxy("GET/robot_ip", Trigger)
        res = getIP()
        return res.message

    def stream_data(self):
        rate = rospy.Rate(self.datacollectionFreq)
        while not rospy.is_shutdown():
            # Robot joint values
            qs = self.robotRecieve.get_robot_qs()
            self.pub_qs_msg.publish(qs)
            rate.sleep()



if __name__ == "__main__":
    try:
        node= UR_recieve_node()
    except:
        print("UR_Recieve was unable to connect")
