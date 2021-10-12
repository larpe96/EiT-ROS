#!/usr/bin/env python
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np

from ur_robot_pkg.srv import p2p_cmove, jointQs, RobState, CurrTCPPose


class Robot(RTDEControl, RTDEReceive, RTDEIO):
    def __init__(self, _ip):
        RTDEControl.__init__(self, _ip)
        RTDEIO.__init__(self, _ip,)
        RTDEReceive.__init__(self, _ip,[])
        self.ip = _ip

    def cb_test(self, req):
        hello_str = "SERVICE: hello World %s" % rospy.get_time()
        return True, hello_str

    def cb_p2p_cmove(self, req):
        req_pose = req.pose
        vel = req.vel
        acc = req.acc
        req_q = R.from_quat([req_pose.orientation.x, req_pose.orientation.y, req_pose.orientation.z, req_pose.orientation.w])
        req_pos = [req_pose.position.x,req_pose.position.y,req_pose.position.z]
        ur_pose = np.ravel([req_pos,req_q.as_rotvec()])
        b = self.moveL(ur_pose,vel, acc, req.move_async)
        if b:
            str = "OK"
        else:
            str = "BAD- Failed moving"
        return b, str

    def cb_get_joint_state(self, req):
        ur_Q = self.getActualQ()
        ur_dQ = self.getActualQd()
        ur_curr = self.getActualCurrent()
        return ur_Q, ur_dQ, ur_curr

    def cb_get_RobState(self, req):
        _isCon = self.isConnected()
        _isProStop = self.isProtectiveStopped()
        _isEmStop = self.isEmergencyStopped()
        _isStead = self.isSteady()

        return _isCon, _isProStop, _isEmStop, _isStead

    def cb_get_TCP_Pose(self, req):
        tcp =self.getActualTCPPose()
        point_ = Point(tcp[0], tcp[1], tcp[2])
        _ori = R.from_rotvec(tcp[3:6]).as_quat()
        quat = Quaternion(_ori[0],_ori[1],_ori[2],_ori[3])
        return Pose(point_, quat)

class Servicenode(Robot):
    def __init__(self, _ip):
        self.robot = Robot(_ip)
        rospy.init_node('UR_robot_interface_node', anonymous=True)
        self.init_serivices()

    def init_serivices(self):
        srv_test = rospy.Service("urtest/ur_test_srv", Trigger, self.robot.cb_test)
        srv_p2p_Cmove = rospy.Service("SET/p2p_Cmove_srv", p2p_cmove, self.robot.cb_p2p_cmove)
        srv_jointQ = rospy.Service("GET/jointQ_srv", jointQs, self.robot.cb_get_joint_state)
        srv_rob_state = rospy.Service("GET/robot_state_srv", RobState, self.robot.cb_get_RobState)
        srv_Tcp_pose = rospy.Service("GET/tcp_pose_srv", CurrTCPPose, self.robot.cb_get_TCP_Pose)
        rospy.spin()    


# def test(robot):
#     pub = rospy.Publisher('msg_test', String, queue_size=20)
#     rospy.init_node('UR_robot', anonymous=True)
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         hello_str = "hello World %s" % rospy.get_time()
#         pub.publish(hello_str)
#         rate.sleep()


if __name__ =="__main__":
    # _ip = "127.0.0.1" # VM
    _ip = "192.168.1.68" # Robot

    try:
        ser_node = Servicenode(_ip)
    except rospy.ROSInterruptException:
        pass
