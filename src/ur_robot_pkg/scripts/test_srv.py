#!/usr/bin/env python
import rospy
from rospy import rostime
from rospy.impl.tcpros_service import wait_for_service
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

from ur_robot_pkg.srv import p2p_cmove, jointQs, RobState, CurrTCPPose, set_TCP_offset, get_TCP_offset


if __name__ == "__main__":
    rospy.init_node("UR_node_tester", anonymous=True)

    print("Ur srv test")
    rospy.wait_for_service("urtest/ur_test_srv")
    ur_test = rospy.ServiceProxy("urtest/ur_test_srv", Trigger)
    resp = ur_test()
    print("========resp:========\n%s\n========END========\n"%resp)

    print("jointQ")
    rospy.wait_for_service("GET/jointQ_srv")
    jointqs = rospy.ServiceProxy("GET/jointQ_srv", jointQs)
    resp_joint = jointqs()
    print("========resp:========\n%s\n========END========\n"%resp_joint)

    print("RobState")
    rospy.wait_for_service("GET/robot_state_srv")
    robstate = rospy.ServiceProxy("GET/robot_state_srv", RobState)
    resp_robState = robstate()
    print("========resp:========\n%s\n========END========\n"%resp_robState)

    print("currTcp")
    rospy,wait_for_service("GET/tcp_pose_srv")
    tcpPose = rospy.ServiceProxy("GET/tcp_pose_srv", CurrTCPPose)
    resp_tcp = tcpPose()
    print("========resp:========\n%s\n========END========\n"%resp_tcp)

    # print("MOVE ROBOT")
    # ans = input("ARE YOU READY TO PRESS EM-STOP? y/n: ")
    # while ans != "y":
    #     ans = input("ARE YOU READY TO PRESS EM-STOP? y/n: ")
    # rospy.wait_for_service("SET/p2p_Cmove_srv")
    # moveRobot = rospy.ServiceProxy("SET/p2p_Cmove_srv", p2p_cmove)
    # r = R.from_euler('xyz',[0,  math.pi, 0]).as_quat()
    # _q = Quaternion(r[0],r[1], r[2], r[3])
    # _p = Point(0.2, -0.4, 0.20)
    # pose = Pose(position=_p, orientation=_q)
    # vel = float(0.25)
    # acc = float(0.1 )
    # resp_mov = moveRobot(pose, vel, acc, False) 
    # print("========resp:========\n%s\n========END========\n"%resp_mov)


    print("enable Free drive")
    rospy.wait_for_service("SET/enable_teach")
    enabl_freeDrive = rospy.ServiceProxy("SET/enable_teach", Trigger)
    resp_enab = enabl_freeDrive()
    print("========resp:========\n%s\n========END========\n"%resp_enab)
    
    
    print("disable Free drive")
    rospy.wait_for_service("SET/enable_teach")
    disabl_freeDrive = rospy.ServiceProxy("SET/disable_teach", Trigger)
    resp_disp = disabl_freeDrive()
    print("========resp:========\n%s\n========END========\n"%resp_disp)


    print("set TCP offset")
    rospy.wait_for_service("SET/tcp_offset_srv")
    set_TCP_offset = rospy.ServiceProxy("SET/tcp_offset_srv",set_TCP_offset)
    set_off_r = R.from_euler('xyz',[0,  0, 0]).as_quat()
    set_off_q = Quaternion(set_off_r[0],set_off_r[1], set_off_r[2], set_off_r[3])
    set_off_p = Point(0.0, -0.0, 0.0)
    set_off_pose = Pose(position=set_off_p, orientation=set_off_q)
    set_off_resp = set_TCP_offset(set_off_pose)
    print("========resp:========\n%s\n========END========\n"%set_off_resp)



    # 34 mm offset in tcp z