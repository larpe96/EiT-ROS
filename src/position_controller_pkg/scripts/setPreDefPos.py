#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from ur_robot_pkg.srv import CurrTCPPose



if __name__ == "__main__":
    rospy.init_node("SetPoses")
    rospy.wait_for_service("GET/tcp_pose_srv")
    get_tcp = rospy.ServiceProxy("GET/tcp_pose_srv", CurrTCPPose)
    enabl_freeDrive = rospy.ServiceProxy("SET/enable_teach", Trigger)
    disabl_freeDrive = rospy.ServiceProxy("SET/disable_teach", Trigger)

    answer = input("would you like to add a pose? (y/n): ")
    i =0
    while(answer != "n"):
        res_en = enabl_freeDrive()
        if not res_en.success:
            print(res_en.message)
            print("Terminating program")
            break
        input("move to desired pos and press enter")
        dis_res = disabl_freeDrive()
        if not dis_res.success:
            print(dis_res.message)
            print("Terminating program")
            break
        pos_res = get_tcp()
        file = open("../etc/"+str(i)+".eit", "w")
        file.write(str(pos_res.position.position.x)+"\n")
        file.write(str(pos_res.position.position.y)+"\n")
        file.write(str(pos_res.position.position.z)+"\n")
        file.write(str(pos_res.position.orientation.x)+"\n")
        file.write(str(pos_res.position.orientation.y)+"\n")
        file.write(str(pos_res.position.orientation.z)+"\n")
        file.write(str(pos_res.position.orientation.w)+"\n")
        file.close()
        i = i+1


        answer = input("would you like to add a new pose? (y/n): ")

    res = disabl_freeDrive()
    print(res.success, res.message)
