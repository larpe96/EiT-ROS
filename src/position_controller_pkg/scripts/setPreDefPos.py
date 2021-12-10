#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from ur_robot_pkg.srv import CurrTCPPose


def to_file(pos_res,prefix,z):
    file = open("../etc/"+prefix+str(i)+".eit", "w")
    file.write(str(pos_res.position.position.x)+"\n")
    file.write(str(pos_res.position.position.y)+"\n")
    file.write(str(z)+"\n")
    file.write(str(0.819152)+"\n")
    file.write(str(0.5735764)+"\n")
    file.write(str(0.0)+"\n")
    file.write(str(0.0)+"\n")
    file.close()

if __name__ == "__main__":
    rospy.init_node("SetPoses")
    rospy.wait_for_service("GET/tcp_pose_srv")
    get_tcp = rospy.ServiceProxy("GET/tcp_pose_srv", CurrTCPPose)
    enabl_freeDrive = rospy.ServiceProxy("SET/enable_teach", Trigger)
    disabl_freeDrive = rospy.ServiceProxy("SET/disable_teach", Trigger)
    i =0
    answer = 'y'
    while(answer != "n"):
        res_en = enabl_freeDrive()
        if not res_en.success:
            print(res_en.message)
            print("Terminating program")
            break
        answer = input("move to desired pos and press enter")
        dis_res = disabl_freeDrive()
        if not dis_res.success:
            print(dis_res.message)
            print("Terminating program")
            break
        pos_res = get_tcp()
        to_file(pos_res,"place_",-0.020)

        to_file(pos_res,"above_", 0.1)

        i = i+1
        print("saving file: " +str(i) + ".eit")

    res = disabl_freeDrive()
    print(res.success, res.message)
