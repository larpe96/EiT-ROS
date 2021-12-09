#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from Classifier_class import Classifier

from classifier_pkg.srv import classify_detections
#from classifier_pkg.msg import Classification_params

class Classifier_ros_node:
    def __init__(self) -> None:
        self.cf = Classifier()
        rospy.init_node("Classifier_ros_node")
        rospy.Service("classify_detections_srv",classify_detections,self.cb_classify_detections)
        rospy.spin()

    def cb_classify_detections(self,req):
        names = []
        masks = []
        
        for param in req.params:
            data = [param.width, param.height]
            label, prob = self.cf.classify(data)
            mask = False
            class_label = "False detection"
            #print("data: ", data)
            #print("prob: ", prob)
            #print("label: ", label)
            if param.width > 15 and param.height > 15:
                #mask = False
                #class_label = "False detection"
                if 1 >= prob > 1e-4:
                    mask = True
                    class_label = "obj_"+str(label)
                else:
                    mask = False
                    class_label = "False detection"
            names.append(class_label)
            masks.append(mask)

        print(names)
        print(masks)
        
        return names, masks

    

if __name__ == "__main__":
    cf_rn = Classifier_ros_node()
