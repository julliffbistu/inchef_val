#! /usr/bin/env python

import os,sys
import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
#from tracker import Tracker
from sanitychecker import Checker
from kitchen.msg import obj
from kitchen.msg import objs
from kitchen.msg import events
from kitchen.msg import obj_A
from kitchen.msg import objs_A

class mainprocessor:
    def __init__(self):
        rospy.init_node('processor_sanity')
        rospy.set_param('objreadinessflag', 0)
        rospy.set_param('sanitycheckflag', 1)
        self.bridge = CvBridge()
        self.checker = Checker()

        self.objects_sub = rospy.Subscriber("processor/objs", objs, self.callback)
        self.events_pub = rospy.Publisher("/processor/events",Float32MultiArray, queue_size=10)
        self.objreadiness_pub = rospy.Publisher("/processor/objreadiness",Float32MultiArray, queue_size=10)
        self.foreignpos_pub = rospy.Publisher("/processor/foreignpos",Float32MultiArray, queue_size=10)
        
    def callback(self, data):
	    
        flag = 0
        if rospy.has_param('sanitycheckflag'):
		    flag = rospy.get_param('sanitycheckflag')
       
        if flag == 0:
            cv2.destroyAllWindows() 
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(data.rgb_img,"bgr8")         
        detectedObjs = objs_A()

        for i in range(len(data.objects_vector[0].id)):
            detectObj = obj_A()
            detectObj.id = data.objects_vector[0].id[i]
            detectObj.classname = data.objects_vector[0].classname[i]
            detectObj.probability = data.objects_vector[0].score[i]
            detectObj.roi = data.objects_vector[0].roi[i]
            detectedObjs.objects_vector.append(detectObj)

        sanityevents = self.checker.check(cv_image,detectedObjs)
        # print(sanityevents)
        
        self.events_pub.publish(sanityevents.abnormalEvents)
        self.objreadiness_pub.publish(sanityevents.objReadiness)
        self.foreignpos_pub.publish(sanityevents.foreignObjPos)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     rospy.signal_shutdown('Quit')
        #     cv2.destroyAllWindows()       

if __name__ == '__main__':
    
    processor = mainprocessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down") 
    cv2.destroyAllWindows()
