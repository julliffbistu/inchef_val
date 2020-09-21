#! /usr/bin/env python
import roslib
import os,sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
#from tracker import Tracker
from sanitychecker import Checker
from kitchen.msg import objs
from kitchen.msg import events
from detector import Detector
from detector import DetectorType
import time

class mainprocessor:
    def __init__(self):
        rospy.init_node('processor_sanity')
        self.bridge = CvBridge()
        self.detector = Detector(DetectorType.MODEL_YOLOV4)
	self.checker = Checker()
        self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)
        self.objects_pub = rospy.Publisher("/processor/objs",objs,queue_size=10)
        self.events_pub = rospy.Publisher("/processor/events",Float32MultiArray, queue_size=10)
        self.objreadiness_pub = rospy.Publisher("/processor/objreadiness",Float32MultiArray, queue_size=10)
        self.foreignpos_pub = rospy.Publisher("/processor/foreignpos",Float32MultiArray, queue_size=10)
        
    def callback(self, image):
        
        prev_time = time.time()
        
	cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        detectedObjs = self.detector.detect(cv_image)
        
        sanityevents = self.checker.check(cv_image,detectedObjs)
        
        self.events_pub.publish(sanityevents.abnormalEvents)
        self.objreadiness_pub.publish(sanityevents.objReadiness)
        self.foreignpos_pub.publish(sanityevents.foreignObjPos)
        print(1/(time.time()-prev_time))
        #print('processing time:',(time.time()-prev_time)*1000)

        #trackedObjs = self.tracker.track(cv_image,detectedObjs) 
        #print(detectedObjs)
        #if(len(detectedObjs.objects_vector) > 0):
        #    self.objects_pub.publish(detectedObjs)
      
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('Quit')
            cv2.destroyAllWindows()       

if __name__ == '__main__':
    
    processor = mainprocessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down") 
    cv2.destroyAllWindows()
