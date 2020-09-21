#!/usr/bin/env python

import cv2
import numpy as np
import os
import sys
import tensorflow as tf
import maskrcnnval
import rospy
from kitchen.msg import objs

i = 1
img_global=np.zeros([540,960,3],dtype=np.uint8)
img_hd_global=np.zeros([1080,1920,3],dtype=np.uint8)
def main():
    rospy.init_node("maskrcnn_node")
    detect_objs_pub = rospy.Publisher("processor/objs",objs,queue_size=1)
    detect_objs = None
    global i
    #global img_global
    #global img_hd_global
    while 1:
        img = cv2.imread('./model_val/test_datasets/'+ str(i) + '.jpg')
        if img is None:
            break
        #img_global = img
        img_hd = cv2.imread("./model_val/img_hd.png")
        #img_hd_global = img_hd
        detect_objs = maskrcnnval.model_detection(img,img_hd)
        detect_objs_pub.publish(detect_objs)
        #cv2.imshow("img",img)
        #cv2.imshow("img_hd",img_hd)
        cv2.waitKey(10)
        print("img number is: ",i)
        i = i + 1
        

if __name__ == '__main__':  
    main()
    cv2.destroyAllWindows()

 

    
