# -*- coding: UTF-8 -*-
import cv2

import numpy as np
from matplotlib import pyplot as plt
import math
import imutils
import os
import rosnode
import rospy
import sys

from collections import Counter
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

minv = 1000
maxv = 100
hs_pan = 0
hs_cai = 0
hs_pot = 0
hs_net = 0

z_history_pan=[0.0]
z_history_cai=[0.0]
z_history_pot=[0.0]
z_history_net=[0.0]
cnt=0

bar_list = np.zeros(16,dtype=float)
send_barlist = np.zeros(16,dtype=float)
def callback_bar_calibration(data):
    global bar_list
    bar_list=data.data
    
    #calibration_bar_pub = rospy.Publisher("calibration/bar",bar_calib,queue_size=1)
    #calibration_bar_pub.publish(bar_calib_obj)

def callback_dep_hd(image):
    global bar_list
    now = rospy.get_rostime()
    # create OpenCV depth image using defalut passthrough encoding
    bridge=CvBridge()
    print("~~~~~~~~~~~~~~~~~~~",bar_list)

    cf_u_pan = int(bar_list[0])
    cf_v_pan = int(bar_list[1])

    cf_u_cai = int(bar_list[3])
    cf_v_cai = int(bar_list[4])

    cf_u_pot = int(bar_list[6])
    cf_v_pot = int(bar_list[7])

    cf_u_net = int(bar_list[9])
    cf_v_net = int(bar_list[10])

    di=bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    print("!!!!!!!!!!11",di.shape)
    cf_d_pan = di[cf_v_pan*2, cf_u_pan*2]* 1.000
    print("222222222222222",cf_d_pan)

    cf_d_cai = di[cf_v_cai, cf_u_cai]* 1.000
    cf_d_pot = di[cf_v_pot, cf_u_net]* 1.000
    cf_d_net = di[cf_v_net, cf_u_net]* 1.000

    global minv
    global maxv
    global hs_pan
    global hs_cai
    global hs_pot
    global hs_net

    global cnt
    global z_history_pan
    global z_history_cai
    global z_history_pot
    global z_history_net

    if(cf_d_pan < minv):
        minv= cf_d_pan
    if(cf_d_pan>maxv):
        maxv = cf_d_pan
    if(hs_pan < 1):
        hs_pan=cf_d_pan
    hs_pan = hs_pan + cf_d_pan

    if(cf_d_cai < minv):
        minv= cf_d_cai
    if(cf_d_cai>maxv):
        maxv = cf_d_cai
    if(hs_cai < 1):
        hs_cai=cf_d_cai
    hs_cai = hs_cai + cf_d_cai

    if(cf_d_pot < minv):
        minv= cf_d_pot
    if(cf_d_pot>maxv):
        maxv = cf_d_pot
    if(hs_pot < 1):
        hs_pot=cf_d_pot
    hs_pot = hs_pot + cf_d_pot

    if(cf_d_net < minv):
        minv= cf_d_net
    if(cf_d_net>maxv):
        maxv = cf_d_net
    if(hs_net < 1):
        hs_net=cf_d_net
    hs_net = hs_net + cf_d_net

    cnt = cnt +1

    z_history_pan.append(cf_d_pan)
    z_history_cai.append(cf_d_cai)
    z_history_pot.append(cf_d_pot)
    z_history_net.append(cf_d_net)

    most_pan = Counter(z_history_pan).most_common(10)
    most_cai = Counter(z_history_cai).most_common(10)
    most_pot = Counter(z_history_pot).most_common(10)
    most_net = Counter(z_history_net).most_common(10)

    (most_v_pan,count) = most_pan[0]
    (most_v_cai,count) = most_cai[0]
    (most_v_pot,count) = most_pot[0]
    (most_v_net,count) = most_net[0]


    if(cf_d_pan< 400):
        print("aaaa")
    send_barlist[0],send_barlist[1],send_barlist[2],send_barlist[3] = bar_list[0],bar_list[1],bar_list[2],most_v_pan
    send_barlist[4],send_barlist[5],send_barlist[6],send_barlist[7] = bar_list[3],bar_list[4],bar_list[5],most_v_cai
    send_barlist[8],send_barlist[9],send_barlist[10],send_barlist[11] = bar_list[6],bar_list[7],bar_list[8],most_v_pot
    send_barlist[12],send_barlist[13],send_barlist[14],send_barlist[15] = bar_list[9],bar_list[10],bar_list[11],most_v_net

    temp_send_barlist = Float32MultiArray(data = send_barlist)
    temp_send_barlist_pub = rospy.Publisher("process_uvd/bar_pos",Float32MultiArray,queue_size=10)
    temp_send_barlist_pub.publish(temp_send_barlist)
    #print("!!!!!!!!!!!!!!!!!!11",temp_send_barlist)
    if cnt>30000:
        z_history_pan=[]
        z_history_cai=[]
        z_history_pot=[]
        z_history_net=[]


    #mn = sum /16 
    #rospy.loginfo("Depth: x at %d  y at %d  z at %f, 4x4 mean: %f, history:%f, min:%f,  max:%f", int(cf_u), int(cf_v), cf_d  ,mn, hs /cnt,  minv,  maxv)
    return 

if __name__=='__main__':
    rospy.init_node("fs_answer")
    rospy.Subscriber("/kinect2/hd/image_depth_rect",Image,callback_dep_hd)
    rospy.Subscriber("/process_uv/bar_pos",Float32MultiArray,callback_bar_calibration,queue_size=10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down") 
    cv2.destroyAllWindows()