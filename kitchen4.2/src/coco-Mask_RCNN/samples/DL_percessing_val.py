# -*- coding:UTF-8 -*-
#!/usr/bin/env python
import rospy
import roslib
import sys
import os
#from后边是自己的包.msg，也就是自己包的msg文件夹下，test是我的msg文件名test.msg
from kitchen.msg import element_info
from kitchen.msg import obj
from kitchen.msg import objs
from kitchen.msg import broclist
from kitchen.msg import sglobj
from kitchen.msg import listobj
#print("element_info",element_info())
import cv2
import numpy as np
import detectionbar
import broccolidetection
from std_msgs.msg import String, Float32MultiArray

from PIL import Image
from cv_bridge import CvBridge
import math

abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../../lib/comm")
sys.path.append(abs_file + "/../../../lib/log")

from rlog import rlog
log = rlog()
log.setModuleName("inchef_maskrcnn")
log.set_priority("info")

list_obj_pub = None
temp_det_bar_pos_pub = None
list_broc_pub = None
#print("sglobj",sglobj())

def prcess_contours(cv_image, classname, masks_img, mask_id, x, y, width, height, scores):
    list_info=[]
    #print("max : ",i,classname)
    binery,contours, hierarchy = cv2.findContours(masks_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in range(len(contours)):
        #Area = cv2.contourArea(contours[c])
        rect = cv2.minAreaRect(contours[c])
        box = cv2.boxPoints(rect)#得到端点
        box = np.int0(box)#向下取整
        #cv2.drawContours(cv_image,[box],0,(0,255,0),1)

        cv2.rectangle(cv_image,(x, y),(x+height, y+width),(255,0,0),2)
        #print("***********",[x, y,x+height, y+width])
        #cx1, cy1 = rect[0] #min area center
        cx1 = int (x + height/2) # rect area center
        cy1 = int (y + width/2) # rect area center

        cv2.circle(cv_image, (np.int32(cx1), np.int32(cy1)), 6, (0,0,255), -1)
        cv2.drawContours(cv_image, contours, c, (0, 0, 255), 1, 8)

        score=scores if scores is not None else None
        caption='{}{:.2f}'.format(classname,scores) if scores else classname
        image=cv2.putText(cv_image,caption,(x, y),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)


        grasp_delta = 20 #pixcel
        grasp_left_x = int(cx1 - height/2 - grasp_delta)
        grasp_left_y = cy1 
        grasp_right_x = int(cx1 + height/2 + grasp_delta)
        grasp_right_y = cy1
        cv2.circle(cv_image, (np.int32(grasp_left_x), np.int32(grasp_left_y)), 6, (0,0,255), -1)
        cv2.circle(cv_image, (np.int32(grasp_right_x), np.int32(grasp_right_y)), 6, (0,0,255), -1)
        im = cv2.line(cv_image, (np.int32(grasp_left_x),np.int32(grasp_left_y)), 
            (np.int32(grasp_right_x), np.int32(grasp_right_y)), (0, 255, 255), 1)

        number = int(len(contours[c])/4)
        press1_x,press1_y = int(contours[c][0][0][0]), int(contours[c][0][0][1])
        press2_x,press2_y = int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
        press3_x,press3_y = int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
        press4_x,press4_y = int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])
        cv2.circle(cv_image, (press1_x,press1_y), 3, (0,0,0), -1)
        cv2.circle(cv_image, (press2_x,press2_y), 3, (0,0,0), -1)
        cv2.circle(cv_image, (press3_x,press3_y), 3, (0,0,0), -1)
        cv2.circle(cv_image, (press4_x,press4_y), 3, (0,0,0), -1)

        grasp_left_theta = int(180)
        grasp_right_theta = int(0)
        mask_id = int(mask_id)

        list_info = [int(mask_id), int(grasp_left_x),int(grasp_left_y),int(grasp_left_theta),
        int(grasp_right_x),int(grasp_right_y),int(grasp_right_theta),int(cx1),int(cy1),
        int(press1_x),int(press1_y),int(press2_x),int(press2_y),int(press3_x),int(press3_y),
        int(press4_x),int(press4_y),int(width),int(height)]
        #print("111111111:",list_info)

        return list_info


img_out1 = np.zeros((540,960),dtype=np.uint8)
img_out2 = np.zeros((540,960),dtype=np.uint8)
img_out3 = np.zeros((540,960),dtype=np.uint8)
det_bar_pos = np.zeros(12,dtype=float)
temp_single_obj=sglobj()

#postion_list = [left_plate1,right_plate1,up_plate1,down_plate1]
plate1 = [119,424,230,530] #(90,415) (190,525)
plate2 = [690,270,795,370] #(675,260) (775,340)
plate3 = [695,395,825,525] #(680,375) (795,475)
pan_roi = [340,260,480,408] #(680,375) (795,475)
beef_roi = [170,275,480,400]

bottle1 = [60,70, 150,150]
bottle2 = [170,70,240,150]


def callback(data):

    list_obj = listobj()
    list_obj.single_obj_info = []
    brocili_list = broclist()
    brocili_list.broc_uv_list = []
    temp_list_broc = []
    global temp_single_obj
    gettime = data.header
    #print("22222222222",data.header)
    #print(data.objects_vector[0].id)
   # print(data.objects_vector[0].classname)
   # print(data.objects_vector[0].score)
    #print("---"*20)

    single_obj = sglobj()
    #print("single_obj",single_obj)
    single_obj.element_data_temp = []

    single_obj.classname = []
    single_obj.score = []

    get_msg = data.objects_vector[0]
    bridge=CvBridge()
    cv_image =bridge.imgmsg_to_cv2(data.rgb_img,"bgr8")
    cv_image_hd =bridge.imgmsg_to_cv2(data.rgb_img_hd,"bgr8")
    #cv_depth_image = bridge.imgmsg_to_cv2(data.depth_img,"passthrough")
    rgb_img_to_pos = cv_image.copy()
    #depth_img_to_pos = cv_depth_image.copy()

    plate_score=[0,0]
    pan_score=[0]
    vegetablebowl_score=[0]
    broccoli_score=[0,0,0]
    souppothandle_score=[0]
    panhandle_score=[0]
    beef_score=[0]
    nethandle_score=[0]
    seasoningbottle_score=[0,0]
    seasoningbowl_score=[0]

    for i in range(len(get_msg.id)):
        if(get_msg.classname[i] == "pan"):
            pan_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "beef"):
            beef_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "plate"):
            plate_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "vegetablebowl"):
            vegetablebowl_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "broccoli"):
            broccoli_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "souppothandle"):
            souppothandle_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "panhandle"):
            panhandle_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "seasoningbowl"):
            beef_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "nethandle"):
            nethandle_score.append(get_msg.score[i])
        if(get_msg.classname[i] == "seasoningbottle"):
            seasoningbottle_score.append(get_msg.score[i])

    plate_score = sorted(plate_score,reverse=True)
    pan_score = sorted(pan_score,reverse=True)
    vegetablebowl_score = sorted(vegetablebowl_score,reverse=True)
    broccoli_score = sorted(broccoli_score,reverse=True)
    souppothandle_score = sorted(souppothandle_score,reverse=True)
    panhandle_score = sorted(panhandle_score,reverse=True)
    beef_score = sorted(beef_score,reverse=True)
    nethandle_score = sorted(nethandle_score,reverse=True)
    seasoningbottle_score = sorted(seasoningbottle_score,reverse=True)
    seasoningbowl_score = sorted(seasoningbowl_score,reverse=True)
    #cv2.imshow("niubi",cv_image_hd)
    det_bar_pos = detectionbar.model_detection(cv_image_hd,cv_image)

    bro_list = broccolidetection.broccoli_detection(cv_image)
    global temp_list_broc

    if len(bro_list)>0:
        brocili_list.broc_uv_list=[]
        brocili_list.broc_uv_list = bro_list
        temp_list_broc = bro_list
    if len(bro_list)<=0:
        brocili_list.broc_uv_list = temp_list_broc
        #print("wo cao :",temp_list_broc)
    #print("xi lan hua: ",brocili_list.broc_uv_list)
    
    '''
    print("plate score :           ",plate_score)
    print("pan score :             ",pan_score)
    print("vegetablebowl score :   ",vegetablebowl_score)
    print("broccoli score :        ",broccoli_score)
    print("souppothandle score :   ",souppothandle_score)
    print("panhandle score :       ",panhandle_score)
    print("beef score :            ",beef_score)
    print("nethandle score :       ",nethandle_score)
    print("seasoningbottle score : ",seasoningbottle_score)
    print("seasoningbowl score :  ",seasoningbowl_score)
    '''

    single_obj.element_data_temp = []
    single_obj.classname = []
    single_obj.score = []
    element_data = element_info()
    element_data.id_info = []
    for i in range(len(get_msg.id)):
        element_data = element_info()
        element_data.id_info = []
        
        mask_area = get_msg.roi[i].height * get_msg.roi[i].width
        if mask_area > 0 and get_msg.score[i] > 0.19:

            mask_image =bridge.imgmsg_to_cv2(get_msg.masks[i],"passthrough")

            if get_msg.classname[i] == "plate":
                center_x = int (get_msg.roi[i].x_offset + get_msg.roi[i].height/2) # rect area center
                center_y = int (get_msg.roi[i].y_offset + get_msg.roi[i].width/2) # rect area center
                if center_x>plate1[0]  and center_x<plate1[2]   and center_y>plate1[1] and center_y<plate1[3]:
                    temp_list = []
                    temp_list = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                    get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])
                    log.info("Got plate1 center:get_time: {},center_pos: {}".format(gettime,temp_list[7:9]))

                    if temp_list!=None:
                        #element_data.id_info = []
                        element_data.id_info = temp_list
                        single_obj.element_data_temp.append(element_data)
                        #print("aaa:",single_obj.element_data_temp)
                        single_obj.score.append(get_msg.score[i])
                        single_obj.classname.append("plate1")
            
            if get_msg.classname[i] == "plate":
                center_x = int (get_msg.roi[i].x_offset + get_msg.roi[i].height/2) # rect area center
                center_y = int (get_msg.roi[i].y_offset + get_msg.roi[i].width/2) # rect area center
                if center_x>plate2[0]  and center_x<plate2[2]   and center_y>plate2[1] and center_y<plate2[3]:
                    temp_list1 = []
                    temp_list1 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                    get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])
                    log.info("Got plate2 center:get_time: {},center_pos_u_v: {}".format(gettime,temp_list1[7:9]))
                    if temp_list1!=None:
                        #element_data.id_info = []
                        element_data.id_info = temp_list1
                        single_obj.element_data_temp.append(element_data)
                        #single_obj.classname.append(get_msg.classname[i])
                        single_obj.score.append(get_msg.score[i])
                        single_obj.classname.append("plate2")
            
            if get_msg.classname[i] == "plate":
                center_x = int (get_msg.roi[i].x_offset + get_msg.roi[i].height/2) # rect area center
                center_y = int (get_msg.roi[i].y_offset + get_msg.roi[i].width/2) # rect area center
                if center_x>plate3[0]  and center_x<plate3[2]   and center_y>plate3[1] and center_y<plate3[3]:
                    temp_list2 = []
                    temp_list2 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                    get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])
                    log.info("Got plate3 center:get_time: {},center_pos_u_v: {}".format(gettime,temp_list2[7:9]))
                    if temp_list2!=None:                   
                        element_data.id_info = temp_list2
                        #print("wo-2-cia",temp_list)
                        single_obj.element_data_temp.append(element_data)
                        #single_obj.classname.append(get_msg.classname[i])
                        single_obj.score.append(get_msg.score[i])
                        single_obj.classname.append("plate3")
            
            if get_msg.classname[i] == "beef" and get_msg.score[i] == beef_score[0]:
                center_x = int (get_msg.roi[i].x_offset + get_msg.roi[i].height/2) # rect area center
                center_y = int (get_msg.roi[i].y_offset + get_msg.roi[i].width/2) # rect area center
                if center_x>beef_roi[0]  and center_x<beef_roi[2]   and center_y>beef_roi[1] and center_y<beef_roi[3]:
                    temp_list3 = []
                    temp_list3 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i], 
                    get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])
                    log.info("Got beef center:get_time: {},center_pos_u_v: {},beef_roi: {}".format(gettime,temp_list3[7:9],temp_list3[17:19]))
                    if temp_list3!=None:
                        
                        element_data.id_info = temp_list3
                        single_obj.element_data_temp.append(element_data)
                        single_obj.classname.append(get_msg.classname[i])
                        single_obj.score.append(get_msg.score[i])

            if get_msg.classname[i] == "pan":
                center_x = int (get_msg.roi[i].x_offset + get_msg.roi[i].height/2) # rect area center
                center_y = int (get_msg.roi[i].y_offset + get_msg.roi[i].width/2) # rect area center
                if center_x>pan_roi[0]  and center_x<pan_roi[2]   and center_y>pan_roi[1] and center_y<pan_roi[3]:
                    temp_list4 = []
                    temp_list4 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                    get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])
                    log.info("Got pan center: get_time: {},center_pos_u_v: {}".format(gettime,temp_list4[7:9]))
                    if temp_list4!=None:
                        
                        element_data.id_info = temp_list4
                        single_obj.element_data_temp.append(element_data)
                        single_obj.classname.append(get_msg.classname[i])
                        single_obj.score.append(get_msg.score[i])
                
            '''
            if get_msg.classname[i] == "broccoli" and get_msg.score[i] == broccoli_score[0]:
                temp_list5 =[]
                temp_list = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])

                if temp_list5!=None:

                    element_data.id_info = temp_list5
                    single_obj.element_data_temp.append(element_data)
                    single_obj.classname.append(get_msg.classname[i])
                    single_obj.score.append(get_msg.score[i])

            if get_msg.classname[i] == "broccoli" and get_msg.score[i] == broccoli_score[1]:
                temp_list6 = []
                temp_list6 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])

                if temp_list6!=None:
                    
                    element_data.id_info = temp_list6
                    single_obj.element_data_temp.append(element_data)
                    single_obj.classname.append(get_msg.classname[i])
                    single_obj.score.append(get_msg.score[i])

            if get_msg.classname[i] == "broccoli" and get_msg.score[i] == broccoli_score[2]:
                temp_list7 = []
                temp_list7 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])

                if temp_list7!=None:
                    
                    element_data.id_info = temp_list7
                    single_obj.element_data_temp.append(element_data)
                    single_obj.classname.append(get_msg.classname[i])
                    single_obj.score.append(get_msg.score[i])

            if get_msg.classname[i] == "souppothandle" and get_msg.score[i] == souppothandle_score[0]:
                temp_list8 = []
                temp_list8 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])

                if temp_list8!=None:
                    
                    element_data.id_info = temp_list8
                    single_obj.element_data_temp.append(element_data)
                    single_obj.classname.append(get_msg.classname[i])
                    single_obj.score.append(get_msg.score[i])
            '''
            '''
            if get_msg.classname[i] == "nethandle" and get_msg.score[i] == nethandle_score[0]:
                temp_list9 = []
                temp_list9 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])

                if temp_list9!=None:
                    
                    element_data.id_info = temp_list9
                    single_obj.element_data_temp.append(element_data)
                    single_obj.classname.append(get_msg.classname[i])
                    single_obj.score.append(get_msg.score[i])

            if get_msg.classname[i] == "vegetablebowl" and get_msg.score[i] == vegetablebowl_score[0]:
                temp_list10 = []
                temp_list10 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])

                if temp_list10!=None:
                    
                    element_data.id_info = temp_list10
                    single_obj.element_data_temp.append(element_data)
                    single_obj.classname.append(get_msg.classname[i])
                    single_obj.score.append(get_msg.score[i])
            '''
            
            if get_msg.classname[i] == "seasoningbowl" and get_msg.score[i] == seasoningbowl_score[0]:
                temp_list11 = []
                temp_list11 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])

                if temp_list11!=None:
                    
                    element_data.id_info = temp_list11
                    single_obj.element_data_temp.append(element_data)
                    #single_obj.classname.append(get_msg.classname[i])
                    single_obj.score.append(get_msg.score[i])
            
            
            if get_msg.classname[i] == "seasoningbottle":
                center_x = int (get_msg.roi[i].x_offset + get_msg.roi[i].height/2) # rect area center
                center_y = int (get_msg.roi[i].y_offset + get_msg.roi[i].width/2) # rect area center
                if center_x>bottle1[0]  and center_x<bottle1[2]   and center_y>bottle1[1] and center_y<bottle1[3]:
                    temp_list12 = []
                    temp_list12 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                    get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])
                    

                    if temp_list12!=None:
                    
                        single_obj.classname.append("seasoningbottle1")
                        element_data.id_info = temp_list12
                        single_obj.element_data_temp.append(element_data)
                        #single_obj.classname.append(get_msg.classname[i])
                        single_obj.score.append(get_msg.score[i])

            if get_msg.classname[i] == "seasoningbottle":
                center_x = int (get_msg.roi[i].x_offset + get_msg.roi[i].height/2) # rect area center
                center_y = int (get_msg.roi[i].y_offset + get_msg.roi[i].width/2) # rect area center
                if center_x>bottle2[0]  and center_x<bottle2[2]   and center_y>bottle2[1] and center_y<bottle2[3]:
                    temp_list13 = []
                    temp_list13 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                    get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])
                    

                    if temp_list13!=None:

                        single_obj.classname.append("seasoningbottle2")
                        element_data.id_info = temp_list13
                        single_obj.element_data_temp.append(element_data)
                        #single_obj.classname.append(get_msg.classname[i])
                        single_obj.score.append(get_msg.score[i])
                        '''
            if get_msg.classname[i] == "panhandle" and get_msg.score[i] == panhandle_score[0]:
                temp_list14 = []
                temp_list14 = prcess_contours(cv_image, get_msg.classname[i], mask_image, get_msg.id[i],
                get_msg.roi[i].x_offset, get_msg.roi[i].y_offset, get_msg.roi[i].height, get_msg.roi[i].width, get_msg.score[i])

                if temp_list14!=None:
                    
                    element_data.id_info = temp_list14
                    single_obj.element_data_temp.append(element_data)
                    single_obj.classname.append(get_msg.classname[i])
                    single_obj.score.append(get_msg.score[i])
            '''
    #print("88888",single_obj)

    if len(single_obj.score)>0:
        #print("1111:",single_obj)
        temp_single_obj = single_obj
        #print("****",temp_single_obj)
        list_obj.single_obj_info.append(single_obj)
    else:
        
        list_obj.single_obj_info.append(temp_single_obj)

    #print("1111:",list_obj.single_obj_info)
    #list_obj.header.stamp = data.header.stamp
    #list_obj.header.frame_id = data.header.frame_id
    print("---"*20)
    #print("--ww--",list_obj.header)
    list_obj.rgb_img_to_pos = bridge.cv2_to_imgmsg(rgb_img_to_pos, encoding="bgr8")
    #list_obj.depth_img_to_pos = bridge.cv2_to_imgmsg(depth_img_to_pos,"passthrough")

    temp_det_bar_pos = Float32MultiArray(data = det_bar_pos)
    #print("1111:",list_obj)
   
    global list_obj_pub
    global temp_det_bar_pos_pub
    global list_broc_pub

    list_broc_pub.publish(brocili_list)
    list_obj_pub.publish(list_obj)
    temp_det_bar_pos_pub.publish(temp_det_bar_pos)
    log.info("Got broc center: get_time: {},center_pos_u_v: {}".format(gettime,brocili_list))
    log.info("Got bar pos: get_time: {},pos_info_u_v: {}".format(gettime,temp_det_bar_pos))
    
    cv2.rectangle(cv_image,(plate2[0], plate2[1]),(plate2[0]+(plate2[2]-plate2[0]), plate2[1]+(plate2[3]-plate2[1])),(0,0,255),1)
    cv2.rectangle(cv_image,(plate1[0], plate1[1]),(plate1[0]+(plate1[2]-plate1[0]), plate1[1]+(plate1[3]-plate1[1])),(0,0,255),1)
    cv2.rectangle(cv_image,(plate3[0], plate3[1]),(plate3[0]+(plate3[2]-plate3[0]), plate3[1]+(plate3[3]-plate3[1])),(0,0,255),1)
    cv2.rectangle(cv_image,(pan_roi[0], pan_roi[1]),(pan_roi[0]+(pan_roi[2]-pan_roi[0]), pan_roi[1]+(pan_roi[3]-pan_roi[1])),(0,0,255),1)
    cv2.rectangle(cv_image,(beef_roi[0], beef_roi[1]),(beef_roi[0]+(beef_roi[2]-beef_roi[0]), beef_roi[1]+(beef_roi[3]-beef_roi[1])),(0,0,255),1)
    #cv2.rectangle(cv_image,(bottle1[0], bottle1[1]),(bottle1[0]+(bottle1[2]-bottle1[0])/2, bottle1[1]+(bottle1[3]-bottle1[1])),(0,0,255),2)
    #cv2.rectangle(cv_image,(bottle2[0], bottle2[1]),(bottle2[0]+(bottle2[2]-bottle2[0])/2, bottle2[1]+(bottle2[3]-bottle2[1])),(0,0,255),2)
    cv2.imshow("img2",cv_image)
    cv2.waitKey(1)


def listener():
    global list_obj_pub
    global temp_det_bar_pos_pub
    global list_broc_pub
    rospy.init_node('listener', anonymous=False)
    list_obj_pub = rospy.Publisher("process_uv/listobjs",listobj,queue_size=1)
    temp_det_bar_pos_pub = rospy.Publisher("process_uv/bar_pos",Float32MultiArray,queue_size=1)
    list_broc_pub = rospy.Publisher("process_uv/listbroc",broclist,queue_size=1)
    rospy.Subscriber("processor/objs", objs, callback)

    rospy.spin()
 
if __name__ == '__main__':
    listener()
