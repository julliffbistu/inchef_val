#!/usr/bin/env python
# coding: utf-8
import rospy
import roslib
from std_msgs.msg import String, Float32MultiArray,Int8
#from sensor_msgs import msg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import math
import sys
import time
import math
##from inchef_calibration.msg import listobj
from kitchen.msg import listframes_calib
from kitchen.msg import bar_calib
from kitchen.msg import broc_calib
from kitchen.msg import sglframe_calib
from kitchen.msg import sglobject_calib
from kitchen.msg import listobj
from kitchen.msg import broclist
from kitchen.msg import rail_cmd_algo

import calibration_class
import yaml
import scipy.io as scio
import copy
abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../lib/comm")
sys.path.append(abs_file + "/../../lib/log")

from rlog import rlog
log = rlog()
log.setModuleName("inchef_calibration")
log.set_priority("info")




ROOT_DIR = os.path.abspath("./")
sys.path.append(ROOT_DIR)
print(ROOT_DIR)
times_record=1
params_calib=[]
#params_yaml = yaml.load_all(file('yaml_test.yaml', 'r'), Loader=yaml.FullLoader)
params_yaml = yaml.load_all(file('yaml_test_03.yaml', 'r'))

for y in params_yaml:
    params_calib.append(y)
print(params_calib)


###calibration class putin
IMG_COL=960
IMG_ROW=540
NAME = 'inchef_calibration_node'
BAR_FEATURE_NUMS=3
BROC_FEATURE_NUMS=2
PIXEL_LENGTH=0.0015
rail_time_secs=sys.maxsize
rail_time_nsecs=sys.maxsize
img_time_secs=0
img_time_nsecs=0
dataFile = ROOT_DIR+'/depth_data_03.mat'
print(dataFile)
depth_dict = scio.loadmat(dataFile)
still_depth_image=depth_dict['cv_image_depth_left']
cv_depth_image=np.zeros([540,960,1],dtype=np.uint16)
DEPTH_COMPARE_TH=300
global raillist
raillist=[0.0,0.0,0.0]
def listener():
    rospy.init_node(NAME)
    #print("~~~~~~~~~~~~~~")
    rospy.Subscriber("/process_uv/listobjs",listobj,callback_masrcnn_calibration,queue_size=10)
    rospy.Subscriber("/process_uv/bar_pos",Float32MultiArray,callback_bar_calibration,queue_size=10)
    rospy.Subscriber("/process_uv/listbroc",broclist,callback_broc_calibration,queue_size=10)
    rospy.Subscriber('/rail/position_cmd', rail_cmd_algo, callback_rail,queue_size=10)
    rospy.spin()
def spinOnce():
    #print("::::::::::::rate")
    rate=10
    r = rospy.Rate(rate)
    r.sleep()

def findcalibparams(railpos,robotarm):
    ########
    Quat=np.zeros(4,dtype=float)
    Translation=np.zeros([3,1],dtype=float)

    rail_h_param=railpos[0]
    rail_l_param=railpos[1]
    rail_r_param=railpos[2]
    '''

    rail_h_param=0.45
    rail_l_param=0.2
    rail_r_param=1.35
    '''
    for i in range(len(params_calib)):
        dicttemp=params_calib[i]
        rail_h=dicttemp['rail']['rail_h']
        rail_l=dicttemp['rail']['rail_l']
        rail_r=dicttemp['rail']['rail_r']
        arm=dicttemp['arm']
        #print("dicttemp['arm']",dicttemp['arm'])
        #print(dicttemp['rail'])
        if robotarm=='leftarm':
            if (abs(rail_h_param-rail_h)<0.01) and (abs(rail_l_param-rail_l)<0.01) and (arm=='left'):
               # print(dicttemp)
                Quat[0]=dicttemp['rotation']['x']
                Quat[1]=dicttemp['rotation']['y']
                Quat[2]=dicttemp['rotation']['z']
                Quat[3]=dicttemp['rotation']['w']
                Translation[0][0]=dicttemp['translation']['x']
                Translation[1][0]=dicttemp['translation']['y']
                Translation[2][0]=dicttemp['translation']['z']
        else:
            if (abs(rail_h_param-rail_h)<0.01) and (abs(rail_r_param-rail_r)<0.01) and (arm=='right'):
                #print(dicttemp)
                Quat[0]=dicttemp['rotation']['x']
                Quat[1]=dicttemp['rotation']['y']
                Quat[2]=dicttemp['rotation']['z']
                Quat[3]=dicttemp['rotation']['w']
                Translation[0][0]=dicttemp['translation']['x']
                Translation[1][0]=dicttemp['translation']['y']
                Translation[2][0]=dicttemp['translation']['z']

    return Quat,Translation

def find_camera_internal_params():
    camera_internal_params=np.zeros([3,3],dtype=float)
    dicttemp=params_calib[0]
    camera_internal_params[0][0]=dicttemp['Kmatrix']['k00']
    camera_internal_params[0][2]=dicttemp['Kmatrix']['k02']
    camera_internal_params[1][1]=dicttemp['Kmatrix']['k11']
    camera_internal_params[1][2]=dicttemp['Kmatrix']['k12']
    camera_internal_params[2][2]=dicttemp['Kmatrix']['k22']
    return camera_internal_params

def callback_rail(data):
    global raillist
    raillist=data.rail_pos
    global rail_time_secs
    global rail_time_nsecs
    rail_time_secs = data.header.stamp.secs
    rail_time_nsecs = data.header.stamp.nsecs
    #print("~~~~~~~~~~",now.secs)
    print(raillist)
    log.info("Got rail: rail_time_secs: {},rail_time_nsecs: {},rail_pos: {}".format(rail_time_secs,rail_time_nsecs,raillist))

def rail_img_time_cpm(mode):
    statu_temp=True
    if mode==True:
        if (img_time_secs<rail_time_secs):
            statu_temp=False
        elif(img_time_secs==rail_time_secs):
            if(img_time_nsecs<rail_time_nsecs):
                statu_temp=False
    return statu_temp





def callback_masrcnn_calibration(data):
    #publish
    print("raillist",raillist)

    global img_time_secs
    global img_time_nsecs
    img_time_secs=data.header.stamp.secs
    img_time_nsecs=data.header.stamp.nsecs
    log.info("Got img_time_secs: {},img_time_nsecs: {}".format(img_time_secs,img_time_nsecs))
    raillist_temp=copy.copy(raillist)
    calibra_instance=calibration_class.point_transformation()
    multi_frame=listframes_calib()
    multi_frame.frames_info=[]
    single_frame = sglframe_calib()
    single_frame.feature_info_left=[]
    single_frame.feature_info_right=[]
    single_frame.classname = []
    single_frame.score = []
    get_msg = data.single_obj_info[0]
    #print("get_msg",get_msg)
    bridge=CvBridge()
    global cv_depth_image
    cv_depth_image = bridge.imgmsg_to_cv2(data.depth_img_to_pos,"passthrough")
    cv_image =bridge.imgmsg_to_cv2(data.rgb_img_to_pos,"bgr8")
    rgb_img_to_pos = cv_image.copy()
    depth_img_to_pos = cv_depth_image.copy()
    #find rail calib params
    Quat=np.zeros(4,dtype=float)
    Translation=np.zeros([3,1],dtype=float)
    Quat,Translation=findcalibparams(raillist_temp,"leftarm")
    log.info("leftarm maskrcnn calibration params: Quat: {},Translation: {}".format(Quat,Translation))
    camera_internal_params=np.zeros([3,3],dtype=float)
    camera_internal_params=find_camera_internal_params()
    log.info("Got maskrcnn camera_internal_params: {}".format(camera_internal_params))
    calibra_instance.load_camera_internal_params(camera_internal_params)

    calibra_instance.load_position(Quat,Translation)
    single_frame_calibration_fun_temp(get_msg,depth_img_to_pos,single_frame.feature_info_left,calibra_instance)
    Quat,Translation=findcalibparams(raillist_temp,"rightarm")
    log.info("rightarm maskrcnn calibration params: Quat: {},Translation: {}".format(Quat,Translation))

    calibra_instance.load_position(Quat,Translation)
    single_frame_calibration_fun_temp(get_msg,depth_img_to_pos,single_frame.feature_info_right,calibra_instance)
##################


    single_frame.classname=get_msg.classname
    single_frame.score=get_msg.score

    score_list=list(get_msg.score)
    situation=rail_img_time_cpm(True)
    #print("score_list,situation",score_list,situation)
    for i in range(len(score_list)):
        score_list[i]=score_list[i]*situation
    single_frame.score=score_list




    multi_frame.frames_info.append(single_frame)

    multi_frame.header.stamp = data.header.stamp
    multi_frame.header.frame_id = data.header.frame_id
    ####print("------------------------")
    multi_frame.rail_mask=raillist_temp
    multi_frame.rgb_img_to_pos = bridge.cv2_to_imgmsg(cv_image.copy(), encoding="bgr8")
    multi_frame.depth_img_to_pos = bridge.cv2_to_imgmsg(cv_depth_image.copy(),"passthrough")


    global times_record
    times_record=times_record+1
    print(times_record)
    multi_frame_pub = rospy.Publisher("calibration/mask_multi_frame",listframes_calib,queue_size=10)
    multi_frame_pub.publish(multi_frame)
def callback_bar_calibration(data):
    raillist_temp=copy.copy(raillist)
    print("raillist_temp",raillist_temp)
    calibra_instance=calibration_class.point_transformation()
    bar_list=data.data

    bar_calib_obj=bar_calib()
    depth_img_to_pos = cv_depth_image.copy()
    Quat=np.zeros(4,dtype=float)
    Translation=np.zeros([3,1],dtype=float)
    Quat,Translation=findcalibparams(raillist_temp,"leftarm")
    log.info("leftarm bar calibration params: Quat: {},Translation: {}".format(Quat,Translation))

    calibra_instance.load_position(Quat,Translation)
    camera_internal_params=np.zeros([3,3],dtype=float)
    camera_internal_params=find_camera_internal_params()
    log.info("Got bar camera_internal_params: {}".format(camera_internal_params))

    calibra_instance.load_camera_internal_params(camera_internal_params)
    #print("~~~~~~~11~~~~~~~",bar_list)
    single_frame_calib_bar(depth_img_to_pos,bar_list,bar_calib_obj.left_arm,calibra_instance)
    Quat,Translation=findcalibparams(raillist_temp,"rightarm")
    log.info("rightarm bar calibration params: Quat: {},Translation: {}".format(Quat,Translation))

    calibra_instance.load_position(Quat,Translation)
    single_frame_calib_bar(depth_img_to_pos,bar_list,bar_calib_obj.right_arm,calibra_instance)
    bar_calib_obj.rail_bar=raillist_temp
    score_bar_list=[]
    situation=rail_img_time_cpm(True)
    for i in range(int(len(bar_list)/BAR_FEATURE_NUMS)):
        score_bar_list.append(situation*1.0)
    bar_calib_obj.score_bar=score_bar_list



    #print("bar_calib_obj",bar_calib_obj.right_arm)
    calibration_bar_pub = rospy.Publisher("calibration/bar",bar_calib,queue_size=1)
    calibration_bar_pub.publish(bar_calib_obj)
def callback_broc_calibration(data):
    raillist_temp=copy.copy(raillist)

    calibra_instance=calibration_class.point_transformation()
    broc_uv_list=data.broc_uv_list
    broc_calib_obj=broc_calib()

    depth_img_to_pos = cv_depth_image.copy()
    Quat=np.zeros(4,dtype=float)
    Translation=np.zeros([3,1],dtype=float)
    Quat,Translation=findcalibparams(raillist_temp,"leftarm")
    log.info("leftarm broc calibration params: Quat: {},Translation: {}".format(Quat,Translation))

    calibra_instance.load_position(Quat,Translation)
    camera_internal_params=np.zeros([3,3],dtype=float)
    camera_internal_params=find_camera_internal_params()
    log.info("Got broc camera_internal_params: {}".format(camera_internal_params))


    calibra_instance.load_camera_internal_params(camera_internal_params)
    #print("leftarm",Translation)
    single_frame_calib_broc(depth_img_to_pos,broc_uv_list,broc_calib_obj.left_arm,calibra_instance)
    Quat,Translation=findcalibparams(raillist_temp,"rightarm")
    calibra_instance.load_position(Quat,Translation)
    log.info("rightarm broc calibration params: Quat: {},Translation: {}".format(Quat,Translation))

   # print("rightarm",Translation)
    single_frame_calib_broc(depth_img_to_pos,broc_uv_list,broc_calib_obj.right_arm,calibra_instance)
    #print("left",broc_calib_obj.left_arm)
    #print("right",broc_calib_obj.right_arm)
    broc_calib_obj.rail_broc=raillist_temp
    score_broc_list=[]
    situation=rail_img_time_cpm(True)
    for i in range(int(len(broc_uv_list)/BROC_FEATURE_NUMS)):
        score_broc_list.append(situation*1.0)
    broc_calib_obj.score_broc=score_broc_list

    calibration_broc_pub = rospy.Publisher("calibration/broc",broc_calib,queue_size=1)
    calibration_broc_pub.publish(broc_calib_obj)
def single_frame_calib_bar(depth_img_to_pos,bar_list,arm,calibra_instance):
    depth_img_copy=depth_img_to_pos.copy()
    for i in range(int(len(bar_list)/BAR_FEATURE_NUMS)):
        #i=0
        temp_u=bar_list[i*BAR_FEATURE_NUMS+0]
        temp_v=bar_list[i*BAR_FEATURE_NUMS+1]
        temp_theta=bar_list[i*BAR_FEATURE_NUMS+2]
        temp_d_fun=get_point_depth_min(depth_img_copy,temp_u,temp_v)
        print("~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",temp_u,temp_v,temp_d_fun)
        temp_x,temp_y,temp_z=calibra_instance.Pix2baselink_points(temp_u,temp_v,temp_d_fun)
        arm.append(float(temp_x))
        arm.append(float(temp_y))
        arm.append(float(temp_z))
        arm.append(0)
        arm.append(0)
        #arm.append(temp_theta/math.pi*180)
        arm.append(temp_theta)



def single_frame_calib_broc(depth_img_to_pos,broc_uv_list,arm,calibra_instance):
    depth_img_copy=depth_img_to_pos.copy()

    for i in range(int(len(broc_uv_list)/BROC_FEATURE_NUMS)):
        temp_u=broc_uv_list[i*BROC_FEATURE_NUMS+0]
        temp_v=broc_uv_list[i*BROC_FEATURE_NUMS+1]
        temp_theta=0
        temp_d_fun=get_point_depth_min(depth_img_copy,temp_u,temp_v)
        temp_x,temp_y,temp_z=calibra_instance.Pix2baselink_points(temp_u,temp_v,temp_d_fun)
        arm.append(float(temp_x))
        arm.append(float(temp_y))
        arm.append(float(temp_z))

        arm.append(0)
        arm.append(0)
       # arm.append(temp_theta/math.pi*180)
        arm.append(temp_theta)
def single_frame_calibration_fun(get_msg,cv_depth_image):



    sglobject_obj=sglobject_calib()
    feature_info_list=[]
    classname_list=[]
    score_list=[]
    subobject_info=get_msg.id_info

    sglobject_obj.features=[]

    #print("subobject_info",len(subobject_info)/len(get_msg.classname))
    margin=len(subobject_info)/len(get_msg.classname)
    for i in range(len(get_msg.classname)):
        bigin_pos=i*margin
        mask_id=subobject_info[0+bigin_pos]
        ####grasp_left
        grasp_left_u,grasp_left_v,grasp_left_theta=subobject_info[1+bigin_pos:4+bigin_pos]
        cv_depth_image_temp1=cv_depth_image.copy()
        grasp_left_d_fun=get_point_depth_min(cv_depth_image_temp1,grasp_left_u,grasp_left_v)
        #########################
        ################################
        grasp_left_x,grasp_left_y,grasp_left_z=calibra_instance.Pix2baselink_points(grasp_left_u,grasp_left_v,grasp_left_d_fun)
        ####grasp_right
        grasp_right_u,grasp_right_v,grasp_right_theta=subobject_info[4+bigin_pos:7+bigin_pos]
        #print()
       # grasp_right_d=cv_depth_image[grasp_right_v,grasp_right_u]
        grasp_right_d_fun=get_point_depth_min(cv_depth_image_temp1,grasp_right_u,grasp_right_v)
        grasp_right_x,grasp_right_y,grasp_right_z=calibra_instance.Pix2baselink_points(grasp_right_u,
        grasp_right_v,grasp_right_d_fun)
        ##center
        center_u,center_v=subobject_info[7+bigin_pos:9+bigin_pos]
        #center_d=cv_depth_image[center_v,center_u]
        center_d_fun=get_point_depth_min(cv_depth_image_temp1,center_u,center_v)
        center_x,center_y,center_z=calibra_instance.Pix2baselink_points(center_u,center_v,center_d_fun)
        #press
        press1_u,press1_v=subobject_info[9+bigin_pos:11+bigin_pos]
       # press1_d=cv_depth_image[press1_v,press1_u]
        press1_d_fun=get_point_depth_min(cv_depth_image_temp1,press1_u,press1_v)
        press1_x,press1_y,press1_z=calibra_instance.Pix2baselink_points(press1_u,press1_v,press1_d_fun)
        press2_u,press2_v=subobject_info[11+bigin_pos:13+bigin_pos]
        #press2_d=cv_depth_image[press2_v,press2_u]
        press2_d_fun=get_point_depth_min(cv_depth_image_temp1,press2_u,press2_v)
        press2_x,press2_y,press2_z=calibra_instance.Pix2baselink_points(press2_u,press2_v,press2_d_fun)
        press3_u,press3_v=subobject_info[13+bigin_pos:15+bigin_pos]
        #press3_d=cv_depth_image[press3_v,press3_u]
        press3_d_fun=get_point_depth_min(cv_depth_image_temp1,press3_u,press3_v)
        press3_x,press3_y,press3_z=calibra_instance.Pix2baselink_points(press3_u,press3_v,press3_d_fun)
        press4_u,press4_v=subobject_info[15+bigin_pos:17+bigin_pos]
        #press4_d=cv_depth_image[press4_v,press4_u]
        press4_d_fun=get_point_depth_min(cv_depth_image_temp1,press4_u,press4_v)
        press4_x,press4_y,press4_z=calibra_instance.Pix2baselink_points(press4_u,press4_v,press4_d_fun)
        #beef lenght and width

        subobject_info_cal=[float(grasp_left_x),float(grasp_left_y),float(grasp_left_z),
        float(grasp_right_x),float(grasp_right_y),float(grasp_right_z),
        float(center_x),float(center_y),float(center_z),
        float(press1_x),float(press1_y),float(press1_z),
        float(press2_x),float(press2_y),float(press2_z),
        float(press3_x),float(press3_y),float(press3_z),
        float(press4_x),float(press4_y),float(press4_z)]

        sglobject_obj.features=subobject_info_cal
        feature_info_list.append(sglobject_obj.features)
        classname_list.append(get_msg.classname[i])
        score_list.append(get_msg.score[i])
    return feature_info_list,classname_list,score_list

def single_frame_calibration_fun_temp(get_msg,cv_depth_image,single_frame_features,calibra_instance):




    classname_list=[]
    score_list=[]
    cv_depth_image_temp1=cv_depth_image.copy()


    for i in range(len(get_msg.classname)):
        sglobject_obj=sglobject_calib()
        sglobject_obj.features=[]
        subobject_info=get_msg.element_data_temp[i].id_info
        mask_id=subobject_info[0]
        ####grasp_left
        grasp_left_u,grasp_left_v,grasp_left_theta=subobject_info[1:4]
        grasp_left_d_fun=get_point_depth_min(cv_depth_image_temp1,grasp_left_u,grasp_left_v)
        grasp_left_x,grasp_left_y,grasp_left_z=calibra_instance.Pix2baselink_points(grasp_left_u,grasp_left_v,grasp_left_d_fun)
        ####grasp_right
        grasp_right_u,grasp_right_v,grasp_right_theta=subobject_info[4:7]
        grasp_right_d_fun=get_point_depth_min(cv_depth_image_temp1,grasp_right_u,grasp_right_v)
        grasp_right_x,grasp_right_y,grasp_right_z=calibra_instance.Pix2baselink_points(grasp_right_u,grasp_right_v,grasp_right_d_fun)
        ##center
        center_u,center_v=subobject_info[7:9]
        center_d_fun=get_point_depth_min(cv_depth_image_temp1,center_u,center_v)
       # print("beef",center_u,center_v,center_d_fun)
        center_x,center_y,center_z=calibra_instance.Pix2baselink_points(center_u,center_v,center_d_fun)
       # print("beef_pos",center_x,center_y,center_z)
        #press1
        press1_u,press1_v=subobject_info[9:11]
        press1_d_fun=get_point_depth_min(cv_depth_image_temp1,press1_u,press1_v)
        press1_x,press1_y,press1_z=calibra_instance.Pix2baselink_points(press1_u,press1_v,press1_d_fun)
        #press2
        press2_u,press2_v=subobject_info[11:13]
        press2_d_fun=get_point_depth_min(cv_depth_image_temp1,press2_u,press2_v)
        press2_x,press2_y,press2_z=calibra_instance.Pix2baselink_points(press2_u,press2_v,press2_d_fun)
        #press3
        press3_u,press3_v=subobject_info[13:15]
        press3_d_fun=get_point_depth_min(cv_depth_image_temp1,press3_u,press3_v)
        press3_x,press3_y,press3_z=calibra_instance.Pix2baselink_points(press3_u,press3_v,press3_d_fun)
        #press4
        press4_u,press4_v=subobject_info[15:17]
        press4_d_fun=get_point_depth_min(cv_depth_image_temp1,press4_u,press4_v)
        press4_x,press4_y,press4_z=calibra_instance.Pix2baselink_points(press4_u,press4_v,press4_d_fun)
       # PIXEL_LENGTH
        #beef lenght and width
        lenght,witdh=subobject_info[17:19]

        subobject_info_cal=[float(grasp_left_x),float(grasp_left_y),float(grasp_left_z),grasp_left_theta,0,0,
        float(grasp_right_x),float(grasp_right_y),float(grasp_right_z),grasp_right_theta,0,0,
        float(center_x),float(center_y),float(center_z),0,0,0,
        float(press1_x),float(press1_y),float(press1_z),0,0,0,
        float(press2_x),float(press2_y),float(press2_z),0,0,0,
        float(press3_x),float(press3_y),float(press3_z),0,0,0,
        float(press4_x),float(press4_y),float(press4_z),0,0,0,
        float(witdh*PIXEL_LENGTH),float(lenght*PIXEL_LENGTH)]
        sglobject_obj.features=subobject_info_cal
        single_frame_features.append(sglobject_obj)
    return
        #process depth_img to void zeross
def get_point_depth(cv_depth_image,u,v):
    depth_image=cv_depth_image.copy()
    times=5
    u=int(u)
    v=int(v)


    if u<0:
        u=0
    if u>IMG_COL-1:
        u=IMG_COL-1
    if v<0:
        v=0
    if v>IMG_ROW-1:
        v=IMG_ROW-1
    depth=int(depth_image[v,u])
    u_temp=copy.copy(u)
    v_temp=copy.copy(v)
    for time in range(times):
        #print("time_depth",time,depth)
        if depth!=0:
            break
        else:
            depth_image=cv2.resize(depth_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
            depth_image = cv2.dilate(depth_image, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)),iterations=1)
            v=math.floor(v/2)
            u=math.floor(u/2)
            print("u",u)
            depth=int(depth_image[int(v),int(u)])
    depth_still=int(still_depth_image[int(v_temp),int(u_temp)])
    #print("depth,depth_still",depth,depth_still)
    temp=depth-depth_still

    if abs(temp)>DEPTH_COMPARE_TH and depth_still!=0:
        depth=depth_still
       # print("~~~~~depth",depth)

    return depth


def get_point_depth_min(cv_depth_image,u,v):
    depth_image=cv_depth_image.copy()
    times=5
    u=int(u)
    v=int(v)
    num=0

    if u<1:
        u=0
        num=num+1
    if u>IMG_COL-2:
        u=IMG_COL-1
        num=num+1
    if v<1:
        v=0
        num=num+1
    if v>IMG_ROW-2:
        v=IMG_ROW-1
        num=num+1
    if num!=0:
        depth=int(depth_image[v,u])
        return depth
    depth=int(depth_image[v,u])
    u_temp=copy.copy(u)
    v_temp=copy.copy(v)
    for time in range(times):
        #print("time_depth",time,depth)
        if depth!=0:
            break
        else:
            depth_min=10000
            for i in range(-1,2,1):
                for j in range(-1,2,1):
                    depth_temp=int(depth_image[int(v_temp+i),int(u_temp+j)])
                    if (depth_temp!=0)and (depth_temp<depth_min):
                        depth_min=depth_temp
                    else:
                        pass
            if depth_min!=10000:
                depth=depth_min
                break
            depth_image=cv2.resize(depth_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
            v_temp=math.floor(v_temp/2)
            u_temp=math.floor(u_temp/2)
    #print("u",u_temp)
    depth_still=int(still_depth_image[int(v),int(u)])
    #print("depth,depth_still",depth,depth_still)
    temp=depth-depth_still


    if abs(temp)>DEPTH_COMPARE_TH and depth_still!=0:
        depth=depth_still
        #print("~~~~~depth",depth)

    return depth

if __name__ == '__main__':
    listener()
    fp.close()
    cv2.destroyAllWindows()





