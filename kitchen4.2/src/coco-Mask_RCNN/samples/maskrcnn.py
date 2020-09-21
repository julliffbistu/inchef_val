#!/usr/bin/env python
# coding: utf-8

import cv2
import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ['CUDA_VISIBLE_DEVICES'] = "0,1"
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

from keras.backend.tensorflow_backend import set_session
import tensorflow as tf
config = tf.ConfigProto()
config.gpu_options.allow_growth = True  # dynamically grow the memory used on the GPU
config.log_device_placement = True  # to log device placement (on which device the operation ran)
sess = tf.Session(config=config)
set_session(sess)  # set this TensorFlow session as the default session for Keras

import sys
import random
import math
import skimage.io
import time
import mrcnn.config
from mrcnn import utils
from mrcnn import utils
import mrcnn.model as modellib

from mrcnn.config import Config
from mrcnn import visualize
#import model as modellib
import matplotlib.pyplot as plt
ROOT_DIR = os.path.abspath("./")
print("niubibibibi:::::",ROOT_DIR)
sys.path.append(ROOT_DIR)

from PIL import Image
#import coco
def random_colors(N):
    np.random.seed(1)
    colors=[tuple(255*np.random.rand(3)) for _ in range(N)]
    return colors

def apply_mask(image, mask, color,label):
    """Apply the given mask to the image.
    """
    #if(mask[:][0]==True):
    #print("img::",mask[:][240])
    alpha=0.2
    for n, c in enumerate(color):
        image[:, :, n] = np.where(
            mask == 1,
            image[:, :, n] *(1 - alpha) + alpha * c,
            image[:, :, n]
        )
    
    return image
##class_names = ['BG','plate', 'pan','vegetablebowl','broccoli','souppothandle','panhandle','beef','nethandle','seasoningbottle','seasoningbowl']
def display_instances(image,boxes,masks,ids,names,scores):

    n_instances=boxes.shape[0]
    plate_score=[0,0,0,0,0]
    pan_score=[]
    vegetablebowl_score=[]
    broccoli_score=[0,0,0]
    souppothandle_score=[]
    panhandle_score=[]
    beef_score=[]
    nethandle_score=[]
    seasoningbottle_score=[0,0]
    seasoningbowl_score=[]
    for i in range(ids.size):
        if(ids[i]==1):
            pan_score.append(scores[i])
        if(ids[i]==2):
            beef_score.append(scores[i])
        if(ids[i]==3):
            plate_score.append(scores[i])
        if(ids[i]==4):
            vegetablebowl_score.append(scores[i])
        if(ids[i]==5):
            broccoli_score.append(scores[i])
        if(ids[i]==6):
            souppothandle_score.append(scores[i])
        if(ids[i]==7):
            panhandle_score.append(scores[i])
        if(ids[i]==8):
            beef_score.append(scores[i])
        if(ids[i]==9):
            nethandle_score.append(scores[i])
        if(ids[i]==10):
            seasoningbottle_score.append(scores[i])
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

    print("plate score 1:           ",plate_score)
    print("pan score 2:             ",pan_score)
    print("vegetablebowl score 3:   ",vegetablebowl_score)
    print("broccoli score 4:        ",broccoli_score)
    print("souppothandle score 5:   ",souppothandle_score)
    print("panhandle score 6:       ",panhandle_score)
    print("beef score 7:            ",beef_score)
    print("nethandle score 8:       ",nethandle_score)
    print("seasoningbottle score 9: ",seasoningbottle_score)
    print("seasoningbowl score 10:  ",seasoningbowl_score)
    print('---'*30)

    n=0
    if not n_instances:
        print('No instances to display')
    else:
        assert boxes.shape[0] == masks.shape[-1] == ids.shape[0]  
    colors=random_colors(n_instances)
    height, width = image.shape[:2]
    for i,color in enumerate(colors):
        #if scores[i]>0.85:
        if not np.any(boxes[i]):
            continue
        y1,x1,y2,x2=boxes[i]
        mask=masks[:,:,i]
        label=names[ids[i]]
        
        if(label=="beef" and scores[i] == beef_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                
                rect = cv2.minAreaRect(contours[c])
                cx, cy = rect[0]
                Area = cv2.contourArea(contours[c])
                x,y,w,h = cv2.boundingRect(contours[c])
                cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
                
                grasp_pixcel = 12    #yan chang 
                grasp_pointx_left,grasp_pointy_left = cx-w/2-grasp_pixcel,cy
                grasp_pointx_right,grasp_pointy_right = cx + w/2+grasp_pixcel,cy
                
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                print("************-------------*****************")
                image = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=180,0
                
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)

            print("beef is:                  ",[int(cx), int(cy)],scores[i])
            print("beef grasp_point_left:    ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("beef grasp_point_right:   ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("beef theta:[left,right]:  ",((theta_left),(theta_right)))
            print("beef push point1:         ",[push_point1_x,push_point1_y])
            print("beef push point2:         ",[push_point2_x,push_point2_y])
            print("beef push point3:         ",[push_point3_x,push_point3_y])
            print("beef push point4:         ",[push_point4_x,push_point4_y])
            print("beef area is:             ",Area)
        
        
        
        '''
        if(label=="beef" and scores[i] == beef_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)

            print("beef is:                  ",[int(cx), int(cy)],scores[i])
            print("beef grasp_point_left:    ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("beef grasp_point_right:   ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("beef theta:[left,right]:  ",((theta_left),(theta_right)))
            print("beef push point1:         ",[push_point1_x,push_point1_y])
            print("beef push point2:         ",[push_point2_x,push_point2_y])
            print("beef push point3:         ",[push_point3_x,push_point3_y])
            print("beef push point4:         ",[push_point4_x,push_point4_y])
            print("beef area is:             ",Area)
            '''

        if(label=="pan" and scores[i] == pan_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("pan is:                   ",[int(cx), int(cy)],scores[i])
            print("pan grasp_point_left:     ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("pan grasp_point_right:    ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("pan theta:[left,right]:   ",((theta_left),(theta_right)))
            print("pan push point1:          ",[push_point1_x,push_point1_y])
            print("pan push point2:          ",[push_point2_x,push_point2_y])
            print("pan push point3:          ",[push_point3_x,push_point3_y])
            print("pan push point4:          ",[push_point4_x,push_point4_y])
            print("pan area is:              ",Area)
        
        if(label=="plate" and scores[i] == plate_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                if Area> 5500: #beef 2000
                    cx, cy = rect[0]
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    #print("四个顶点坐标为;", box)
                    #print("中心坐标：", rect[0])
                    cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                    #print("宽度：", rect[1][0])
                    #print("长度：", rect[1][1])

                    length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                    length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                    grasp_pixcel = 60
                    if(rect[1][1]>0):
                        grasp_bit = grasp_pixcel/ rect[1][1]/2
                    else:
                        grasp_bit = 0.2
                    if length1>length2:
                        pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                        grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                        grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                    
                    else:
                        pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                        grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                        grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                    cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                    cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                    im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                    
                    #("size:",contours[c].size)
                    number = int(len(contours[c])/4)
                    #print("size:",number)
                    cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                    
                    push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                    push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                    push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                    push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                    #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                    theta_left,theta_right=0,0
                    if(cx-pointx_left!=0):
                        theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                        #print("旋转 right 角度：", theta_right)
                        theta_left = theta_right + 180
                    #print("旋转 left 角度：", theta_left)
                    cv2.drawContours(image,[box],0,(0,255,0),1)
                    cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                    if(len(contours[c]) >= 5):
                        ellipse = cv2.fitEllipse(contours[c])
                        #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                        #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
                    score=scores[i] if scores is not None else None
                    caption='{}{:.2f}'.format(label,score) if score else label
                    image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
                    print("plate one is:             ",[int(cx), int(cy)],scores[i])
                    print("plate grasp_point_left:   ",[int(grasp_pointx_left),int(grasp_pointy_left)])
                    print("plate grasp_point_right:  ",[int(grasp_pointx_right),int(grasp_pointy_right)])
                    print("plate theta:[left,right]: ",((theta_left),(theta_right)))
                    print("plate push point1:        ",[push_point1_x,push_point1_y])
                    print("plate push point2:        ",[push_point2_x,push_point2_y])
                    print("plate push point3:        ",[push_point3_x,push_point3_y])
                    print("plate push point4:        ",[push_point4_x,push_point4_y])
                    print("plate area is:            ",Area)

        if(label=="plate" and scores[i] == plate_score[1] and plate_score[1]!=0):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                if Area> 5500: #beef 2000
                    cx, cy = rect[0]
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    #print("四个顶点坐标为;", box)
                    #print("中心坐标：", rect[0])
                    cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                    #print("宽度：", rect[1][0])
                    #print("长度：", rect[1][1])

                    length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                    length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                    grasp_pixcel = 60
                    if(rect[1][1]>0):
                        grasp_bit = grasp_pixcel/ rect[1][1]/2
                    else:
                        grasp_bit = 0.2
                    if length1>length2:
                        pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                        grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                        grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                    
                    else:
                        pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                        grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                        grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                    cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                    cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                    im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                    
                    #("size:",contours[c].size)
                    number = int(len(contours[c])/4)
                    #print("size:",number)
                    cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                    
                    push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                    push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                    push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                    push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                    #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                    theta_left,theta_right=0,0
                    if(cx-pointx_left!=0):
                        theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                        #print("旋转 right 角度：", theta_right)
                        theta_left = theta_right + 180
                    #print("旋转 left 角度：", theta_left)
                    cv2.drawContours(image,[box],0,(0,255,0),1)
                    cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                    if(len(contours[c]) >= 5):
                        ellipse = cv2.fitEllipse(contours[c])
                        #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                        #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
                    score=scores[i] if scores is not None else None
                    caption='{}{:.2f}'.format(label,score) if score else label
                    image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
                    print("plate two is:             ",[int(cx), int(cy)],scores[i])
                    print("plate grasp_point_left:   ",[int(grasp_pointx_left),int(grasp_pointy_left)])
                    print("plate grasp_point_right:  ",[int(grasp_pointx_right),int(grasp_pointy_right)])
                    print("plate theta:[left,right]: ",((theta_left),(theta_right)))
                    print("plate push point1:        ",[push_point1_x,push_point1_y])
                    print("plate push point2:        ",[push_point2_x,push_point2_y])
                    print("plate push point3:        ",[push_point3_x,push_point3_y])
                    print("plate push point4:        ",[push_point4_x,push_point4_y])
                    print("plate area is:            ",Area)
            
        if(label=="plate"and scores[i] == plate_score[2] and plate_score[2]!=0):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                if Area> 5500: #beef 2000
                    cx, cy = rect[0]
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    #print("四个顶点坐标为;", box)
                    #print("中心坐标：", rect[0])
                    cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                    #print("宽度：", rect[1][0])
                    #print("长度：", rect[1][1])

                    length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                    length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                    grasp_pixcel = 60
                    if(rect[1][1]>0):
                        grasp_bit = grasp_pixcel/ rect[1][1]/2
                    else:
                        grasp_bit = 0.2
                    if length1>length2:
                        pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                        grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                        grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                    
                    else:
                        pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                        grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                        grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                    cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                    cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                    im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                    
                    #("size:",contours[c].size)
                    number = int(len(contours[c])/4)
                    #print("size:",number)
                    cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                    cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                    
                    push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                    push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                    push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                    push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                    #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                    theta_left,theta_right=0,0
                    if(cx-pointx_left!=0):
                        theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                        #print("旋转 right 角度：", theta_right)
                        theta_left = theta_right + 180
                    #print("旋转 left 角度：", theta_left)
                    cv2.drawContours(image,[box],0,(0,255,0),1)
                    cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                    if(len(contours[c]) >= 5):
                        ellipse = cv2.fitEllipse(contours[c])
                        #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                        #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
                    score=scores[i] if scores is not None else None
                    caption='{}{:.2f}'.format(label,score) if score else label
                    image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
                    print("plate three is:           ",[int(cx), int(cy)],scores[i])
                    print("plate grasp_point_left:   ",[int(grasp_pointx_left),int(grasp_pointy_left)])
                    print("plate grasp_point_right:  ",[int(grasp_pointx_right),int(grasp_pointy_right)])
                    print("plate theta:[left,right]: ",((theta_left),(theta_right)))
                    print("plate push point1:        ",[push_point1_x,push_point1_y])
                    print("plate push point2:        ",[push_point2_x,push_point2_y])
                    print("plate push point3:        ",[push_point3_x,push_point3_y])
                    print("plate push point4:        ",[push_point4_x,push_point4_y])
                    print("plate area is:            ",Area)



        if(label=="vegetablebowl" and scores[i] == vegetablebowl_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("vegetablebowl is:                ",[int(cx), int(cy)],scores[i])
            print("vegetablebowl grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("vegetablebowl grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("vegetablebowl theta:[left,right]:",((theta_left),(theta_right)))
            print("vegetablebowl push point1:       ",[push_point1_x,push_point1_y])
            print("vegetablebowl push point2:       ",[push_point2_x,push_point2_y])
            print("vegetablebowl push point3:       ",[push_point3_x,push_point3_y])
            print("vegetablebowl push point4:       ",[push_point4_x,push_point4_y])
            print("vegetablebowl area is:           ",Area)

        if(label=="broccoli" and scores[i] == broccoli_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("broccoli one is:            ",[int(cx), int(cy)],scores[i])
            print("broccoli grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("broccoli grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("broccoli theta:[left,right]:",((theta_left),(theta_right)))
            print("broccoli push point1:       ",[push_point1_x,push_point1_y])
            print("broccoli push point2:       ",[push_point2_x,push_point2_y])
            print("broccoli push point3:       ",[push_point3_x,push_point3_y])
            print("broccoli push point4:       ",[push_point4_x,push_point4_y])
            print("broccoli area is:           ",Area)

        if(label=="broccoli" and scores[i] == broccoli_score[1] and broccoli_score[1]!=0):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("broccoli two is:            ",[int(cx), int(cy)],scores[i])
            print("broccoli grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("broccoli grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("broccoli theta:[left,right]:",((theta_left),(theta_right)))
            print("broccoli push point1:       ",[push_point1_x,push_point1_y])
            print("broccoli push point2:       ",[push_point2_x,push_point2_y])
            print("broccoli push point3:       ",[push_point3_x,push_point3_y])
            print("broccoli push point4:       ",[push_point4_x,push_point4_y])
            print("broccoli area is:           ",Area)

        if(label=="broccoli" and scores[i] == broccoli_score[2] and broccoli_score[2]!=0):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("broccoli three is:          ",[int(cx), int(cy)],scores[i])
            print("broccoli grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("broccoli grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("broccoli theta:[left,right]:",((theta_left),(theta_right)))
            print("broccoli push point1:       ",[push_point1_x,push_point1_y])
            print("broccoli push point2:       ",[push_point2_x,push_point2_y])
            print("broccoli push point3:       ",[push_point3_x,push_point3_y])
            print("broccoli push point4:       ",[push_point4_x,push_point4_y])
            print("broccoli area is:           ",Area)

        if(label=="souppothandle" and scores[i] == souppothandle_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("souppothandle is:                ",[int(cx), int(cy)],scores[i])
            print("souppothandle grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("souppothandle grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("souppothandle theta:[left,right]:",((theta_left),(theta_right)))
            print("souppothandle push point1:       ",[push_point1_x,push_point1_y])
            print("souppothandle push point2:       ",[push_point2_x,push_point2_y])
            print("souppothandle push point3:       ",[push_point3_x,push_point3_y])
            print("souppothandle push point4:       ",[push_point4_x,push_point4_y])
            print("souppothandle area is:           ",Area)

        if(label=="panhandle" and scores[i] == panhandle_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("panhandle is:                ",[int(cx), int(cy)],scores[i])
            print("panhandle grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("panhandle grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("panhandle theta:[left,right]:",((theta_left),(theta_right)))
            print("panhandle push point1:       ",[push_point1_x,push_point1_y])
            print("panhandle push point2:       ",[push_point2_x,push_point2_y])
            print("panhandle push point3:       ",[push_point3_x,push_point3_y])
            print("panhandle push point4:       ",[push_point4_x,push_point4_y])
            print("panhandle area is:           ",Area)

        if(label=="nethandle" and scores[i] == nethandle_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("nethandle is:                ",[int(cx), int(cy)],scores[i])
            print("nethandle grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("nethandle grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("nethandle theta:[left,right]:",((theta_left),(theta_right)))
            print("nethandle push point1:       ",[push_point1_x,push_point1_y])
            print("nethandle push point2:       ",[push_point2_x,push_point2_y])
            print("nethandle push point3:       ",[push_point3_x,push_point3_y])
            print("nethandle push point4:       ",[push_point4_x,push_point4_y])
            print("nethandle area is:           ",Area)

        if(label=="seasoningbottle" and scores[i] == seasoningbottle_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("seasoningbottle one is:            ",[int(cx), int(cy)],scores[i])
            print("seasoningbottle grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("seasoningbottle grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("seasoningbottle theta:[left,right]:",((theta_left),(theta_right)))
            print("seasoningbottle push point1:       ",[push_point1_x,push_point1_y])
            print("seasoningbottle push point2:       ",[push_point2_x,push_point2_y])
            print("seasoningbottle push point3:       ",[push_point3_x,push_point3_y])
            print("seasoningbottle push point4:       ",[push_point4_x,push_point4_y])
            print("seasoningbottle area is:           ",Area)

        if(label=="seasoningbottle" and scores[i] == seasoningbottle_score[1] and seasoningbottle_score[1]!=0):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("seasoningbottle two is:            ",[int(cx), int(cy)],scores[i])
            print("seasoningbottle grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("seasoningbottle grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("seasoningbottle theta:[left,right]:",((theta_left),(theta_right)))
            print("seasoningbottle push point1:       ",[push_point1_x,push_point1_y])
            print("seasoningbottle push point2:       ",[push_point2_x,push_point2_y])
            print("seasoningbottle push point3:       ",[push_point3_x,push_point3_y])
            print("seasoningbottle push point4:       ",[push_point4_x,push_point4_y])
            print("seasoningbottle area is:           ",Area)

        if(label=="seasoningbowl" and scores[i] == seasoningbowl_score[0]):
            image=apply_mask(image,mask,color,label)
            cimgmask =np.uint8(mask)
            temp_mask=cimgmask*255
            area = []
            contours, hierarchy = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in range(len(contours)):
                rect = cv2.minAreaRect(contours[c])
                Area = cv2.contourArea(contours[c])
                cx, cy = rect[0]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #print("四个顶点坐标为;", box)
                #print("中心坐标：", rect[0])
                cv2.circle(image, (np.int32(cx), np.int32(cy)), 5, (0,0,255), -1)
                #print("宽度：", rect[1][0])
                #print("长度：", rect[1][1])

                length1=math.pow((box[1][1]-box[0][1]),2)+math.pow((box[1][0]-box[0][0]),2)
                length2=math.pow((box[2][1]-box[1][1]),2)+math.pow((box[2][0]-box[1][0]),2)
                grasp_pixcel = 60
                if(rect[1][1]>0):
                    grasp_bit = grasp_pixcel/ rect[1][1]/2
                else:
                    grasp_bit = 0.2
                if length1>length2:
                    pointx_left,pointy_left=(box[2][0]-box[1][0])/2+box[1][0],(box[2][1]-box[1][1])/2+box[1][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left
                
                else:
                    pointx_left,pointy_left=(box[1][0]-box[0][0])/2+box[0][0],(box[1][1]-box[0][1])/2+box[0][1]
                    grasp_pointx_left,grasp_pointy_left = grasp_bit*(pointx_left-cx)+pointx_left,grasp_bit*(pointy_left-cy)+pointy_left
                    grasp_pointx_right,grasp_pointy_right = 2*cx-grasp_pointx_left,2*cy-grasp_pointy_left

                cv2.circle(image, (np.int32(grasp_pointx_left), np.int32(grasp_pointy_left)), 6, (0,0,255), -1)
                cv2.circle(image, (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), 6, (0,0,255), -1)
                im = cv2.line(image, (np.int32(grasp_pointx_left),np.int32(grasp_pointy_left)), (np.int32(grasp_pointx_right), np.int32(grasp_pointy_right)), (0, 255, 255), 1)
                
                #("size:",contours[c].size)
                number = int(len(contours[c])/4)
                #print("size:",number)
                cv2.circle(image, (contours[c][0][0][0], contours[c][0][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*1][0][0], contours[c][number*1][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*2][0][0], contours[c][number*2][0][1]), 3, (0,0,0), -1)
                cv2.circle(image, (contours[c][number*3][0][0], contours[c][number*3][0][1]), 3, (0,0,0), -1)
                
                push_point1_x,push_point1_y= int(contours[c][0][0][0]), int(contours[c][0][0][1])
                push_point2_x,push_point2_y= int(contours[c][number*1][0][0]), int(contours[c][number*1][0][1])
                push_point3_x,push_point3_y= int(contours[c][number*2][0][0]), int(contours[c][number*2][0][1])
                push_point4_x,push_point4_y= int(contours[c][number*3][0][0]), int(contours[c][number*3][0][1])

                #cv2.circle(image, (contours[c][number*4][0][0], contours[c][number*4][0][1]), 6, (0,0,255), -1)
                theta_left,theta_right=0,0
                if(cx-pointx_left!=0):
                    theta_right = math.degrees(math.atan((pointy_left-cy)/(cx-pointx_left)))
                    #print("旋转 right 角度：", theta_right)
                    theta_left = theta_right + 180
                #print("旋转 left 角度：", theta_left)
                cv2.drawContours(image,[box],0,(0,255,0),1)
                cv2.drawContours(image, contours, c, (0, 0, 255), 1, 8)
                if(len(contours[c]) >= 5):
                    ellipse = cv2.fitEllipse(contours[c])
                    #print("zuobiao:",contours[c][1][0], contours[c][1][0][0],contours[c][1][0][1])
                    #cv2.ellipse(image, ellipse, (255, 255, 0), 2)
            score=scores[i] if scores is not None else None
            caption='{}{:.2f}'.format(label,score) if score else label
            image=cv2.putText(image,caption,(x1-5,y1-7),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),2)
            print("seasoningbowl one is:            ",[int(cx), int(cy)],scores[i])
            print("seasoningbowl grasp_point_left:  ",[int(grasp_pointx_left),int(grasp_pointy_left)])
            print("seasoningbowl grasp_point_right: ",[int(grasp_pointx_right),int(grasp_pointy_right)])
            print("seasoningbowl theta:[left,right]:",((theta_left),(theta_right)))
            print("seasoningbowl push point1:       ",[push_point1_x,push_point1_y])
            print("seasoningbowl push point2:       ",[push_point2_x,push_point2_y])
            print("seasoningbowl push point3:       ",[push_point3_x,push_point3_y])
            print("seasoningbowl push point4:       ",[push_point4_x,push_point4_y])
            print("seasoningbowl area is:           ",Area)

        print("-----"*10)
        
        #image=cv2.rectangle(image,(x1,y1),(x2,y2),color,2)
    return image


sys.path.append(os.path.join(ROOT_DIR, "samples/coco/"))  # To find local version
MODEL_DIR = os.path.join(ROOT_DIR, "logs")
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco_0040.h5")
if not os.path.exists(COCO_MODEL_PATH):
    print('cannot find coco_model')

class CocoConfig(Config):
    # 命名配置
    NAME = "coco"

    # 输入图像resing
    IMAGE_MIN_DIM = 540
    IMAGE_MAX_DIM = 960

    # 使用的GPU数量。 对于CPU训练，请使用1
    GPU_COUNT = 1

    IMAGES_PER_GPU = 1

    batch_size = GPU_COUNT * IMAGES_PER_GPU
    # STEPS_PER_EPOCH = int(train_images / batch_size * (3 / 4))
    STEPS_PER_EPOCH=1000

    VALIDATION_STEPS = STEPS_PER_EPOCH // (1000 // 50)

    NUM_CLASSES = 1 + 10  # 必须包含一个背景（背景作为一个类别） Pascal VOC 2007有20个类，前面加1 表示加上背景

    scale = 1024 // IMAGE_MAX_DIM
    RPN_ANCHOR_SCALES = (32 // scale, 64 // scale, 128 // scale, 256 // scale, 512 // scale)  

    RPN_NMS_THRESHOLD = 0.4  # 0.6

    RPN_TRAIN_ANCHORS_PER_IMAGE = 256 // scale

    MINI_MASK_SHAPE = (56 // scale, 56 // scale)

    TRAIN_ROIS_PER_IMAGE = 200 // scale

    DETECTION_MAX_INSTANCES = 100 * scale * 2 // 3

    DETECTION_MIN_CONFIDENCE = 0.8


#class InferenceConfig(coco.CocoConfig):
class InferenceConfig(CocoConfig):  ###zhu
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

config = InferenceConfig()
config.display()

#model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
# Load weights trained on MS-COCO
model.load_weights(COCO_MODEL_PATH, by_name=True)

class_names = ['BG','pan','beef','plate', 'vegetablebowl','broccoli','souppothandle','panhandle','nethandle','seasoningbottle','seasoningbowl']


def model_detection(img):

    results=model.detect([img],verbose=0)
    r=results[0]
    frame=display_instances(
            img,r['rois'], r['masks'], r['class_ids'], 
                        class_names, r['scores']
    )

