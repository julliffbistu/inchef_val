#!/usr/bin/env python
from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import darknet
from kitchen.msg import objs_A
from kitchen.msg import obj_A
import sys
import os

class YOLODetector():
    def __init__(self):

        self.metaMain = None
        self.netMain = None
        self.altNames = None
        #self.configPath = "./yolo/yolov4-owntest.cfg"
        #self.weightPath = "./yolo/yolov4-owntrain_best.weights"
        #self.metaPath = "./yolo/obj.data"

        abs_file = os.path.abspath(os.path.dirname(__file__))
        self.configPath = abs_file + "/yolov4-owntest.cfg"
        self.weightPath = abs_file + "/yolov4-owntrain_best.weights"
        self.metaPath = abs_file + "/obj.data"

        if not os.path.exists(self.configPath):
            raise ValueError("Invalid config path `" + os.path.abspath(self.configPath)+"`")
        if not os.path.exists(self.weightPath):
            raise ValueError("Invalid weight path `" + os.path.abspath(self.weightPath)+"`")
        if not os.path.exists(self.metaPath):
            raise ValueError("Invalid data file path `" + os.path.abspath(self.metaPath)+"`")
        
        if self.netMain is None:
            self.netMain = darknet.load_net_custom(self.configPath.encode("ascii"), self.weightPath.encode("ascii"), 0, 1)  # batch size = 1
        if self.metaMain is None:
            self.metaMain = darknet.load_meta(self.metaPath.encode("ascii"))
        if self.altNames is None:
            try:
                with open(self.metaPath) as metaFH:
                    metaContents = metaFH.read()
                    import re
                    match = re.search("names *= *(.*)$", metaContents,
                                    re.IGNORECASE | re.MULTILINE)
                    if match:
                        result = match.group(1)
                    else:
                        result = None
                    try:
                        if os.path.exists(result):
                            with open(result) as namesFH:
                                namesList = namesFH.read().strip().split("\n")
                                self.altNames = [x.strip() for x in namesList]
                    except TypeError:
                        pass
            except Exception:
                pass

        self.darknet_image = darknet.make_image(darknet.network_width(self.netMain),
                                        darknet.network_height(self.netMain),3)

    def convertBack(self, x, y, w, h):
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax

    def cvDrawBoxes(self, detections, img):
        for detection in detections:
            if(detection[1]<0.5):
                continue
            x, y, w, h = detection[2][0],\
                detection[2][1],\
                detection[2][2],\
                detection[2][3]
            xmin, ymin, xmax, ymax = self.convertBack(
                float(x), float(y), float(w), float(h))
            pt1 = (xmin, ymin)
            pt2 = (xmax, ymax)
            cv2.rectangle(img, pt1, pt2, (0, 255, 0), 1)
            cv2.putText(img,
                        detection[0].decode() +
                        " [" + str(round(detection[1] * 100, 2)) + "]",
                        (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        [0, 255, 0], 2)
        return img
    if False:
        def segment(self, detections, img):
            widthnet = darknet.network_width(self.netMain)
            heightnet = darknet.network_height(self.netMain)
            (height,width,channel) = img.shape
            print(img.shape)
            
            for detection in detections:
                if('beef' in detection[0] and detection[1]>0.5):
                    roi_lx = detection[2][0]*width/widthnet
                    roi_ly = detection[2][1]*height/heightnet
                    roi_w = detection[2][2]*width/widthnet
                    roi_h = detection[2][3]*height/heightnet
                    xmin, ymin, xmax, ymax = self.convertBack(float(roi_lx), float(roi_ly), float(roi_w), float(roi_h))
                    temproi = img[ymin:ymax,xmin:xmax]
                    cnt = self.getContour(temproi,1)
                    if(cnt is None):
                        return
                    cv2.drawContours(img, [cnt], 0,(255,255,0),2, offset = (xmin,ymin))
                    cv2.imshow('roi',img)
                    cv2.waitKey(2)
                elif('broccoli' in detection[0] and detection[1] > 0.5):
                    roi_lx = detection[2][0]*width/widthnet
                    roi_ly = detection[2][1]*height/heightnet
                    roi_w = detection[2][2]*width/widthnet
                    roi_h = detection[2][3]*height/heightnet
                    xmin, ymin, xmax, ymax = self.convertBack(float(roi_lx), float(roi_ly), float(roi_w), float(roi_h))
                    temproi = img[ymin:ymax,xmin:xmax]
                    cv2.imshow('temp', temproi)
                    cv2.imwrite('temp.jpg', temproi)
                    cnt = self.getContour(temproi,2) 
                    print(cnt)
                    if(cnt is None):
                        return
                    cv2.drawContours(img, [cnt], 0,(255,255,0),2, offset = (xmin,ymin))   
                    cv2.imshow('roi',img)
                    cv2.waitKey(2)

    
        def getContour(self, _img, flag):
            cv2.imshow("img", _img)
            img = _img.copy()
            hsvroi = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            colorMask = None
            if(flag == 1):    
                lower_hsv1 = np.array([0,60,32])
                upper_hsv1 = np.array([20,255,255])
                colorMask1 = cv2.inRange(hsvroi, lower_hsv1, upper_hsv1)
                lower_hsv2 = np.array([170,60,32])
                upper_hsv2 = np.array([180,255,255])
                colorMask2 = cv2.inRange(hsvroi, lower_hsv2, upper_hsv2)
                colorMask = cv2.addWeighted(colorMask1, 1, colorMask2, 1, 0)
            elif(flag == 2):
                print('broccoli')
                lower_hsv = np.array([30,0,32])
                upper_hsv = np.array([50,50,255])
                cv2.imshow("hsv", hsvroi)
                colorMask = cv2.inRange(hsvroi, lower_hsv, upper_hsv)
            cv2.imshow("mask", colorMask)
            eroded = cv2.erode(colorMask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)),iterations=1)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(8,5)),iterations=2)
            im,contours,hier = cv2.findContours(dilated, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            if(len(contours)>0):
                areas = [cv2.contourArea(c) for c in contours]
                max_index = np.argmax(areas)
                cnt = contours[max_index]
                #maxarea = cv2.contourArea(cnt)
                return cnt
                cv2.drawContours(img, [cnt], 0,(255,255,0),2)
                cv2.imshow("img",img)
                cv2.waitKey(2)

        def runGrabCut(self, _image, detections):
            print("runGrabCut")
            widthnet = darknet.network_width(self.netMain)
            heightnet = darknet.network_height(self.netMain)
            (height,width,channel) = _image.shape
            imgs = []
            for detection in detections:
                if(detection[1]>0.5):
                    if('beef' in detection[0] or 'broccoli' in detection[0]):
                        mask = np.zeros(_image.shape[:2],np.uint8)
                        bgdModel = np.zeros((1,65), np.float64)
                        fgbModel = np.zeros((1,65), np.float64)
                        roi_lx = detection[2][0]*width/widthnet
                        roi_ly = detection[2][1]*height/heightnet
                        roi_w = detection[2][2]*width/widthnet
                        roi_h = detection[2][3]*height/heightnet
                        xmin, ymin, xmax, ymax = self.convertBack(float(roi_lx), float(roi_ly), float(roi_w), float(roi_h))
                        _xmin = max(xmin-2,0)
                        _ymin = max(ymin-2,0)
                        _xmax = min(xmax+2,width)
                        _ymax = min(ymax+2,height)
                        rect = ((xmin-_xmin),(ymin-_ymin),(xmax-_xmin),(ymax-_ymin))

                        #rect = (xmin, ymin, xmax, ymax)
                        image = _image[_ymin:_ymax,_xmin:_xmax]  
                        mask = np.zeros(image.shape[:2],np.uint8)
                        cv2.grabCut(image, mask, rect, bgdModel, fgbModel, 10, cv2.GC_INIT_WITH_RECT)
                        mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
                        image = image*mask2[:,:,np.newaxis]
                        imgs.append(image)
            return imgs

    def detect(self, vis):
        (height, width, channel) = vis.shape
        widthnet = darknet.network_width(self.netMain)
        heightnet = darknet.network_height(self.netMain)

        prev_time = time.time()
        # Create an image we reuse for each detect
	
        frame_rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb,
                                (widthnet, heightnet),
                                interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(self.darknet_image,frame_resized.tobytes())

        detections = darknet.detect_image(self.netMain, self.metaMain, self.darknet_image, thresh=0.25)
        
        detectedObjs = objs_A()
        if len(detections) == 0:
            detectedObjs = None
        else:
            i = 0

            for detection in detections:
                if(detection[1]<0.5):
                    continue
                curr = obj_A()
                curr.id = i
                i = i+1
                curr.classname = detection[0]
                curr.probability = detection[1]

            	x, y, w, h = detection[2][0],\
                detection[2][1],\
                detection[2][2],\
                detection[2][3]
            	xmin, ymin, xmax, ymax = self.convertBack(
                	float(x), float(y), float(w), float(h))
                
		curr.roi.x_offset = xmin*width/widthnet
                curr.roi.y_offset = ymin*height/heightnet
                curr.roi.width = detection[2][2]*width/widthnet
                curr.roi.height = detection[2][3]*height/heightnet
                curr.pose.position.x = 0
                curr.pose.position.y = 0
                curr.pose.position.z = 0
                curr.pose.orientation.x = 0
                curr.pose.orientation.y = 0
                curr.pose.orientation.z = 0
                curr.pose.orientation.w = 0
                detectedObjs.objects_vector.append(curr)

        #self.segment(detections, vis)
        #images = self.runGrabCut(vis, detections)
        #for i in range(len(images)):
        #    cv2.imshow("Images{}".format(i), images[i])
    
        image = self.cvDrawBoxes(detections, frame_resized)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #print(1/(time.time()-prev_time))
        image_resized = cv2.resize(image,(960, 540),interpolation=cv2.INTER_LINEAR)
        cv2.imshow('Demo', image_resized)
        cv2.waitKey(1)
        return detectedObjs
