#!/usr/bin/env python
import os,sys
from kitchen.msg import events
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
import math
import rospy
import yaml

abs_file1 = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file1 + "/../../lib/log")

from rlog import rlog
log = rlog()
log.setModuleName('sanitycheck')
log_level = 'info'
log.warn("set log level to [{}]".format(log_level))
log.set_priority(log_level)

abs_file = os.path.abspath(os.path.dirname(__file__))+"/"
config_yaml_file_path = "config.yaml"

global RationalROI, RegionNum, RegionObjectReady, RegionForeign
RationalROI = (0,0,0,0)
RegionNum = 0
RegionObjectReady = {}
RegionForeign = (0,0,0,0)

MatchNum_FRAME = 300
MatchNum_ROI = 30
SHOW_DEBUG = True

def get_config_unit():
    config_full_path = abs_file + config_yaml_file_path
    is_exist = os.path.exists(config_full_path)
    if is_exist is False:
        log.notice("cannot find sanity config file! path:{}".format(config_yaml_file_path))
        return False
    params_yaml = yaml.load_all(file(config_full_path, 'r'))
    if params_yaml is None:
        return False 

    params_sanity = []
    for y in params_yaml:
        params_sanity.append(y)

    global RationalROI
    dicttemp = params_sanity[0]['CameraCheckRationalROI']
    RationalROI = (dicttemp['top'],dicttemp['left'],dicttemp['bottom'],dicttemp['right'])
    global RegionNum 
    RegionNum = params_sanity[0]['ObjReadinessNum']
    global RegionObjectReady

    dicttemp = params_sanity[0]['ObjReadinessRegion']
    for name in dicttemp:
        RegionObjectReady[name] = params_sanity[0]['ObjReadinessRegion'][name]
    
    global RegionForeign
    dicttemp = params_sanity[0]['ForeignObjRegion']
    RegionForeign = (dicttemp['top'],dicttemp['left'],dicttemp['bottom'],dicttemp['right'])

    return True


class Checker:
    def __init__(self):
        get_config_unit()
        self.CameraReadyStatus = 0
        self.boolObjectReadyList = [False for i in range(RegionNum)]
        self.ObjectReadyScoreHistory = [0.0 for i in range(RegionNum)]
        self.ForeignScoreHistory = 0.0
        self.frameCounter = 0
        self.ForeignPosX = 0
        self.ForeignPosY = 0
        
        
    def check(self, vis, objs):
       
        self.frameCounter = self.frameCounter + 1
        sanityevents = events()
        
        abnormalevents = np.zeros(3,float)
        #check camera ready
        if(self.frameCounter <= 20):
            log.notice('checkCameraReady')
            self.CameraReadyStatus = self.checkCameraReady(vis)
            #if self.CameraReadyStatus == 1:
            #    abnormalevents[0] = 1
            #elif self.CameraReadyStatus == 2:
            #    abnormalevents[1] = 1
            abnormalevents[0] = 0
            abnormalevents[1] = 0
        
        #check object ready
        self.checkObjectReady(vis, objs)
        
        #check foreign object exsitance
        self.boolForeignObject = self.checkForeignObject(vis, objs)
        
        if self.ForeignScoreHistory > 0.7:
            abnormalevents[2] = 1
            sanityevents.foreignObjPos.data.append(self.ForeignPosX - 333)
            sanityevents.foreignObjPos.data.append(self.ForeignPosY - 429)

        sanityevents.abnormalEvents = Float32MultiArray(data = abnormalevents)
        sanityevents.objReadiness = Float32MultiArray(data = self.boolObjectReadyList)

        return sanityevents

    def checkCameraReady(self, vis):

        (height, width, channel) = vis.shape
        (roi_ly, roi_lx, roi_ry, roi_rx) = RationalROI
        
        #whole frame processing in order to detect lighting change and occlusion  
        template_path = "template.jpg"
        template_full_path = abs_file + template_path
        alarm_path = abs_file + "alarm.jpg"       
        tempframe = cv2.imread(template_full_path, 0)
        if tempframe is None:
            log.notice('can not find template image to check if camera is ready')
            return
        
        currframe = cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY)
        orb = cv2.ORB_create()
        kp1, des1 = orb.detectAndCompute(tempframe, None)
        kp2, des2 = orb.detectAndCompute(currframe, None)
    
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
        
        if  des1 is not None and des2 is not None:
            matches = bf.match(des1,des2)
        else:
            #cv2.imwrite(alarm_path, vis)
            log.notice('scene error')
            return 2    
    
        matches = sorted(matches, key = lambda x:x.distance)
        matching_frame = cv2.drawMatches(tempframe, kp1, currframe, kp2, matches[:MatchNum_FRAME], None, flags=2)
        
        #if SHOW_DEBUG == True:
            #print('no. of matched feature for each frame', len(matches))
            #cv2.imshow('matching_frame', matching_frame)

        if len(matches) < MatchNum_FRAME:
            #cv2.imwrite(alarm_path, vis)
            log.notice('scene error')
            return 2  

        #roi processing in order to detect camera motion
        temproi = tempframe[roi_ly:roi_ry,roi_lx:roi_rx]
        #currroi = currframe[roi_ly:roi_ry,roi_lx:roi_rx]
        orb_r = cv2.ORB_create()
        kp1_r, des1_r = orb_r.detectAndCompute(temproi, None)
        kp2_r, des2_r = orb_r.detectAndCompute(currframe, None)
        bf_r = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
        
        if des1_r is not None and des2_r is not None:
            matches_r = bf_r.match(des1_r,des2_r)
        else:
            #cv2.imwrite(alarm_path, vis)
            log.notice('camera moved')
            return 1

        matches_r = sorted(matches_r, key = lambda x:x.distance)
        matching_roi = cv2.drawMatches(temproi, kp1_r, currframe, kp2_r, matches_r[:MatchNum_ROI], None, flags=2)
        
        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches[:MatchNum_ROI]]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches[:MatchNum_ROI]]).reshape(-1, 1, 2)

        #if SHOW_DEBUG == True:
            #print('no. of matched feature for each roi', len(matches_r))
            #cv2.imshow("temproi",temproi)
            #cv2.imshow('matching_roi', matching_roi)
            #cv2.waitKey(1)
        
        if(len(src_pts) is not 0):
            error = 0
            for i in range(len(src_pts)):
                dist = math.sqrt((src_pts[i][0][0]-dst_pts[i][0][0])**2 + (src_pts[i][0][1]-dst_pts[i][0][1])**2) 
                error = error + dist
            error = error / len(src_pts)
            log.notice("camera motion:", error)
            if(error < 5):
                return 0
            else:
                #cv2.imwrite(alarm_path, vis)
                log.notice('camera moved')
                return 1
        else:
            #cv2.imwrite(alarm_path, vis)
            log.notice('scene error')
            return 2

    def checkInvision(self, vis):
        return True

    def getObjectReadyScore(self, roi, objtype, objs):
        score = 0
        currscore = 0
        rec1 = roi
        if objs is None:
            return score
        for currobj in objs.objects_vector:
            if currobj.classname == objtype:
                rec2 = (currobj.roi.y_offset, currobj.roi.x_offset, currobj.roi.y_offset+currobj.roi.height, currobj.roi.x_offset+currobj.roi.width)
                S_rec2 = (rec2[2] - rec2[0]) * (rec2[3] - rec2[1]) 
                left_line = max(rec1[1], rec2[1])
                right_line = min(rec1[3], rec2[3])
                top_line = max(rec1[0], rec2[0])
                bottom_line = min(rec1[2], rec2[2])
                if left_line >= right_line or top_line >= bottom_line:
                    currscore = 0
                else:
                    intersect = (right_line - left_line) * (bottom_line - top_line)
                    currscore = intersect*1.0/S_rec2*1.0
                if currscore > score:
                    score = currscore
        return score

    def getObjectReadyScoreNoType(self, rec1, objs):
        score = 0
        currscore = 0
        if objs is None:
            return score
        for currobj in objs.objects_vector:
                rec2 = (currobj.roi.y_offset, currobj.roi.x_offset, currobj.roi.y_offset+currobj.roi.height, currobj.roi.x_offset+currobj.roi.width)
                S_rec2 = (rec2[2] - rec2[0]) * (rec2[3] - rec2[1]) 
                left_line = max(rec1[1], rec2[1])
                right_line = min(rec1[3], rec2[3])
                top_line = max(rec1[0], rec2[0])
                bottom_line = min(rec1[2], rec2[2])
                if left_line >= right_line or top_line >= bottom_line:
                    currscore = 0
                else:
                    intersect = (right_line - left_line) * (bottom_line - top_line)
                    currscore = intersect*1.0/S_rec2*1.0
                if currscore > score:
                    score = currscore
        return score            

    def checkObjectReady(self, vis, objs):
        
        disp = vis.copy()
        
        flag = 0
        if rospy.has_param('objreadinessflag'):
            flag = rospy.get_param('objreadinessflag')
        else:
            flag = 1
        
        for i, j in RegionObjectReady.items():
            re = i.find('_')
            count = int(i[re+1:])
            if flag == 1:
                score = self.getObjectReadyScore(j,i[:re],objs)
                self.ObjectReadyScoreHistory[count] = 0.5*self.ObjectReadyScoreHistory[count] + 0.5*score 
            elif flag == 0:
                self.ObjectReadyScoreHistory[count] = 0
            
            if self.ObjectReadyScoreHistory[count] > 0.6:
                self.boolObjectReadyList[count] = True
            else:
                self.boolObjectReadyList[count] = False
            
            showscore = round(self.ObjectReadyScoreHistory[count],3)
            
            if(SHOW_DEBUG == True):
                if self.ObjectReadyScoreHistory[count] > 0.6:
                    cv2.rectangle(disp, (j[1],j[0]), (j[3],j[2]), (255, 0, 0), 2)
                    cv2.putText(disp,i[re+1:]+':'+str(i[:re])+'('+str(showscore)+')',(j[1],j[0]),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0),2,cv2.LINE_AA)   
                else:
                    cv2.rectangle(disp, (j[1],j[0]), (j[3],j[2]), (0, 0, 255), 2)
                    cv2.putText(disp,i[re+1:]+':'+str(i[:re])+'('+str(showscore)+')',(j[1],j[0]),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2,cv2.LINE_AA)   
 
            
        #for currobj in objs.objects_vector:
            #topleft = (int(currobj.roi.x_offset),int(currobj.roi.y_offset))
            #bottomright = (int(currobj.roi.x_offset+currobj.roi.width),int(currobj.roi.y_offset+currobj.roi.height))
            #cv2.rectangle(disp, topleft, bottomright, (0, 0, 255), 1)
            #cv2.putText(disp,currobj.classname,topleft,cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),1,cv2.LINE_AA)
        
        if(SHOW_DEBUG == True):  
            if flag == 0:
                cv2.putText(disp, 'NO DETECTION', (10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7, (0, 0, 255),2,cv2.LINE_AA) 
            elif flag == 1:
                cv2.putText(disp, 'DETECTING...', (10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7, (0, 0, 255),2,cv2.LINE_AA) 
            cv2.imshow('checkObjectReady', disp)
            cv2.waitKey(1)

    def checkForeignObject(self, vis, objs):
        disp = vis.copy()
        (roi_ly, roi_lx, roi_ry, roi_rx) = RegionForeign
        image_roi = disp[roi_ly:roi_ry, roi_lx:roi_rx].copy()

        gray_roi = cv2.cvtColor(image_roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray_roi, (5,5), 0)
        thresh = cv2.threshold(blurred, 150, 250, cv2.THRESH_BINARY_INV)[1]
        dilated = cv2.dilate(thresh, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(8,8)),iterations=1)
        #eroded = cv2.erode(thresh, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)),iterations=1)
        #cv2.imshow('thresh',dilated)
        contours = None
        hier = None
        if cv2.__version__.startswith('2') or cv2.__version__.startswith('4'):
            contours,hier = cv2.findContours(dilated.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        else:
            im, contours, hier = cv2.findContours(dilated.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        score = 0.0
        if(len(contours) > 0):
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            cnt = contours[max_index]
            maxarea = cv2.contourArea(cnt)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            longside = max(rect[1][0],rect[1][1])
            shortside = min(rect[1][0], rect[1][1])
            #print('long',longside,'short',shortside)
            cv2.drawContours(disp, [box], 0,(255,0,0),2, offset=(roi_lx,roi_ly))
            
            overlapscore = 0
            (height, width, channel) = vis.shape
            xmin = width
            ymin = height
            xmax = 0
            ymax = 0
            for point in box:
                if(point[0] < xmin):
                    xmin = point[0]
                if(point[0] > xmax):
                    xmax = point[0]
                if(point[1] < ymin):
                    ymin = point[1]
                if(point[1] > ymax):
                    ymax = point[1]
            rec1 = (ymin+roi_ly, xmin+roi_lx, ymax+roi_ly, xmax+roi_lx)
            #overlapscore = self.getObjectReadyScoreNoType(rec1, objs)
            
            #if(shortside > 40 and shortside < 70 and longside > 91 and longside < 110 and overlapscore < 0.3):
            if(shortside > 30 and shortside < 80 and longside > 70 and longside < 120):
                score = 1.0
                self.ForeignPosX = (rec1[1] + rec1[3])*0.5
                self.ForeignPosY = (rec1[0] + rec1[2])*0.5
                if(SHOW_DEBUG == True):
                    cv2.drawContours(disp, [box], 0,(255,255,0),2, offset=(roi_lx,roi_ly))
            else:
                score = 0.0
        else:
            score = 0.0

        self.ForeignScoreHistory = 0.6*self.ForeignScoreHistory + 0.4*score 
        if self.ForeignScoreHistory < 0.1:
            self.ForeignScoreHistory = 0.0
        result = round(self.ForeignScoreHistory,2)
        if(SHOW_DEBUG == True):
            cv2.rectangle(disp, (roi_lx,roi_ly), (roi_rx,roi_ry), (0,0,255), 2)
            #cv2.rectangle(disp,(rec1[1],rec1[0]),(rec1[3],rec1[2]),(0,255,0),2)
            cv2.putText(disp, 'foreignObjRegion['+str(result)+']', (roi_lx,roi_ly),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1,cv2.LINE_AA)
            if(self.ForeignScoreHistory > 0.8):
                cv2.putText(disp, 'ALARM!!!', (15+roi_lx,15+roi_ly),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1,cv2.LINE_AA)
                #cv2.imwrite('alarm.jpg',disp)
            cv2.imshow('checkForeignObject', disp)
            cv2.waitKey(1)
