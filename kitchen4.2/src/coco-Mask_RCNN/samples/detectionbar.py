#!/usr/bin/env python
# coding: utf-8
###pip install opencv-contrib-python==3.3.0.10#3.4.2.16
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt


img_out1 = np.zeros((1080,1920),dtype=np.uint8)
img_out2 = np.zeros((1080,1920),dtype=np.uint8)
img_out3 = np.zeros((1080,1920),dtype=np.uint8)
img_out4 = np.zeros((1080,1920),dtype=np.uint8)
img_out5 = np.zeros((1080,1920),dtype=np.uint8)
img_out6 = np.zeros((1080,1920),dtype=np.uint8)

bar_pose = np.zeros(12,dtype=float)

pan_roi = [630,715,780,890]#(375,390)(435,415)
cai_roi = [500,950,675,1070]#(215,480)(290,535)
pot_roi = [1202,689,1351,799]#(570,365)(625,385)
net_roi = [1075,627,1216,800]#(640,295)(653,330)
net_roi_plate = [285,885,460,1050]#(135,440)(170,510)

center_x,center_y, theta_pan, theta_pot, theta_net,theta_net_plate= 0,0,1,1,1,1
panx, pany, pan_theta = 0,0,0
dlta_pan,dlta_pot,dlta_net,dlta_net_plat = -45,135,-145,0

def func_bar_detection_pan(img2):
    global center_x
    global center_y
    global theta_pan
    global panx
    global pany
    global pan_theta
    img1 = cv2.imread('./samples/roiimgback/pan_roi.png')
    cv2.imshow("back",img1)
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
   
    sift = cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    good = []
    #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)             
    if des1 is not None and des2 is not None:
    #    matches = bf.match(des1,des2)
        matches = flann.knnMatch(des1, des2, k=2)
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)
        print ("Points pan detected: ",len(kp1), " and ", len(kp2), " match: ",len(good))
  
    MIN_MATCH_COUNT = 38
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape
        pts = np.float32([[0,0],[0,h],[w,h],[w,0]]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts, M)
        img2 = cv2.polylines(img2, [np.int32(dst)],True,(128,121,111),3,cv2.LINE_AA) 
        #cv2.imshow('roi',img2)

        if np.shape(M) == ():
            print("No transformation matrix possible")
            exit()

        theta_pan =  -math.atan2(M[0,1],M[0,0]) + dlta_pan*math.pi/180 -math.pi/2
        print('-------theta---------', theta_pan*180/math.pi)
        point1_x = np.int32(dst)[0][0][0]
        point1_y = np.int32(dst)[0][0][1]
        point2_x = np.int32(dst)[1][0][0]
        point2_y = np.int32(dst)[1][0][1]
        point3_x = np.int32(dst)[2][0][0]
        point3_y = np.int32(dst)[2][0][1]
        point4_x = np.int32(dst)[3][0][0]
        point4_y = np.int32(dst)[3][0][1]
        center_x = point1_x + (point3_x - point1_x)/2
        center_y = point1_y + (point3_y - point1_y)/2

        draw_params = dict(matchColor=(0,255,0),singlePointColor=None,matchesMask = matchesMask,flags = 2)
        matching_frame = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)
        cv2.imshow('matching_pan',matching_frame) 
        panx,pany,pan_theta = center_x,center_y, theta_pan
        return center_x,center_y, theta_pan
    else:
        return panx, pany, pan_theta

net_platex,net_platey,net_plate_theta = 0,0,0
def func_bar_detection_net_plate(img2):
    global center_x
    global center_y
    global theta_net_plate
    global net_platex
    global net_platey
    global net_plate_theta
    img1 = cv2.imread('./samples/roiimgback/net_plate.png')
    cv2.imshow("back",img1)
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
   
    sift = cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    good = []
    #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)             
    if des1 is not None and des2 is not None:
    #    matches = bf.match(des1,des2)
        matches = flann.knnMatch(des1, des2, k=2)
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)
        print ("Points net_plate detected: ",len(kp1), " and ", len(kp2), " match: ",len(good))
    
    MIN_MATCH_COUNT = 35
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape
        pts = np.float32([[0,0],[0,h],[w,h],[w,0]]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts, M)
        img2 = cv2.polylines(img2, [np.int32(dst)],True,(128,121,111),3,cv2.LINE_AA) 
        #cv2.imshow('roi',img2)

        if np.shape(M) == ():
            print("No transformation matrix possible")
            exit()

        theta_net_plate = - math.atan2(M[0,1],M[0,0]) + dlta_net_plat*math.pi/180
        #print('theta  net plate',theta)
        point1_x = np.int32(dst)[0][0][0]
        point1_y = np.int32(dst)[0][0][1]
        point2_x = np.int32(dst)[1][0][0]
        point2_y = np.int32(dst)[1][0][1]
        point3_x = np.int32(dst)[2][0][0]
        point3_y = np.int32(dst)[2][0][1]
        point4_x = np.int32(dst)[3][0][0]
        point4_y = np.int32(dst)[3][0][1]
        center_x = point1_x + (point3_x - point1_x)/2
        center_y = point1_y + (point3_y - point1_y)/2

        draw_params = dict(matchColor=(0,255,0),singlePointColor=None,matchesMask = matchesMask,flags = 2)
        matching_frame = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)
        cv2.imshow('matching net_plate',matching_frame) 
        #cv2.destroyAllWindows()
        net_platex,net_platey,net_plate_theta = center_x,center_y, theta_net_plate
        return center_x,center_y, theta_net_plate
    else:
        return net_platex,net_platey,net_plate_theta

netx, nety, net_theta = 0,0,0
def func_bar_detection_net(img2):
    global center_x
    global center_y
    global theta_net
    global netx
    global nety
    global net_theta
    img1 = cv2.imread('./samples/roiimgback/net_roi.png')
    
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
   
    sift = cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    good = []
    #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)             
    if des1 is not None and des2 is not None:
    #    matches = bf.match(des1,des2)
        matches = flann.knnMatch(des1, des2, k=2)
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)
        print ("Points net detected: ",len(kp1), " and ", len(kp2), " match: ",len(good))
    
    #print("jinlaile!")

    
    MIN_MATCH_COUNT = 38
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape
        pts = np.float32([[0,0],[0,h],[w,h],[w,0]]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts, M)
        img2 = cv2.polylines(img2, [np.int32(dst)],True,(128,121,111),3,cv2.LINE_AA) 
        #cv2.imshow('roi',img2)

        if np.shape(M) == ():
            print("No transformation matrix possible")
            exit()

        #theta = - math.atan2(M[0,1],M[0,0])*180/math.pi
        theta_net = - math.atan2(M[0,1],M[0,0]) + dlta_net*math.pi/180
        print('theta net bar**--------',theta_net*180/math.pi)
        point1_x = np.int32(dst)[0][0][0]
        point1_y = np.int32(dst)[0][0][1]
        point2_x = np.int32(dst)[1][0][0]
        point2_y = np.int32(dst)[1][0][1]
        point3_x = np.int32(dst)[2][0][0]
        point3_y = np.int32(dst)[2][0][1]
        point4_x = np.int32(dst)[3][0][0]
        point4_y = np.int32(dst)[3][0][1]
        center_x = point1_x + (point3_x - point1_x)/2
        center_y = point1_y + (point3_y - point1_y)/2

        draw_params = dict(matchColor=(0,255,0),singlePointColor=None,matchesMask = matchesMask,flags = 2)
        matching_frame = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)
        cv2.imshow('matching net',matching_frame)
        netx, nety, net_theta =  center_x,center_y, theta_net
        return center_x,center_y, theta_net
    else:
        return netx, nety, net_theta
        
potx, poty, pot_theta = 0, 0, 0
def func_bar_detection_pot(img2):
    global center_x
    global center_y
    global theta_pot
    global potx
    global poty
    global pot_theta
    img1 = cv2.imread('./samples/roiimgback/pot_roi.png')
    cv2.imshow("back",img1)
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
   
    sift = cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    good = []
    #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)             
    if des1 is not None and des2 is not None:
    #    matches = bf.match(des1,des2)
        matches = flann.knnMatch(des1, des2, k=2)
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)
        print ("Points pot detected: ",len(kp1), " and ", len(kp2), " match: ",len(good))
  
    MIN_MATCH_COUNT = 38
    
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape
        pts = np.float32([[0,0],[0,h],[w,h],[w,0]]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts, M)
        img2 = cv2.polylines(img2, [np.int32(dst)],True,(128,121,111),3,cv2.LINE_AA) 
        #cv2.imshow('roi',img2)

        if np.shape(M) == ():
            print("No transformation matrix possible")
            exit()

        theta_pot = -math.atan2(M[0,1],M[0,0]) + dlta_pot*math.pi/180
        print('*****theta*****',theta_pot*180/math.pi)
        point1_x = np.int32(dst)[0][0][0]
        point1_y = np.int32(dst)[0][0][1]
        point2_x = np.int32(dst)[1][0][0]
        point2_y = np.int32(dst)[1][0][1]
        point3_x = np.int32(dst)[2][0][0]
        point3_y = np.int32(dst)[2][0][1]
        point4_x = np.int32(dst)[3][0][0]
        point4_y = np.int32(dst)[3][0][1]
        center_x = point1_x + (point3_x - point1_x)/2
        center_y = point1_y + (point3_y - point1_y)/2

        draw_params = dict(matchColor=(0,255,0),singlePointColor=None,matchesMask = matchesMask,flags = 2)
        matching_frame = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)
        cv2.imshow('matching pot',matching_frame) 
        potx, poty, pot_theta = center_x,center_y, theta_pot
        return center_x,center_y, theta_pot
    else:
        return potx, poty, pot_theta

temp_potx,temp_poty,temp_pottheta=0,0,0
temp_platex,temp_platey,temp_platetheta=0,0,0

def model_detection(img,cv_img):
    cv2.imshow("11111",img)
    temp_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #####################pan_bar######################
    
    img_rect1 = temp_img[pan_roi[1]:pan_roi[3], pan_roi[0]:pan_roi[2]]
    cv_img=cv2.rectangle(cv_img,(int(pan_roi[0]/2),int(pan_roi[1]/2)),(int (pan_roi[2]/2),int(pan_roi[3]/2)),(0,255,255),1)
    temp_x, temp_y, temp_theta = func_bar_detection_pan(img_rect1)
    #print("*********",temp_x, temp_y, temp_theta)
    bar_pose[0],bar_pose[1],bar_pose[2] = (temp_x+pan_roi[0])/2, (temp_y+pan_roi[1])/2, temp_theta
    print("1111111111111")
    print("pan pan bar: ",bar_pose[0],bar_pose[1],bar_pose[2]*180/math.pi)
    cv2.circle(cv_img, (np.int32(bar_pose[0]), np.int32(bar_pose[1])), 2, (0, 0, 255), 2, 8, 0)
    
    #####################cai_bar###################### 

    img_rect6 = temp_img[net_roi_plate[1]:net_roi_plate[3], net_roi_plate[0]:net_roi_plate[2]]
    cv_img=cv2.rectangle(cv_img,(int(net_roi_plate[0]/2),int(net_roi_plate[1]/2)),(int(net_roi_plate[2]/2),int(net_roi_plate[3]/2)),(0,255,255),1)
    temp_platex,temp_platey,temp_platetheta = func_bar_detection_net_plate(img_rect6)
    bar_pose[3],bar_pose[4],bar_pose[5] = int((temp_platex+net_roi_plate[0])/2), int((temp_platey+net_roi_plate[1])/2), temp_platetheta

    cv2.circle(cv_img, (np.int32(bar_pose[3]), np.int32(bar_pose[4])), 2, (0, 0, 255), 2, 8, 0)
    print("net plate bar: ",bar_pose[3],bar_pose[4],bar_pose[5]*180/math.pi)

    #####################pot_bar######################        
    img_rect3 = temp_img[pot_roi[1]:pot_roi[3], pot_roi[0]:pot_roi[2]]
    #img_out3[pot_roi[1]:pot_roi[3], pot_roi[0]:pot_roi[2]] = img_rect3.copy()
    cv_img=cv2.rectangle(cv_img,(int(pot_roi[0]/2),int(pot_roi[1]/2)),(int(pot_roi[2]/2),int(pot_roi[3]/2)),(0,255,255),1)
    temp2_x, temp2_y, temp2_theta = func_bar_detection_pot(img_rect3)
    bar_pose[6],bar_pose[7],bar_pose[8] = int((temp2_x+pot_roi[0])/2), int((temp2_y+pot_roi[1])/2), temp2_theta
   
    cv2.circle(cv_img, (np.int32(bar_pose[6]), np.int32(bar_pose[7])), 2, (0, 0, 255), 2, 8, 0)
    print("pot bar: ",bar_pose[6],bar_pose[7],bar_pose[8]*180/math.pi)
    
    #####################net_bar######################

    img_rect5 = temp_img[net_roi[1]:net_roi[3], net_roi[0]:net_roi[2]]
    cv_img=cv2.rectangle(cv_img,(int(net_roi[0]/2),int(net_roi[1]/2)),(int(net_roi[2]/2),int(net_roi[3]/2)),(0,255,255),1)
    cv2.imshow("imgrect5",img_rect5)
    temp_potx,temp_poty,temp_pottheta = func_bar_detection_net(img_rect5)
    bar_pose[9],bar_pose[10],bar_pose[11] = int((temp_potx+net_roi[0])/2), int((temp_poty+net_roi[1])/2), temp_pottheta
    cv2.circle(cv_img, (np.int32(bar_pose[9]), np.int32(bar_pose[10])), 2, (0, 0, 255), 2, 8, 0)
    print("net bar: ",bar_pose[9],bar_pose[10],bar_pose[11]*180/math.pi)
    #cv2.waitKey(1)
    return bar_pose

