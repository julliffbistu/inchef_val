#!/usr/bin/env python

import sys
import cv2
import math
import numpy as np
from numpy import *
from numpy.linalg import inv, qr,det
from math import sqrt





IMG_COL=960
IMG_ROW=540



class point_transformation:

    def __init__(self):
        self.position=0
        self.Quat=np.zeros(4,dtype=float)
        self.Translation=np.zeros([3,1],dtype=float)
        self.Camera_Internal_Mat=np.zeros([3,3],dtype=float)
    def quat_to_Rot(self):
        #print(self.Quat)
        qx=self.Quat[0]
        qy=self.Quat[1]
        qz=self.Quat[2]
        qw=self.Quat[3]
        rot=np.zeros([3,3])
        rot[0,0]=1-2*qy*qy-2*qz*qz
        rot[0,1]=2*qx*qy - 2*qz*qw
        rot[0,2]=2*qx*qz + 2*qy*qw
        rot[1,0]=2*qx*qy + 2*qz*qw
        rot[1,1]=1 - 2*qx*qx - 2*qz*qz
        rot[1,2]=2*qy*qz - 2*qx*qw
        rot[2,0]=2*qx*qz - 2*qy*qw
        rot[2,1]=2*qy*qz + 2*qx*qw
        rot[2,2]=1 - 2*qx*qx - 2*qy*qy 

        return rot
    def load_camera_internal_params(self,camera_internal_params):
        self.Camera_Internal_Mat=camera_internal_params
       # print("self.Camera_Internal_Mat",self.Camera_Internal_Mat)
    def load_position(self,Quat,Translation):
        self.Quat=Quat
        #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",position)
       # print("Quat",self.Quat)
        self.Translation=Translation
    
    def find_nonzeros_pixelvalue(self,img,v,u):
        count=0
        sums=0
        if (v>IMG_ROW-2)|(u>IMG_COL-2)|(u<2)|(v<2):
            print(u)
            print(v)
            print("out img range")
            return(0)
        else:
            if (img[v,u]!=0):
                return(img[v,u])  
                print(img[v,u])
            else:
                for i in range(-1,2,1):
                    for j in range(-1,2,1):
                        if img[v+i,u+j]!=0:
                            count=count+1
                            sums=sums+img[v+i,u+j]
                if count==0:
                    print("3*3 pixels all zeros,bad point")
                    return(0)
                else:
                    return(sums/count)     
     
    def Pix2baselink(self,depth_img,u,v):
       # print("x,y",x,y)


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
        
        #print(self.depth_img)
        pixposition=np.ones((3,1))
        pixposition[0]=u
        pixposition[1]=v

        Camera_External_RotMat  = self.quat_to_Rot()
        baselink2base=np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        #s=self.find_nonzeros_pixelvalue(depth_img,v,u)
        s=depth_img[v,u]*0.001 
        if s==0:
            s=1.01
        #print("sssssssssssssssssss",s)
        Camera_Mat_inv=np.linalg.inv(self.Camera_Internal_Mat)
        cameraPoint=(s*Camera_Mat_inv).dot(pixposition)
        worldPoint_3d = Camera_External_RotMat.dot(cameraPoint)+self.Translation
        worldPoint_3d=baselink2base.dot(worldPoint_3d)
       # print(",,,worldPoint_3d,,,,,,",worldPoint_3d)
        return worldPoint_3d[0],worldPoint_3d[1],worldPoint_3d[2]

    def Pix2baselink_points(self,u,v,d):
       # print("x,y",x,y)


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
        
        #print(self.depth_img)
        pixposition=np.ones((3,1))
        pixposition[0]=u
        pixposition[1]=v

        Camera_External_RotMat  = self.quat_to_Rot()
        baselink2base=np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        #s=self.find_nonzeros_pixelvalue(depth_img,v,u)
        s=d*0.001 
        if s==0:
            s=1.01
        #print("sssssssssssssssssss",s)
        Camera_Mat_inv=np.linalg.inv(self.Camera_Internal_Mat)
        cameraPoint=(s*Camera_Mat_inv).dot(pixposition)
        worldPoint_3d = Camera_External_RotMat.dot(cameraPoint)+self.Translation
        worldPoint_3d=baselink2base.dot(worldPoint_3d)
       # print(",,,worldPoint_3d,,,,,,",worldPoint_3d)
        return worldPoint_3d[0],worldPoint_3d[1],worldPoint_3d[2]

    def rpy2rotvec(Rx,Ry,Rz):
        Rx=Rx/180*math.pi
        Ry=Ry/180*math.pi
        Rz=Rz/180*math.pi
        Txyz=np.array([[math.cos(Rz)*math.cos(Ry),math.cos(Rz)*math.sin(Ry)*math.sin(Rx)-math.sin(Rz)*math.cos(Rx),math.cos(Rz)*math.sin(Ry)*math.cos(Rx)+math.sin(Rz)*math.sin(Rx)],[math.sin(Rz)*math.cos(Ry),math.sin(Rz)*math.sin(Ry)*math.sin(Rx)+math.cos(Rz)*math.cos(Rx),math.sin(Rz)*math.sin(Ry)*math.cos(Rx)-math.cos(Rz)*math.sin(Rx)],[-math.sin(Ry),math.cos(Ry)*math.sin(Rx),math.cos(Ry)*math.cos(Rx)]])
        p=math.acos((Txyz[0,0]+Txyz[1,1]+Txyz[2,2]-1)/2)
        kx=(1/(2*math.sin(p)))*(Txyz[2,1]-Txyz[1,2])*p
        ky=(1/(2*math.sin(p)))*(Txyz[0,2]-Txyz[2,0])*p
        kz=(1/(2*math.sin(p)))*(Txyz[1,0]-Txyz[0,1])*p

        return kx,ky,kz

def findcalibparams(raillist):
    Quat=np.zeros(4,dtype=float)
    Translation=np.zeros([3,1],dtype=float)
    rail_h_param=raillist[0]
    rail_l_param=raillist[1]
    rail_r_param=raillist[2]
    for i in range(len(params_calib)):
        dicttemp=params_calib[i]
        rail_h=dicttemp['rail']['rail_h']
        rail_l=dicttemp['rail']['rail_l']
        rail_r=dicttemp['rail']['rail_r']
        print(dicttemp['rail'])
        if (abs(rail_h_param-rail_h)<0.01) and (abs(rail_l_param-rail_l)<0.01)and (abs(rail_r_param-rail_r)<0.01):
            print(dicttemp)
            Quat[0]=dicttemp['rotation']['x']
            Quat[1]=dicttemp['rotation']['y']
            Quat[2]=dicttemp['rotation']['z']
            Quat[3]=dicttemp['rotation']['w']
            Translation[0][0]=dicttemp['translation']['x']
            Translation[1][0]=dicttemp['translation']['y']
            Translation[2][0]=dicttemp['translation']['z']
    return Quat,Translation
'''
if __name__ == '__main__':
    
    calibration = point_transformation()


    #out.release()
    cv2.destroyAllWindows()
    raillist=[1,0,0]
    Quat=np.zeros(4,dtype=float)
    Translation=np.zeros([3,1],dtype=float)
    Quat,Translation=findcalibparams(raillist)
    print("Quat",Quat)
    print("Translation",Translation)
'''
