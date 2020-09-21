# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 09:29:11 2020

@author: lifu_zhu
"""

import cv2
import codecs
import numpy as np

f = codecs.open("grondtruth_uv.txt",'w','utf-8')

#a = []
#b = []
a=0
b=0
i=1

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    global a
    global b
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        #a.append(x)
        #b.append(y)
        a=x
        b=y
        cv2.circle(img, (x, y), 4, (0, 0, 255), thickness=-1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0, 0, 0), thickness=1)
        cv2.imshow("image", img)
        
while 1:
    
    img = cv2.imread('./test_datasets/' + str(i) + '.jpg')
    
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
    cv2.imshow("image", img)
    if cv2.waitKey(0) & 0xFF == ord('s'):
        print("img id: ",i)
        print("pic point: ",a,b)
        f.write(str(i) + ".jpg" + ',' + str(a) + ',' + str(b) + '\r\n')  #\r\n为换行符
        cv2.waitKey(1)
        i=i+1
    
    if cv2.waitKey(0) & 0xFF == ord('q'):      
        f.close()
        break
