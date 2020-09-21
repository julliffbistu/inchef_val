# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 19:01:31 2020

@author: lifu_zhu
"""

import codecs
import numpy as np
import math

dis_thread = 35 #pixel
detect_count = 0

GT = codecs.open('grondtruth_beef_uv.txt', mode='r', encoding='utf-8')  # 打开txt文件，以‘utf-8’编码读取
PR = codecs.open('predict_beef_uv.txt', mode='r', encoding='utf-8')  # 打开txt文件，以‘utf-8’编码读取
line_GT = GT.readline()   # 以行的形式进行读取文件
line_PR = PR.readline()   # 以行的形式进行读取文件

list_GT_u = []
list_GT_v = []
list_PR_u = []
list_PR_v = []

while line_GT:
    a = line_GT.split(",")
    b = a[1]   # 这是选取需要读取的位数
    c = a[2]
    list_GT_u.append(b)  # 将其添加在列表之中
    list_GT_v.append(c)
    line_GT = GT.readline()

while line_PR:
    a1 = line_PR.split(",")
    b1 = a1[1]   # 这是选取需要读取的位数
    c1 = a1[2]
    list_PR_u.append(b1)  # 将其添加在列表之中
    list_PR_v.append(c1)
    line_PR = PR.readline()
    
GT.close()
PR.close()

if len(list_GT_u) == len(list_PR_u):
    print("two list same!")
    
    for i in range(len(list_GT_u)):
        #print(int(list_GT_u[i]),int(list_GT_v[i]))
        #print(int(list_PR_u[i]),int(list_PR_v[i]))
        
        dis_temp = math.pow((int(list_PR_u[i])-int(list_GT_u[i])),2) + math.pow((int(list_PR_v[i])-int(list_GT_v[i])),2)
        if dis_temp < math.pow(dis_thread,2):
            detect_count = detect_count + 1
        else:
            print("dectection error point is: %d.jpg" %i)
            print("dectection error point is:",[int(list_PR_u[i]),int(list_PR_v[i])], [int(list_GT_u[i]),int(list_GT_v[i])])

    percent = detect_count*1.0/len(list_GT_u)*100.0

    print("dectection right count is:",detect_count)
    print("dectection image count is:",len(list_GT_u))
    print("dectection rate is: %.4f%%" %percent)
    print("finished caculatinon! ")
else:
    print("error lenght is not same!")
    