#!/usr/bin/env python
# coding: utf-8
import rospy
import roslib
from std_msgs.msg import String, Float32MultiArray,Int8
import numpy as np
import codecs
from kitchen.msg import listobj
from kitchen.msg import broclist



def listener():
    rospy.init_node("maskrcnn")
    rospy.Subscriber("/process_uv/listobjs",listobj,callback_masrcnn,queue_size=1)
    rospy.Subscriber("/process_uv/bar_pos",Float32MultiArray,callback_bar,queue_size=10)
    rospy.Subscriber("/process_uv/listbroc",broclist,callback_broc,queue_size=10)
    rospy.spin()

f = codecs.open("predict_uv.txt",'w','utf-8')
number = 1
def callback_masrcnn(data):
    global number
    
    cnt = 0
    label_list = data.single_obj_info[0].classname
    info_list = data.single_obj_info[0].element_data_temp
    score_list = data.single_obj_info[0].score
    #print("number:",number)
    #print("masrcnn classname:",labe_list)
    #print("masrcnn score:",info_list)
    #print("masrcnn element_data:",score_list)
    
    for i in range(len(label_list)):
        if label_list[i] == "beef":
            cnt = 1
            print("center point is: ",number, info_list[i].id_info[7:9])
            f.write(str(number) + ".jpg" + ',' + str(info_list[i].id_info[7]) + ',' + str(info_list[i].id_info[8]) + '\r\n')  #\r\n为换行符

    if cnt==0:
        f.write(str(number) + ".jpg" + ',' + str(0) + ',' + str(0) + '\r\n')  #\r\n为换行符
        print("center point is: ",number, (0,0))
    number = number + 1
    
def callback_bar(data):
    pass
    #print("callback_bar",data)

def callback_broc(data):
    pass
    #print("callback_broc",data)

if __name__ == '__main__':
    listener()
    f.close()





