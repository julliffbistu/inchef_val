#!/usr/bin/env python
from kitchen.msg import objs
from kitchen.msg import events

from kitchen.msg import listframes_calib
from kitchen.msg import bar_calib
from kitchen.msg import broc_calib
from kitchen.msg import sglframe_calib
from kitchen.msg import sglobject_calib
from kitchen.msg import listframes_luban


import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String

class LBFunc:
    def __init__(self, api_name, topic, data_class):
        self.api_name = api_name
        self.topic = topic
        self.data_class = (data_class)
        if data_class is None:
            raise ValueError("topic parameter 'data_class' is not initialized")
        if not type(data_class) == type:
            raise ValueError("data_class [%s] is not a class"%data_class)

api_list = [ #17 api
    LBFunc("checkenv","luban_bridge/checkenv", Bool), #check env,
    LBFunc("getbeefcenterpose","luban_bridge/getbeefcenterpose", Pose), # get beef center pose
    LBFunc("checkbeefdrop","luban_bridge/checkbeefdrop", Bool),# check beef dropped
    LBFunc("calib_maskrcnn","luban_bridge/calib_maskrcnn", listframes_luban),# maskrcnn calib output
    LBFunc("calib_bar","luban_bridge/calib_bar", bar_calib),# bar calib output
    LBFunc("calib_broc","luban_bridge/calib_broc", broc_calib)# broc calib output
   ]

class Bridge:

    def __init__(self):
        
        rospy.init_node('luban_bridger',anonymous = True)
        self.objects_sub = rospy.Subscriber("processor/objs",objs,self.callbackobjs)
        self.events_sub = rospy.Subscriber("processor/events",events,self.callbackevents)
        self.calib_mask_sub= rospy.Subscriber("/calibration/mask_multi_frame",listframes_calib,self.callback_calib_mask)
        self.calib_bar_sub= rospy.Subscriber("/calibration/bar",bar_calib,self.callback_calib_bar)
        self.calib_broc_sub= rospy.Subscriber("/calibration/broc",broc_calib,self.callback_calib_broc)

        self.pub_handle = {}
        print("init func list:")
        i = 0
        for al in (api_list):
            self.pub_handle[i] = rospy.Publisher(al.topic,al.data_class,queue_size=10)
            i =i+1

    def find_handler(self, name):
        i = 0
        for al in (api_list):
            if(al.api_name == name):
                return self.pub_handle[i] 
            i =i+1
        return NotImplementedError

    def callback_calib_mask(self,data):
       # print(data)
        listframes_luban_obj=listframes_luban()
        listframes_luban_obj.frames_info=data.frames_info
        listframes_luban_obj.header.stamp = data.header.stamp
        listframes_luban_obj.header.frame_id = data.header.frame_id
        listframes_luban_obj.rail_mask=data.rail_mask
        #print("listframes_luban_obj",listframes_luban_obj)

        self.pub_handle[3].publish(listframes_luban_obj)
        #print(self.pub_handle[3])
        print("listframes_luban_obj callback",listframes_luban_obj)


    def callback_calib_bar(self,data):
       # print(data)
        bar_calib_obj=bar_calib()
        bar_calib_obj.left_arm=data.left_arm
        bar_calib_obj.right_arm=data.right_arm
        bar_calib_obj.rail_bar=data.rail_bar
        bar_calib_obj.score_bar=data.score_bar
        #print("bar_calib_obj",bar_calib_obj)
        self.pub_handle[4].publish(bar_calib_obj)
        #print(self.pub_handle[4])
       # print("bar_calib_obj callback")
    def callback_calib_broc(self,data):
       # print(data)
        broc_calib_obj=broc_calib()
        broc_calib_obj.left_arm=data.left_arm
        broc_calib_obj.right_arm=data.right_arm
        broc_calib_obj.rail_broc=data.rail_broc
        broc_calib_obj.score_broc=data.score_broc
        #print("broc_calib_obj",broc_calib_obj)
        self.pub_handle[5].publish(broc_calib_obj)
       # print(self.pub_handle[5])
        #print("broc_calib_obj callback")


    def callbackobjs(self,objs):
        
        print("some callback")
        
    def callbackevents(self,events):
        if 'CameraReady' in events.events:
            bridge.find_handler("checkenv").publish(True)
        print("test callback")
    
if __name__ == '__main__':
    bridge = Bridge()
    try:
        #test
        bridge.find_handler("checkenv").publish(True)
        #while
        rospy.spin()
        print("luban_bridge node exit!")
    except KeyboardInterrupt:
        print("Shutting Down") 

