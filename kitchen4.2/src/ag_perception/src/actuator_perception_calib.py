#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import os
import time
import json
import threading
# import numpy as np
import glob
import collections
import rospy
import math
abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../../lib/comm")
sys.path.append(abs_file + "/../../../lib/log")
ROOT_DIR = sys.path.append(abs_file + "/../../../src/coco-Mask_RCNN/samples/")
from actuator import Actuator
from actuator import ErrorInfo
from actuator import ActuatorCmdType
from proxy_client import PS_Socket
from std_msgs.msg import Float32MultiArray
from kitchen.msg import listframes_luban
from kitchen.msg import bar_calib
from kitchen.msg import broc_calib
from kitchen.msg import sglframe_calib
from kitchen.msg import sglobject_calib
from kitchen.msg import listframes_calib
from kitchen.msg import rail_cmd_algo
from rlog import rlog
import numpy as np
log = rlog()

# define rail pose topic name
RAILH_POSEINFO = "ag_peripheral:chefrail_H:poseinfo"
RAILL_POSEINFO = "ag_peripheral:chefrail_L:poseinfo"
RAILR_POSEINFO = "ag_peripheral:chefrail_R:poseinfo"
POWEROFF_TOPIC = "ag_peripheral:hwmanager:poweroff"

# Define Error code
MOD_ERR_NUM = 3600
MOD_ERR_SELF_OFFSET = 20
E_OK = 0
E_MOD_PARAM = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 1
E_MOD_STATUS = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 2
E_MOD_DRIVER = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 3
E_MOD_EXCEPTION = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 5
E_MOD_ABORT_FAILED = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 6
E_MOD_GRAPS_FAILED = 0

FEATURE_NUMS=6
HEIGHT_LIMITED_LOW=0.35
HEIGHT_LIMITED_HIGH=0.65
left_pose_list=['beef_center_pose','beef_grasp_pose_left','beef_grasp_pose_right','beef_press_pose_one','beef_press_pose_two','beef_press_pose_three',
                'beef_press_pose_four','pan_center_pose','pot_center_pose','pan_bar_pose','cai_bar_pose','pot_bar_pose',
                'net_bar_pose','plate1_pose','plate2_pose','plate3_pose','seasoningbottle_left_pose','seasoningbottle_right_pose','broc_pose']
right_pose_list=['beef_center_pose','beef_grasp_pose_left','beef_grasp_pose_right','beef_press_pose_one','beef_press_pose_two','beef_press_pose_three',
                'beef_press_pose_four','pan_center_pose','pot_center_pose','pan_bar_pose','cai_bar_pose','pot_bar_pose',
                'net_bar_pose','plate1_pose','plate2_pose','plate3_pose','seasoningbottle_left_pose','seasoningbottle_right_pose','broc_pose']
rail_position_list=['rail_broc','rail_mask','rail_bar']
# Define status
STATUS_UNINIT = 'uninitialized'
STATUS_IDLE = 'idle'
STATUS_BUSY = 'busy'
STATUS_ERROR = 'error'

# command description dict
cmd_description_dict = {
    'cmddescribe': {
        'version': '0.0.1',
        'date': '20200323',
        'time': '11:06:25',
    },
    'cmdlist': [
        {
            'cmd': 'StartMaskrcnn',
            'atype': 'motion',
            'params':[],
            'return':{
                'type': 'bool'
            }
        },
        {
            'cmd': 'StopMaskrcnn',
            'atype': 'motion',
            'params':[],
            'return':{
                'type': 'bool'
            }
        },
        {
            'cmd': 'beef_leftarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'grapslist',
                    'type': 'string',
                    'default': 'beef_grasp_pose_right',
                    'listlimit': ['beef_grasp_pose_left', 'beef_grasp_pose_right',
                    'beef_center_pose','beef_press_pose_one','beef_press_pose_two',
                    'beef_press_pose_three','beef_press_pose_four']
                },
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'beef_rightarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'grapslist',
                    'type': 'string',
                    'default': 'beef_grasp_pose_right',
                    'listlimit': ['beef_grasp_pose_left', 'beef_grasp_pose_right',
                    'beef_center_pose','beef_press_pose_one','beef_press_pose_two',
                    'beef_press_pose_three','beef_press_pose_four']
                },
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'pan_center_pose_leftarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'pan_center_pose_rightarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'plate_leftarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'grapslist',
                    'type': 'string',
                    'default': 'plate1_pose',
                    'listlimit': ['plate1_pose', 'plate2_pose','plate3_pose']
                },
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'plate_rightarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'grapslist',
                    'type': 'string',
                    'default': 'plate1_pose',
                    'listlimit': ['plate1_pose', 'plate2_pose','plate3_pose']
                },
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'bar_leftarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'grapslist',
                    'type': 'string',
                    'default': 'pan_bar_pose',
                    'listlimit': ['pan_bar_pose', 'cai_bar_pose','pot_bar_pose','net_bar_pose']
                },
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'bar_rightarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'grapslist',
                    'type': 'string',
                    'default': 'pan_bar_pose',
                    'listlimit': ['pan_bar_pose', 'cai_bar_pose','pot_bar_pose','net_bar_pose']
                },
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'broc_leftarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'broc_rightarm',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'beef_wl',
            'atype': 'sensing',
            'params':[
                {
                    'name': 'size',
                    'type': 'string',
                    'default': 'beef_witdh',
                    'listlimit': ['beef_witdh', 'beef_length']
                },
                {
                    'name': 'timeout',
                    'type': 'float',
                    'default': 2,
                    'unit': 's'
                }
            ],
            'return':{
                'type': 'float'
            }
        },
        {
            'cmd': 'rail_cmd',
            'atype': 'motion',
            'params':[
                {
                    'name': 'arm',
                    'type': 'string',
                    'default': 'left',
                    'listlimit': ['left', 'right']
                },
                {
                    'name': 'hrail',
                    'type': 'float',
                    'default': 0,
                    'unit': 'm'
                }
            ]
        }
    ]
}

def get_dict_key_value(dict_ins, key, value_type):
    if key in dict_ins:
        value = dict_ins.get(key)
        if isinstance(value, value_type) is False:
            value = None
    else:
        value = None
    return value

class ActuatorPerceptionCalib(Actuator):
    def __init__(self, name, is_simulation, proxy_name, proxy_ip):
        Actuator.__init__(self, name)

        self.proxy_name_ = proxy_name
        self.is_simulation_ = is_simulation
        self.data_condition_ = threading.Condition()
        self.bar_condition_ = threading.Condition()
        self.broc_condition_ = threading.Condition()
        self.obj_condition_ = threading.Condition()
        self.proxy_ip = proxy_ip
        self.status_ = STATUS_IDLE
        self.set_status(STATUS_IDLE)
        self.statuscode_ = E_OK
        self.railparams=np.zeros(3,dtype=float)
        self.rail_cmd=np.zeros(3,dtype=float)
        self.rail_cmd[0] = 0.3
        self.rail_stamp = None
        self.rail_stamp_secs = 0
        self.rail_stamp_nsecs = 0
        self.rail_condition_pub = rospy.Publisher('/rail/position_temp', Float32MultiArray, queue_size=20)
        self.rail_cmd_pub = rospy.Publisher('/rail/position_cmd', rail_cmd_algo, queue_size=20)
        self.left_pose_dict={}
        self.right_pose_dict={}
        self.rail_position_dict={}
        self.img_stamp_secs=0
        self.img_stamp_nsecs=0

        for i in range(len(left_pose_list)):
            self.left_pose_dict[left_pose_list[i]]=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        for i in range(len(right_pose_list)):
            self.right_pose_dict[right_pose_list[i]]=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        for i in range(len(rail_position_list)):
            self.rail_position_dict[rail_position_list[i]]=[0.0,0.0,0.0]
        self.beef_witdh=0.0
        self.beef_length=0.0
        self.start_maskrcnn = False
        self.stop_maskrcnn = False
        #print("self.pose_dict",self.)

        # connect to proxy
        self.pub_socket = PS_Socket(self.proxy_ip)
        self.sub_socket = PS_Socket(self.proxy_ip, self.luban_callback, self)
        self.sub_topics = [POWEROFF_TOPIC, RAILH_POSEINFO, RAILL_POSEINFO, RAILR_POSEINFO]
        self.sub_socket.subscribe(self.sub_topics)

        if self.is_simulation_ is False:
            self.bar_sub = rospy.Subscriber("/luban_bridge/calib_bar",bar_calib,self.bar_calib_callback)
            self.broc_sub = rospy.Subscriber("/luban_bridge/calib_broc",broc_calib,self.broc_calib_callback)
            self.obj_sub = rospy.Subscriber("/luban_bridge/calib_maskrcnn",listframes_luban,self.listframes_luban_callback)
            self.bar_topic_time = 0
            self.broc_topic_time = 0
            self.obj_topic_time = 0

    def height_limited(self,height,mode):
        statu_temp=1
        if mode==True:
            if (height<HEIGHT_LIMITED_LOW) or (height>HEIGHT_LIMITED_HIGH):
                statu_temp=0
        return statu_temp

    def bar_calib_callback(self,data):
        with self.bar_condition_:
            bar_left_arm=data.left_arm
            bar_right_arm=data.right_arm
            scores=data.score_bar
            self.rail_position_dict['rail_bar']=data.rail_bar
            log.notice("rail_bar position: {}".format(self.rail_position_dict['rail_bar']))

            #left_arm
            self.left_pose_dict['pan_bar_pose'][1:7]=bar_left_arm[0:6]
            self.left_pose_dict['pan_bar_pose'][0]=scores[0]*self.height_limited(self.left_pose_dict['pan_bar_pose'][3],True)
            log.notice("left_arm： pan_bar_pose: {}，score: {}".format(self.left_pose_dict['pan_bar_pose'][1:7],self.left_pose_dict['pan_bar_pose'][0]))



            self.left_pose_dict['cai_bar_pose'][1:7]=bar_left_arm[6:12]
            self.left_pose_dict['cai_bar_pose'][0]=scores[1]*self.height_limited(self.left_pose_dict['cai_bar_pose'][3],True)
            log.notice("left_arm： cai_bar_pose: {}，score: {}".format(self.left_pose_dict['cai_bar_pose'][1:7],self.left_pose_dict['cai_bar_pose'][0]))

            self.left_pose_dict['pot_bar_pose'][1:7]=bar_left_arm[12:18]
            self.left_pose_dict['pot_bar_pose'][0]=scores[2]*self.height_limited(self.left_pose_dict['pot_bar_pose'][3],True)
            log.notice("left_arm： pot_bar_pose: {}，score: {}".format(self.left_pose_dict['pot_bar_pose'][1:7],self.left_pose_dict['pot_bar_pose'][0]))


            self.left_pose_dict['net_bar_pose'][1:7]=bar_left_arm[18:24]
            self.left_pose_dict['net_bar_pose'][0]=scores[3]*self.height_limited(self.left_pose_dict['net_bar_pose'][3],True)
            log.notice("left_arm： net_bar_pose: {}，score: {}".format(self.left_pose_dict['net_bar_pose'][1:7],self.left_pose_dict['net_bar_pose'][0]))

            #right_arm
            self.right_pose_dict['pan_bar_pose'][1:7]=bar_right_arm[0:6]
            self.right_pose_dict['pan_bar_pose'][0]=scores[0]*self.height_limited(self.right_pose_dict['pan_bar_pose'][3],True)
            log.notice("right_arm: pan_bar_pose: {}，score: {}".format(self.right_pose_dict['pan_bar_pose'][1:7],self.right_pose_dict['pan_bar_pose'][0]))


            self.right_pose_dict['cai_bar_pose'][1:7]=bar_right_arm[6:12]
            self.right_pose_dict['cai_bar_pose'][0]=scores[1]*self.height_limited(self.right_pose_dict['cai_bar_pose'][3],True)
            log.notice("right_arm: cai_bar_pose: {}，score: {}".format(self.right_pose_dict['cai_bar_pose'][1:7],self.right_pose_dict['cai_bar_pose'][0]))


            self.right_pose_dict['pot_bar_pose'][1:7]=bar_right_arm[12:18]
            self.right_pose_dict['pot_bar_pose'][0]=scores[2]*self.height_limited(self.right_pose_dict['pot_bar_pose'][3],True)
            log.notice("right_arm: pot_bar_pose: {}，score: {}".format(self.right_pose_dict['pot_bar_pose'][1:7],self.right_pose_dict['pot_bar_pose'][0]))

            self.right_pose_dict['net_bar_pose'][1:7]=bar_right_arm[18:24]
            self.right_pose_dict['net_bar_pose'][0]=scores[3]*self.height_limited(self.right_pose_dict['net_bar_pose'][3],True)
            log.notice("right_arm: net_bar_pose: {}，score: {}".format(self.right_pose_dict['net_bar_pose'][1:7],self.right_pose_dict['net_bar_pose'][0]))
        self.bar_topic_time = rospy.get_time()

    def broc_calib_callback(self,data):
        with self.broc_condition_:
            broc_left_arm=data.left_arm
            broc_right_arm=data.right_arm
            scores=data.score_broc
            self.rail_position_dict['rail_broc']=data.rail_broc
            log.notice("rail_broc position: {}".format(self.rail_position_dict['rail_broc']))
            #left_arm
            if len(broc_left_arm)>0:
                self.left_pose_dict['broc_pose'][1:7]=broc_left_arm[0:6]
                self.left_pose_dict['broc_pose'][0]=scores[0]*self.height_limited(self.left_pose_dict['broc_pose'][3],True)
                log.notice("left_arm： broc_pose: {}，score: {}".format(self.left_pose_dict['broc_pose'][1:7],self.left_pose_dict['broc_pose'][0]))
            else:
                self.left_pose_dict['broc_pose'][0]=0.0
                log.notice("left_arm： broc_pose: {}，score: {}".format(self.left_pose_dict['broc_pose'][1:7],self.left_pose_dict['broc_pose'][0]))

            if len(broc_right_arm)>0:
                self.right_pose_dict['broc_pose'][1:7]=broc_right_arm[0:6]
                self.right_pose_dict['broc_pose'][0]=scores[0]*self.height_limited(self.right_pose_dict['broc_pose'][3],True)
                log.notice("right_arm： broc_pose: {}，score: {}".format(self.right_pose_dict['broc_pose'][1:7],self.right_pose_dict['broc_pose'][0]))
            else:
                self.right_pose_dict['broc_pose'][0]=0
                log.notice("right_arm： broc_pose: {}，score: {}".format(self.right_pose_dict['broc_pose'][1:7],self.right_pose_dict['broc_pose'][0]))
        self.broc_topic_time = rospy.get_time()

    def listframes_luban_callback(self,data):
        with self.obj_condition_:
            classname=data.frames_info[0].classname
            score=data.frames_info[0].score
            feature_info_left=data.frames_info[0].feature_info_left
            feature_info_right=data.frames_info[0].feature_info_right
            self.rail_position_dict['rail_mask']=data.rail_mask
            self.img_stamp_secs=data.header.stamp.secs
            self.img_stamp_nsecs=data.header.stamp.nsecs
            log.notice("rail_mask position: {}".format(self.rail_position_dict['rail_mask']))
            for i in range(len(classname)):
                if classname[i]=='beef':
                    self.left_pose_dict['beef_grasp_pose_left'][1:7]=feature_info_left[i].features[0:6]
                    self.left_pose_dict['beef_grasp_pose_left'][0]=score[i]*self.height_limited(self.left_pose_dict['beef_grasp_pose_left'][3],True)
                    self.left_pose_dict['beef_grasp_pose_right'][1:7]=feature_info_left[i].features[6:12]
                    self.left_pose_dict['beef_grasp_pose_right'][0]=score[i]*self.height_limited(self.left_pose_dict['beef_grasp_pose_right'][3],True)
                    self.left_pose_dict['beef_center_pose'][1:7]=feature_info_left[i].features[12:18]
                    self.left_pose_dict['beef_center_pose'][0]=score[i]*self.height_limited(self.left_pose_dict['beef_center_pose'][3],True)
                    self.left_pose_dict['beef_press_pose_one'][1:7]=feature_info_left[i].features[18:24]
                    self.left_pose_dict['beef_press_pose_one'][0]=score[i]*self.height_limited(self.left_pose_dict['beef_press_pose_one'][3],True)
                    self.left_pose_dict['beef_press_pose_two'][1:7]=feature_info_left[i].features[24:30]
                    self.left_pose_dict['beef_press_pose_two'][0]=score[i]*self.height_limited(self.left_pose_dict['beef_press_pose_two'][3],True)
                    self.left_pose_dict['beef_press_pose_three'][1:7]=feature_info_left[i].features[30:36]
                    self.left_pose_dict['beef_press_pose_three'][0]=score[i]*self.height_limited(self.left_pose_dict['beef_press_pose_three'][3],True)
                    self.left_pose_dict['beef_press_pose_four'][1:7]=feature_info_left[i].features[36:42]
                    self.left_pose_dict['beef_press_pose_four'][0]=score[i]*self.height_limited(self.left_pose_dict['beef_press_pose_four'][3],True)
                    log.notice("left_arm： beef_grasp_pose_left: {}，score: {}".format(self.left_pose_dict['beef_grasp_pose_left'][1:7],self.left_pose_dict['beef_grasp_pose_left'][0]))
                    log.notice("left_arm： beef_grasp_pose_right: {}，score: {}".format(self.left_pose_dict['beef_grasp_pose_right'][1:7],self.left_pose_dict['beef_grasp_pose_right'][0]))
                    log.notice("left_arm： beef_center_pose: {}，score: {}".format(self.left_pose_dict['beef_center_pose'][1:7],self.left_pose_dict['beef_center_pose'][0]))

                    self.right_pose_dict['beef_grasp_pose_left'][1:7]=feature_info_right[i].features[0:6]
                    self.right_pose_dict['beef_grasp_pose_left'][0]=score[i]*self.height_limited(self.right_pose_dict['beef_grasp_pose_left'][3],True)
                    self.right_pose_dict['beef_grasp_pose_right'][1:7]=feature_info_right[i].features[6:12]
                    self.right_pose_dict['beef_grasp_pose_right'][0]=score[i]*self.height_limited(self.right_pose_dict['beef_grasp_pose_right'][3],True)
                    self.right_pose_dict['beef_center_pose'][1:7]=feature_info_right[i].features[12:18]
                    self.right_pose_dict['beef_center_pose'][0]=score[i]*self.height_limited(self.right_pose_dict['beef_center_pose'][3],True)
                    self.right_pose_dict['beef_press_pose_one'][1:7]=feature_info_right[i].features[18:24]
                    self.right_pose_dict['beef_press_pose_one'][0]=score[i]*self.height_limited(self.right_pose_dict['beef_press_pose_one'][3],True)
                    self.right_pose_dict['beef_press_pose_two'][1:7]=feature_info_right[i].features[24:30]
                    self.right_pose_dict['beef_press_pose_two'][0]=score[i]*self.height_limited(self.right_pose_dict['beef_press_pose_two'][3],True)
                    self.right_pose_dict['beef_press_pose_three'][1:7]=feature_info_right[i].features[30:36]
                    self.right_pose_dict['beef_press_pose_three'][0]=score[i]*self.height_limited(self.right_pose_dict['beef_press_pose_three'][3],True)
                    self.right_pose_dict['beef_press_pose_four'][1:7]=feature_info_right[i].features[36:42]
                    self.right_pose_dict['beef_press_pose_four'][0]=score[i]*self.height_limited(self.right_pose_dict['beef_press_pose_four'][3],True)

                    self.beef_witdh=feature_info_right[i].features[42]
                    self.beef_length=feature_info_right[i].features[43]
                    log.notice("right_arm： beef_grasp_pose_left: {}，score: {}".format(self.right_pose_dict['beef_grasp_pose_left'][1:7],self.right_pose_dict['beef_grasp_pose_left'][0]))
                    log.notice("right_arm： beef_grasp_pose_right: {}，score: {}".format(self.right_pose_dict['beef_grasp_pose_right'][1:7],self.right_pose_dict['beef_grasp_pose_right'][0]))
                    log.notice("right_arm： beef_center_pose: {}，score: {}".format(self.right_pose_dict['beef_center_pose'][1:7],self.right_pose_dict['beef_center_pose'][0]))
                    log.notice("right_arm： beef_witdh: {}，beef_length: {},score: {}".format(self.beef_witdh,self.beef_length,self.right_pose_dict['beef_center_pose'][0]))

                elif classname[i]=='pan':

                    self.left_pose_dict['pan_center_pose'][1:7]=feature_info_left[i].features[12:18]
                    self.left_pose_dict['pan_center_pose'][0]=score[i]*self.height_limited(self.left_pose_dict['pan_center_pose'][3],True)
                    log.notice("left_arm： pan_center_pose: {}，score: {}".format(self.left_pose_dict['pan_center_pose'][1:7],self.left_pose_dict['pan_center_pose'][0]))

                    self.right_pose_dict['pan_center_pose'][1:7]=feature_info_right[i].features[12:18]
                    self.right_pose_dict['pan_center_pose'][0]=score[i]*self.height_limited(self.right_pose_dict['pan_center_pose'][3],True)
                    log.notice("right_arm： pan_center_pose: {}，score: {}".format(self.right_pose_dict['pan_center_pose'][1:7],self.right_pose_dict['pan_center_pose'][0]))
                elif classname[i]=='plate1':
                    self.left_pose_dict['plate1_pose'][1:7]=feature_info_left[i].features[12:18]
                    self.left_pose_dict['plate1_pose'][0]=score[i]*self.height_limited(self.left_pose_dict['plate1_pose'][3],True)
                    log.notice("left_arm： plate1_pose: {}，score: {}".format(self.left_pose_dict['plate1_pose'][1:7],self.left_pose_dict['plate1_pose'][0]))

                    self.right_pose_dict['plate1_pose'][1:7]=feature_info_right[i].features[12:18]
                    self.right_pose_dict['plate1_pose'][0]=score[i]*self.height_limited(self.right_pose_dict['plate1_pose'][3],True)
                    log.notice("right_arm： plate1_pose: {}，score: {}".format(self.right_pose_dict['plate1_pose'][1:7],self.right_pose_dict['plate1_pose'][0]))
                elif classname[i]=='plate2':

                    self.left_pose_dict['plate2_pose'][1:7]=feature_info_left[i].features[12:18]
                    self.left_pose_dict['plate2_pose'][0]=score[i]*self.height_limited(self.left_pose_dict['plate2_pose'][3],True)
                    log.notice("left_arm： plate2_pose: {}，score: {}".format(self.left_pose_dict['plate2_pose'][1:7],self.left_pose_dict['plate2_pose'][0]))

                    self.right_pose_dict['plate2_pose'][1:7]=feature_info_right[i].features[12:18]
                    self.right_pose_dict['plate2_pose'][0]=score[i]*self.height_limited(self.right_pose_dict['plate2_pose'][3],True)
                    log.notice("right_arm： plate2_pose: {}，score: {}".format(self.right_pose_dict['plate2_pose'][1:7],self.right_pose_dict['plate2_pose'][0]))

                elif classname[i]=='plate3':

                    self.left_pose_dict['plate3_pose'][1:7]=feature_info_left[i].features[12:18]
                    self.left_pose_dict['plate3_pose'][0]=score[i]*self.height_limited(self.left_pose_dict['plate3_pose'][3],True)
                    log.notice("left_arm： plate3_pose: {}，score: {}".format(self.left_pose_dict['plate3_pose'][1:7],self.left_pose_dict['plate3_pose'][0]))

                    self.right_pose_dict['plate3_pose'][1:7]=feature_info_right[i].features[12:18]
                    self.right_pose_dict['plate3_pose'][0]=score[i]*self.height_limited(self.right_pose_dict['plate3_pose'][3],True)
                    log.notice("right_arm： plate3_pose: {}，score: {}".format(self.right_pose_dict['plate3_pose'][1:7],self.right_pose_dict['plate3_pose'][0]))
        self.obj_topic_time = rospy.get_time()

    def luban_callback(self, caller_args, topic, content):
        log.debug("luban_callback: topic: {}, content: {}".format(topic, content))
        if topic == POWEROFF_TOPIC:
            log.notice("luban_callback: topic: {}, content: {}".format(topic, content))
            os.system('shutdown -h now')
        else:
            content_dic = json.loads(content)
            if content_dic is not None:
                status = get_dict_key_value(content_dic, 'status', (str, unicode))
                if (status is not None) and (status == "run"):
                    pos = get_dict_key_value(content_dic, 'pos', float)
                    if pos is not None:
                        if topic == RAILH_POSEINFO:
                            self.railparams[0] = pos
                            self.rail_cmd[0] = pos
                        elif topic == RAILL_POSEINFO:
                            self.railparams[1] = pos
                        elif topic == RAILR_POSEINFO:
                            self.railparams[2] = pos
                        rail_pos = Float32MultiArray(data=self.railparams)
                        self.rail_condition_pub.publish(rail_pos)
                        log.debug("luban_callback: ros topic publish rail pos: {}".format(self.railparams))

    def cmd_rail_publish(self):
        rail_cmd_obj=rail_cmd_algo()
        rail_cmd_obj.header.stamp = self.rail_stamp
        rail_cmd_obj.header.frame_id = "map"
        rail_cmd_obj.rail_pos = self.rail_cmd
        self.rail_cmd_pub.publish(rail_cmd_obj)

    def rail_cmp(self, arm, rail_cmd, rail_res):
        ret_val = True
        if (abs(rail_cmd[0] - rail_res[0]) > 0.01):
            ret_val = False
        elif (arm == 'left'):
            if (abs(rail_cmd[1] - rail_res[1]) > 0.01):
                ret_val = False
        elif (arm == 'right') :
            if (abs(rail_cmd[2] - rail_res[2]) > 0.01):
                ret_val = False
        if ret_val == False:
            log.notice("rail_res is not equal to rail_cmd, rail_res: {}, rail_cmd: {}".format(rail_res, rail_cmd))
        return ret_val

    def rail_stamp_cmp(self):
        ret_val = True
        log.notice("rail_stamp_cmp: rail_stamp_secs: {}, rail_stamp_nsecs: {}, img_stamp_secs: {}, img_stamp_nsecs: {}".format(self.rail_stamp_secs, self.rail_stamp_nsecs, self.img_stamp_secs, self.img_stamp_nsecs))
        if (self.img_stamp_secs < self.rail_stamp_secs):
            ret_val = False
        elif (self.img_stamp_secs == self.rail_stamp_secs):
            if (self.img_stamp_nsecs < self.rail_stamp_nsecs):
                ret_val = False
        return ret_val

    def clear_bar_pos(self):
        log.notice("clear_bar_pos")
        with self.bar_condition_:
            self.rail_position_dict['rail_bar']=[0.0,0.0,0.0]
            self.left_pose_dict['pan_bar_pose'][0] = 0
            self.left_pose_dict['cai_bar_pose'][0] = 0
            self.left_pose_dict['pot_bar_pose'][0] = 0
            self.left_pose_dict['net_bar_pose'][0] = 0
            self.right_pose_dict['pan_bar_pose'][0] = 0
            self.right_pose_dict['cai_bar_pose'][0] = 0
            self.right_pose_dict['pot_bar_pose'][0] = 0
            self.right_pose_dict['net_bar_pose'][0] = 0

    def clear_broc_pos(self):
        log.notice("clear_broc_pos")
        with self.broc_condition_:
            self.rail_position_dict['rail_broc']=[0.0,0.0,0.0]
            self.left_pose_dict['broc_pose'][0] = 0
            self.right_pose_dict['broc_pose'][0] = 0

    def clear_obj_pos(self):
        log.notice("clear_obj_pos")
        with self.obj_condition_:
            self.rail_position_dict['rail_mask']=[0.0,0.0,0.0]
            self.left_pose_dict['beef_grasp_pose_left'][0] = 0
            self.left_pose_dict['beef_grasp_pose_right'][0] = 0
            self.left_pose_dict['beef_center_pose'][0] = 0
            self.left_pose_dict['beef_press_pose_one'][0] = 0
            self.left_pose_dict['beef_press_pose_two'][0] = 0
            self.left_pose_dict['beef_press_pose_three'][0] = 0
            self.left_pose_dict['beef_press_pose_four'][0] = 0
            self.right_pose_dict['beef_grasp_pose_left'][0] = 0
            self.right_pose_dict['beef_grasp_pose_right'][0] = 0
            self.right_pose_dict['beef_center_pose'][0] = 0
            self.right_pose_dict['beef_press_pose_one'][0] = 0
            self.right_pose_dict['beef_press_pose_two'][0] = 0
            self.right_pose_dict['beef_press_pose_three'][0] = 0
            self.right_pose_dict['beef_press_pose_four'][0] = 0

            self.left_pose_dict['pan_center_pose'][0] = 0
            self.right_pose_dict['pan_center_pose'][0] = 0
            self.left_pose_dict['plate1_pose'][0] = 0
            self.right_pose_dict['plate1_pose'][0] = 0
            self.left_pose_dict['plate2_pose'][0] = 0
            self.right_pose_dict['plate2_pose'][0] = 0
            self.left_pose_dict['plate3_pose'][0] = 0
            self.right_pose_dict['plate3_pose'][0] = 0

    def left_plate_pos(self, grapslist, timeout):
        ret_value = []
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            wait_time += 0.4
            time.sleep(0.4)
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True) and (self.rail_cmp("left", self.rail_cmd, self.rail_position_dict['rail_mask']) == True):
                    if grapslist=='plate1_pose':
                        if self.left_pose_dict['plate1_pose'][0]!=0:
                            ret_value = self.left_pose_dict['plate1_pose'][1:7]
                            break
                    elif grapslist=='plate2_pose':
                        if self.left_pose_dict['plate2_pose'][0]!=0:
                            ret_value = self.left_pose_dict['plate2_pose'][1:7]
                            break
                    elif grapslist=='plate3_pose':
                        if self.left_pose_dict['plate3_pose'][0]!=0:
                            ret_value = self.left_pose_dict['plate3_pose'][1:7]
                            break
        log.notice("left_plate {} pos:{}".format(grapslist, ret_value))
        return ret_value

    def right_plate_pos(self, grapslist, timeout):
        ret_value = []
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            wait_time += 0.4
            time.sleep(0.4)
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True) and (self.rail_cmp("right", self.rail_cmd, self.rail_position_dict['rail_mask']) == True):
                    if grapslist=='plate1_pose':
                        if self.right_pose_dict['plate1_pose'][0]!=0:
                            ret_value = self.right_pose_dict['plate1_pose'][1:7]
                            break
                    elif grapslist=='plate2_pose':
                        if self.right_pose_dict['plate2_pose'][0]!=0:
                            ret_value = self.right_pose_dict['plate2_pose'][1:7]
                            break
                    elif grapslist=='plate3_pose':
                        if self.right_pose_dict['plate3_pose'][0]!=0:
                            ret_value = self.right_pose_dict['plate3_pose'][1:7]
                            break
        log.notice("right_plate {} pos:{}".format(grapslist, ret_value))
        return ret_value

    def left_beef_pos(self, grapslist, timeout):
        ret_value = []
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            wait_time += 0.4
            time.sleep(0.4)
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True) and (self.rail_cmp("left", self.rail_cmd, self.rail_position_dict['rail_mask']) == True):
                    if grapslist=='beef_grasp_pose_left':
                        if self.left_pose_dict['beef_grasp_pose_left'][0]!=0:
                            ret_value = self.left_pose_dict['beef_grasp_pose_left'][1:7]
                            break
                    elif grapslist=='beef_grasp_pose_right':
                        if self.left_pose_dict['beef_grasp_pose_right'][0]!=0:
                            ret_value = self.left_pose_dict['beef_grasp_pose_right'][1:7]
                            break
                    elif grapslist=='beef_center_pose':
                        if self.left_pose_dict['beef_center_pose'][0]!=0:
                            ret_value = self.left_pose_dict['beef_center_pose'][1:7]
                            break
                    elif grapslist=='beef_press_pose_one':
                        if self.left_pose_dict['beef_press_pose_one'][0]!=0:
                            ret_value = self.left_pose_dict['beef_press_pose_one'][1:7]
                    elif grapslist=='beef_press_pose_two':
                        if self.left_pose_dict['beef_press_pose_two'][0]!=0:
                            ret_value = self.left_pose_dict['beef_press_pose_two'][1:7]
                            break
                    elif grapslist=='beef_press_pose_three':
                        if self.left_pose_dict['beef_press_pose_three'][0]!=0:
                            ret_value = self.left_pose_dict['beef_press_pose_three'][1:7]
                            break
                    elif grapslist=='beef_press_pose_four':
                        if self.left_pose_dict['beef_press_pose_four'][0]!=0:
                            ret_value = self.left_pose_dict['beef_press_pose_four'][1:7]
                            break
        log.notice("left_beef {} pos:{}".format(grapslist, ret_value))
        return ret_value

    def right_beef_pos(self, grapslist, timeout):
        ret_value = []
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            wait_time += 0.4
            time.sleep(0.4)
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True) and (self.rail_cmp("right", self.rail_cmd, self.rail_position_dict['rail_mask']) == True):
                    if grapslist=='beef_grasp_pose_left':
                        if self.right_pose_dict['beef_grasp_pose_left'][0]!=0:
                            ret_value = self.right_pose_dict['beef_grasp_pose_left'][1:7]
                            break
                    elif grapslist=='beef_grasp_pose_right':
                        if self.right_pose_dict['beef_grasp_pose_right'][0]!=0:
                            ret_value = self.right_pose_dict['beef_grasp_pose_right'][1:7]
                            break
                    elif grapslist=='beef_center_pose':
                        if self.right_pose_dict['beef_center_pose'][0]!=0:
                            ret_value = self.right_pose_dict['beef_center_pose'][1:7]
                            break
                    elif grapslist=='beef_press_pose_one':
                        if self.right_pose_dict['beef_press_pose_one'][0]!=0:
                            ret_value = self.right_pose_dict['beef_press_pose_one'][1:7]
                            break
                    elif grapslist=='beef_press_pose_two':
                        if self.right_pose_dict['beef_press_pose_two'][0]!=0:
                            ret_value = self.right_pose_dict['beef_press_pose_two'][1:7]
                            break
                    elif grapslist=='beef_press_pose_three':
                        if self.right_pose_dict['beef_press_pose_three'][0]!=0:
                            ret_value = self.right_pose_dict['beef_press_pose_three'][1:7]
                            break
                    elif grapslist=='beef_press_pose_four':
                        if self.right_pose_dict['beef_press_pose_four'][0]!=0:
                            ret_value = self.right_pose_dict['beef_press_pose_four'][1:7]
                            break
        log.notice("right_beef {} pos:{}".format(grapslist, ret_value))
        return ret_value

    def beef_side_length(self, size, timeout):
        ret_value = 0.0
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True):
                    if size == 'beef_witdh':
                        if self.left_pose_dict['beef_center_pose'][0]!=0:
                            ret_value = self.beef_witdh
                            break
                    elif size == 'beef_length':
                        if self.left_pose_dict['beef_center_pose'][0] != 0:
                            ret_value = self.beef_length
                            break
            wait_time += 0.4
            time.sleep(0.4)
        return ret_value

    def pan_center_pos(self, arm, timeout):
        ret_value = []
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            wait_time += 0.4
            time.sleep(0.4)
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True) and (self.rail_cmp(arm, self.rail_cmd, self.rail_position_dict['rail_mask']) == True):
                    if (arm == 'left') and (self.left_pose_dict['pan_center_pose'][0]!=0):
                        ret_value = self.left_pose_dict['pan_center_pose'][1:7]
                        break
                    elif (arm == 'right') and (self.right_pose_dict['pan_center_pose'][0]!=0):
                        ret_value = self.right_pose_dict['pan_center_pose'][1:7]
                        break
        log.notice("{} arm pan_center pos:{}".format(arm, ret_value))
        return ret_value

    def left_bar_pos(self, grapslist, timeout):
        ret_value = []
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            wait_time += 0.4
            time.sleep(0.4)
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True) and (self.rail_cmp("left", self.rail_cmd, self.rail_position_dict['rail_bar']) == True):
                    if grapslist=='pan_bar_pose':
                        if self.left_pose_dict['pan_bar_pose'][0]!=0:
                            ret_value = self.left_pose_dict['pan_bar_pose'][1:7]
                            break
                    elif grapslist=='cai_bar_pose':
                        if self.left_pose_dict['cai_bar_pose'][0]!=0:
                            ret_value = self.left_pose_dict['cai_bar_pose'][1:7]
                            break
                    elif grapslist=='pot_bar_pose':
                        if self.left_pose_dict['pot_bar_pose'][0]!=0:
                            ret_value = self.left_pose_dict['pot_bar_pose'][1:7]
                            break
                    elif grapslist=='net_bar_pose':
                        if self.left_pose_dict['net_bar_pose'][0]!=0:
                            ret_value = self.left_pose_dict['net_bar_pose'][1:7]
                            break
        log.notice("left_bar {} pos:{}".format(grapslist, ret_value))
        return ret_value

    def right_bar_pos(self, grapslist, timeout):
        ret_value = []
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            wait_time += 0.4
            time.sleep(0.4)
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True) and (self.rail_cmp("right", self.rail_cmd, self.rail_position_dict['rail_bar']) == True):
                    if grapslist=='pan_bar_pose':
                        if self.right_pose_dict['pan_bar_pose'][0]!=0:
                            ret_value = self.right_pose_dict['pan_bar_pose'][1:7]
                            break
                    elif grapslist=='cai_bar_pose':
                        if self.right_pose_dict['cai_bar_pose'][0]!=0:
                            ret_value = self.right_pose_dict['cai_bar_pose'][1:7]
                            break
                    elif grapslist=='pot_bar_pose':
                        if self.right_pose_dict['pot_bar_pose'][0]!=0:
                            ret_value = self.right_pose_dict['pot_bar_pose'][1:7]
                            break
                    elif grapslist=='net_bar_pose':
                        if self.right_pose_dict['net_bar_pose'][0]!=0:
                            ret_value = self.right_pose_dict['net_bar_pose'][1:7]
                            break
        log.notice("right_bar {} pos:{}".format(grapslist, ret_value))
        return ret_value

    def broc_pos(self, arm, timeout):
        ret_value = []
        wait_time = 0
        if timeout < 2:
            timeout = 2
        while (wait_time < timeout):
            wait_time += 0.4
            time.sleep(0.4)
            with self.obj_condition_:
                if (self.rail_stamp_cmp() == True) and (self.rail_cmp(arm, self.rail_cmd, self.rail_position_dict['rail_broc']) == True):
                    if (arm == 'left') and (self.left_pose_dict['broc_pose'][0]!=0):
                        ret_value = self.left_pose_dict['broc_pose'][1:7]
                        break
                    elif (arm == 'right') and (self.right_pose_dict['broc_pose'][0]!=0):
                        ret_value = self.right_pose_dict['broc_pose'][1:7]
                        break
        log.notice("{} arm broc pos:{}".format(arm, ret_value))
        return ret_value

    # override function
    def sync_cmd_handle(self, msg):
        log.debug("sync_cmd_handle: get cmd: {}, params: {}".format(msg.cmd, msg.params))
        is_has_handle = True
        if msg.cmd == "getcmdlist":
            print "{0}:get {1}()".format(self.name_, msg.cmd)
            result_dic = self.get_cmd_list()
            err_info = ErrorInfo(0, "")
            self.reply_result(msg, err_info, result_dic)
        elif msg.cmd == "getstatus":
            result_dic = self.get_status_dict()
            err_info = ErrorInfo(0, "")
            self.reply_result(msg, err_info, result_dic)
        elif msg.cmd == "rail_cmd":
            error_code = 0
            error_msg = ""
            ret_value = None
            hrail = get_dict_key_value(msg.params, 'hrail', (float, int))
            if hrail is None:
                error_code = E_MOD_PARAM
                error_msg = "get [hrail] params failed"
            if 0 == error_code:
                arm = get_dict_key_value(msg.params, 'arm', (str, unicode))
                if arm is None:
                    error_code = E_MOD_PARAM
                    error_msg = "get [arm] params failed"
            if 0 == error_code:
                if (arm == 'left'):
                    self.rail_cmd[1] = hrail
                elif (arm == 'right') :
                    self.rail_cmd[2] = hrail
                self.rail_stamp = rospy.Time.now()
                self.rail_stamp_secs = self.rail_stamp.secs
                self.rail_stamp_nsecs = self.rail_stamp.nsecs
                #clear pos
                self.clear_bar_pos()
                self.clear_broc_pos()
                self.clear_obj_pos()
                self.cmd_rail_publish()
                log.notice("async_cmd_handle: rail_stamp_secs: {}, rail_stamp_nsecs: {}".format(self.rail_stamp_secs, self.rail_stamp_nsecs))
            err_info = ErrorInfo(error_code, error_msg)
            self.reply_result(msg, err_info, ret_value)
        else:
            is_has_handle = False
        return is_has_handle

    # override function
    def async_cmd_handle(self, msg):
        is_has_handle = True
        error_code = 0
        ret_value = None
        log.info("async_cmd_handle: get cmd: {}, params: {}".format(msg.cmd, msg.params))

        #startmaskrcnn
        if msg.cmd == "StartMaskrcnn":
            start_maskrcnn = True
            rootdir = ROOT_DIR
            os.chmod(rootdir + "ros_mask.py", stat.S_IRWXO)
            os.system('gnome-terminal -x python')
            time.sleep(10)

        #stopmaskrcnn
        elif msg.cmd == "StopMaskrcnn":
            stop_maskrcnn = True
            nodes = os.popen("rosnode list").readlines()
            node = None
            for i in range(len(nodes)):
                if nodes[i].startswith('/maskrcnn_node'):
                    node = nodes[i]
                    node = node.replace("\n","")
            if node is not None:
                os.system('rosnode kill '+ node)

        #beef_leftarm
        elif msg.cmd == "beef_leftarm":
            error_info = ErrorInfo(error_code, "")
            grapslist = get_dict_key_value(msg.params, 'grapslist', (str, unicode))
            if grapslist is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
                if timeout is None:
                    error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.left_beef_pos(grapslist, timeout)

        elif msg.cmd=='beef_wl':
            error_info = ErrorInfo(error_code, "")
            size = get_dict_key_value(msg.params, 'size', (str, unicode))
            if size is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
                if timeout is None:
                    error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.beef_side_length(size, timeout)

        #beef_rightarm
        elif msg.cmd == "beef_rightarm":
            error_info = ErrorInfo(error_code, "")
            grapslist = get_dict_key_value(msg.params, 'grapslist', (str, unicode))
            if grapslist is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
                if timeout is None:
                    error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.right_beef_pos(grapslist, timeout)

        #pan_center_pose_leftarm
        elif msg.cmd == "pan_center_pose_leftarm":
            error_info = ErrorInfo(error_code, "")
            arm = "left"
            timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
            if timeout is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.pan_center_pos(arm, timeout)

        #pan_center_pose_rightarm
        elif msg.cmd == "pan_center_pose_rightarm":
            error_info = ErrorInfo(error_code, "")
            arm = "right"
            timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
            if timeout is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.pan_center_pos(arm, timeout)

        #plate_leftarm
        elif msg.cmd == "plate_leftarm":
            error_info = ErrorInfo(error_code, "")
            grapslist = get_dict_key_value(msg.params, 'grapslist', (str, unicode))
            if grapslist is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
                if timeout is None:
                    error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.left_plate_pos(grapslist, timeout)

         #plate_rightarm
        elif msg.cmd == "plate_rightarm":
            error_info = ErrorInfo(error_code, "")
            grapslist = get_dict_key_value(msg.params, 'grapslist', (str, unicode))
            if grapslist is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
                if timeout is None:
                    error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.right_plate_pos(grapslist, timeout)

        ##bar_leftarm
        elif msg.cmd == "bar_leftarm":
            error_info = ErrorInfo(error_code, "")
            grapslist = get_dict_key_value(msg.params, 'grapslist', (str, unicode))
            if grapslist is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
                if timeout is None:
                    error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.left_bar_pos(grapslist, timeout)

         #bar_rightarm
        elif msg.cmd == "bar_rightarm":
            error_info = ErrorInfo(error_code, "")
            grapslist = get_dict_key_value(msg.params, 'grapslist', (str, unicode))
            if grapslist is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
                if timeout is None:
                    error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.right_bar_pos(grapslist, timeout)

        #broc_leftarm
        elif msg.cmd == "broc_leftarm":
            error_info = ErrorInfo(error_code, "")
            arm = "left"
            timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
            if timeout is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.broc_pos(arm, timeout)

        #broc_rightarm
        elif msg.cmd == "broc_rightarm":
            error_info = ErrorInfo(error_code, "")
            arm = "right"
            timeout = get_dict_key_value(msg.params, 'timeout', (float, int))
            if timeout is None:
                error_code = E_MOD_PARAM
            if error_code == 0:
                ret_value = self.broc_pos(arm, timeout)

        #################################################
        else:
            is_has_handle = False

        if True == is_has_handle:
            if 0 == error_code:
                error_info = ErrorInfo(error_code, "")
            elif E_MOD_PARAM == error_code:
                error_info = ErrorInfo(error_code, "params error!")
            else:
                error_info = ErrorInfo(error_code, "execution error!")
            self.reply_result(msg, error_info, ret_value)

        return is_has_handle

    # override function
    def abort_handle(self):
        log.info("abort_handle")
        return True

    def reset_handle(self):
        log.info("reset_handle")
        return True

    def get_cmd_list(self):
        return cmd_description_dict

    def get_status_dict(self):
        status_dic = {
            'status': self.get_status(),
            'code':self.get_statuscode(),
        }
        return status_dic

    def set_status(self, status):
        with self.data_condition_:
            self.status_ = status

    def get_status(self):
        with self.data_condition_:
            if self.is_exception_mode() is True:
                status = STATUS_ERROR
            else:
                status = self.status_
        return status

    def set_statuscode(self, statuscode):
        with self.data_condition_:
            self.statuscode_ = statuscode

    def get_statuscode(self):
        with self.data_condition_:
            if self.is_exception_mode() is True:
                statuscode = E_MOD_EXCEPTION
            else:
                statuscode = self.statuscode_
        return statuscode
