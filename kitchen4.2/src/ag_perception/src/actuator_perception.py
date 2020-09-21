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

from actuator import Actuator
from actuator import ErrorInfo
from actuator import ActuatorCmdType
from proxy_client import PS_Socket
from std_msgs.msg import Float32MultiArray,String
from rlog import rlog
import numpy as np
log = rlog()
from sensor_msgs.msg import Image
import subprocess

# Define Error code
MOD_ERR_NUM = 3600
MOD_ERR_SELF_OFFSET = 20
E_OK = 0
E_MOD_PARAM = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 1
E_MOD_STATUS = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 2
E_MOD_DRIVER = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 3
E_MOD_EXCEPTION = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 5
E_MOD_ABORT_FAILED = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 6


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
        #sanitycheck
        {
            'cmd': 'setObjReadinessStart',
            'atype': 'motion',
            'params':[]
        },
        {
            'cmd': 'setObjReadinessStop',
            'atype': 'motion',
            'params':[]
        },
        {
            'cmd': 'setSanityYOLOStart',
            'atype': 'motion',
            'params':[]
        },
        {
            'cmd': 'setSanityYOLOStop',
            'atype': 'motion',
            'params':[]
        },
        {
            'cmd': 'setSanityRCNNStart',
            'atype': 'motion',
            'params':[]
        },
        {
            'cmd': 'setSanityRCNNStop',
            'atype': 'motion',
            'params':[]
        },
        {
            'cmd': 'getAbnormalEvents',
            'atype': 'sensing',
            'params':[],
            'return':{
                'type': 'boolean'
            }
        },
        {
            'cmd': 'getObjReadiness',
            'atype': 'sensing',
            'params':[],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'getForeignPos',
            'atype': 'sensing',
            'params':[],
            'return':{
                'type': 'array'
            }
        },
        {
            'cmd': 'startAlgo',
            'atype': 'motion',
            'params':[]
        },
        {
            'cmd': 'stopAlgo',
            'atype': 'motion',
            'params':[]
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

def json_dumps(dict_str):
    try:
        # note [ensure_ascii=False] must be set, then utf-8 chinese code can be use
        json_str = json.dumps(dict_str, ensure_ascii=False)
    except Exception, e:
        log.error("Dict to json error: {}, dict: {}".format(e, dict_str))
        return
    else:
        return json_str

class ActuatorPerception(Actuator):
    def __init__(self, name, is_simulation, proxy_name, proxy_ip):
        Actuator.__init__(self, name)

        self.proxy_name_ = proxy_name
        self.is_simulation_ = is_simulation
        self.data_condition_ = threading.Condition()
        self.proxy_ip = proxy_ip
        self.status_ = STATUS_IDLE
        self.statuscode_ = E_OK

        #sanitycheck
        self.abnormalevents_val = False
        self.objreadiness_val = []
        self.foreignpos_val = np.empty(shape=[0, 2])
        self.camera_err_report = False
        self.scene_err_report = False
        self.rarity_report_time = 0

        # connect to proxy
        self.pub_socket = PS_Socket(self.proxy_ip)

        if self.is_simulation_ is False:
            #sanitycheck
            self.abnormalevents = rospy.Subscriber("/processor/events",Float32MultiArray,self.abnormalevents_callback)
            self.objreadiness = rospy.Subscriber("/processor/objreadiness",Float32MultiArray,self.objreadiness_callback)
            self.foreignpos = rospy.Subscriber("/processor/foreignpos",Float32MultiArray,self.foreignpos_callback)
            #testing
            #self.test = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback_test)
        self.objreadinessStartTime = 0


    #testing
    def callback_test(self, data):
        flag = 0
        if rospy.has_param('sanitycheckflag'):
            flag = rospy.get_param('sanitycheckflag')
        print("=====================================flag = ", flag)
        if(flag == 1):
            print("==start sanity check block==")
            nodes = os.popen("rosnode list").readlines()
            node_yolo = None
            for i in range(len(nodes)):
                if nodes[i].startswith('/processor_sanity'):
                    node_yolo = nodes[i]
            if node_yolo is None:
                abs_file = os.path.abspath(os.path.join(__file__,"../../.."))+'/'
                print(abs_file)
                sanity_path = abs_file + 'sanity/demo.py'
                os.system('gnome-terminal -- python '+ sanity_path)
                time.sleep(1)

        if(flag == 0):
            print("==stop sanity check block==")
            #kill yolo if exists
            nodes = os.popen("rosnode list").readlines()
            node_yolo = None
            for i in range(len(nodes)):
                if nodes[i].startswith('/processor_sanity'):
                    node_yolo = nodes[i]
                    node_yolo = node_yolo.replace("\n","")
            if node_yolo is not None:
                os.system('rosnode kill '+ node_yolo)

    #sanitycheck
    def abnormalevents_callback(self, abnormalevents):
        if abnormalevents.data[0] == 1.0 or abnormalevents.data[1] == 1.0:
            if abnormalevents.data[0] == 1.0:
                if self.scene_err_report == False:
                    self.scene_err_report = True
                    self.exception_report(self.proxy_name_, "scene error!", E_MOD_EXCEPTION)
            else:
                self.scene_err_report = False
            if abnormalevents.data[1] == 1.0:
                if self.camera_err_report == False:
                    self.camera_err_report = True
                    self.exception_report(self.proxy_name_, "camera moved!", E_MOD_EXCEPTION)
            else:
                self.camera_err_report = False
        else:
            self.scene_err_report = False
            self.camera_err_report = False
            rarityDoc = {}
            rarityDoc['data'] = []
            if abnormalevents.data[2] == 1.0:
                self.abnormalevents_val = True
                rarityDoc['data'] = self.foreignpos_val.tolist()
            else:
                self.abnormalevents_val = False

            if (rospy.get_time() - self.rarity_report_time) > 1:
                rarityStr = json_dumps(rarityDoc)
                log.debug("abnormalevents_callback: publish rarity, msg: {}".format(rarityStr))
                self.pub_socket.publish('rarity', rarityStr)
                self.rarity_report_time = rospy.get_time()

        log.notice("abnormal events: {}".format(self.abnormalevents_val))


    def objreadiness_callback(self, objreadiness):
        while len(self.objreadiness_val) > 0 : self.objreadiness_val.pop()
        #self.objreadiness_val.append(objreadiness)
        for i in range(len(objreadiness.data)):
            if i == 1 or i == 3:
                self.objreadiness_val.append(1.0)
            else:
                self.objreadiness_val.append(objreadiness.data[i])

        log.notice("object readinesss: {}".format(self.objreadiness_val))

    def foreignpos_callback(self, foreignpos):
        rarity_val = np.empty(shape=[0, 2])
        for i in range(len(foreignpos.data)):
            if i%2==0:
                rarity_val = np.append(rarity_val, [[foreignpos.data[i], foreignpos.data[i+1]]], axis=0)
            elif i%2 == 1:
                continue
        self.foreignpos_val = rarity_val

        log.notice("foreign object position: {}".format(self.foreignpos_val))

    # override function
    def sync_cmd_handle(self, msg):
        log.debug("sync_cmd_handle: get cmd: {}, params: {}".format(msg.cmd, msg.params))
        is_has_handle = True
        if msg.cmd == "getcmdlist":
            result_dic = self.get_cmd_list()
            err_info = ErrorInfo(0, "")
            self.reply_result(msg, err_info, result_dic)
        elif msg.cmd == "getstatus":
            result_dic = self.get_status_dict()
            err_info = ErrorInfo(0, "")
            self.reply_result(msg, err_info, result_dic)
        else:
            is_has_handle = False
        return is_has_handle

    # override function
    def async_cmd_handle(self, msg):
        is_has_handle = True
        error_code = 0
        ret_value = None
        log.info("async_cmd_handle: get cmd: {}, params: {}".format(msg.cmd, msg.params))

        #sanitycheck
        if msg.cmd == "getAbnormalEvents":
            ret_value = self.abnormalevents_val
        elif msg.cmd == "getObjReadiness":
            if rospy.has_param('objreadinessflag') and rospy.get_param('objreadinessflag') == 1 and (rospy.get_time() - self.objreadinessStartTime) > 2:
                ret_value = self.objreadiness_val
            else:
                ret_value = []
        elif msg.cmd == "getForeignPos":
            ret_value = self.foreignpos_val
        elif msg.cmd == "setObjReadinessStart":
            rospy.set_param('objreadinessflag', 1)
            self.objreadinessStartTime = rospy.get_time()
        elif msg.cmd == "setObjReadinessStop":
            rospy.set_param('objreadinessflag', 0)
            self.objreadinessStartTime = 0
        elif msg.cmd == "setSanityYOLOStart":
            if(self.sanitycheckHistory == 0):
                log.notice("==start sanity check block==")
                nodes = os.popen("rosnode list").readlines()
                node_yolo = None
                for i in range(len(nodes)):
                    if nodes[i].startswith('/processor_sanity'):
                        node_yolo = nodes[i]
                if node_yolo is None:
                    abs_file = os.path.abspath(os.path.join(__file__,"../../.."))+'/'
                    sanity_path = abs_file + 'sanity/demo.py'
                    os.system('gnome-terminal -- python '+ sanity_path)
        elif msg.cmd == "setSanityYOLOStop":
            log.notice("==stop sanity check block==")
            nodes = os.popen("rosnode list").readlines()
            node = None
            for i in range(len(nodes)):
                if nodes[i].startswith('/processor_sanity'):
                    node = nodes[i]
                    node = node.replace("\n","")
            if node is not None:
                os.system('rosnode kill '+ node)
        elif msg.cmd == "setSanityRCNNStart":
            rospy.set_param('sanitycheckflag', 1)
        elif msg.cmd == "setSanityRCNNStop":
            rospy.set_param('sanitycheckflag', 0)
        elif msg.cmd == "startAlgo":
            log.notice("start kinect")
            nodes = os.popen("rosnode list").readlines()
            node_kinect = None
            for i in range(len(nodes)):
                if nodes[i].startswith('/kinect2_bridge'):
                    node_kinect = nodes[i]

            if node_kinect is None:
                os.system("gnome-terminal -- bash -c \"roslaunch kinect2_bridge kinect2_bridge.launch ; exec bash;\"")
        elif msg.cmd == "stopAlgo":
            log.notice("stop kinect")
            nodes = os.popen("rosnode list").readlines()
            node_kinect = None
            for i in range(len(nodes)):
                if nodes[i].startswith('/kinect2_bridge'):
                    node_kinect = nodes[i]
            if node_kinect is not None:
                os.system("ps aux | grep \"bash -c\" | grep kinect2_bridge.launch | awk \'{print $2}\' | xargs kill -9")
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
