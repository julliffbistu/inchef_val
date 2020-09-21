#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os
import time
import threading
import sys
import json


abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../comm")
sys.path.append(abs_file + "/../log")
from proxy_client import ProxyClient, PS_Socket
from rlog import rlog

log = rlog()

MSG_ACK_TIMEOUT_MS = 500

MSG_WAIT_INFINITE = 999999999


class MsgCondition:
    def __init__(self, seq, adapter_name, cmd):
        self.seq_ = seq
        self.adapter_name_ = adapter_name
        self.cmd_ = cmd
        self.condition_ = threading.Condition()
        self.status = "init"
        self.result_data = None
        self.is_has_handle = False
        self.code = 0
        self.codestring = ''


class ActuatorInterface:
    def __init__(self, client_name, proxy_ip=None):
        self.name_ = client_name
        self.actuator_name_ = None
        self.adapter_name_ = None
        self.core_name_ = None
        self.proxy_ip_ = proxy_ip
        self.condition_ = threading.Condition()
        self.msg_cond_list_lock_ = threading.Lock()
        self.msg_cond_list_ = []
        self.send_seq_ = 0
        self.send_seq_lock_ = threading.Lock()
        self.topic_list_ = []

        self.proxy_ = ProxyClient(client_name, self, self.proxy_callback, proxy_addr=proxy_ip)
        self.proxy_.connect()
        self.topic_socket_ = PS_Socket(proxy_ip, self.subscribe_callback)

        log.info("{}:connect to proxy [{}] success!".format(self.name_, self.proxy_ip_))

    def set_target(self, actuator_name, core_name=None):
        self.actuator_name_ = actuator_name
        self.core_name_ = core_name

    def set_target_direct(self, actuator_name, adapter_name):
        self.actuator_name_ = actuator_name
        self.adapter_name_ = adapter_name

    def proxy_callback(self, this_ins, msg):
        log.debug("{}:client recv: {}".format(self.name_, msg))
        adapter_name = self.get_dict_key_value(msg, 'cmdsrc', (str, unicode))
        if adapter_name is None:
            log.error("adapter name error!")
            return

        cmd = self.get_dict_key_value(msg, 'cmd', (str, unicode))
        if cmd is None:
            log.error("cmd error")
            return

        params = self.get_params(msg)

        seq = self.get_dict_key_value(params, 'seq', int)

        if seq is None:
            log.error("seq error")
            return

        type = self.get_dict_key_value(params, 'type', (str, unicode))

        if type is None:
            log.error("type error")
            return

        code = self.get_dict_key_value(params, 'code', int)
        if code is None:
            log.error("code error")
            return

        codestring = self.get_dict_key_value(params, 'codestring', (str, unicode))
        if codestring is None:
            codestring = ''

        data = self.get_dict_key_value(params, 'data')

        self.msg_cond_list_lock_.acquire()
        for msg_cond in self.msg_cond_list_:
            log.debug("cond: seq:{},is_has_handle:{},type:{},adapter_name{},cmd:{}".format(msg_cond.seq_, msg_cond.is_has_handle, type, msg_cond.adapter_name_, msg_cond.cmd_))
            log.debug("rev : seq:{},type:{},adapter_name:{},cmd:{}".format(seq, type, adapter_name, cmd))
            if msg_cond.seq_ == seq and type == 'ack' \
                    and msg_cond.adapter_name_ == adapter_name\
                    and msg_cond.cmd_ == cmd\
                    and msg_cond.is_has_handle is False:
                msg_cond.status = "ack"
                with msg_cond.condition_:
                    msg_cond.condition_.notify()
            elif msg_cond.seq_ == seq and type == 'res' \
                    and msg_cond.adapter_name_ == adapter_name\
                    and msg_cond.cmd_ == cmd\
                    and msg_cond.is_has_handle is False:
                #print("get cond")
                msg_cond.result_data = data
                msg_cond.is_has_handle = True
                msg_cond.status = "res"
                msg_cond.code = code
                msg_cond.codestring = codestring
                with msg_cond.condition_:
                    msg_cond.condition_.notify()
            else:
                log.error("unknow replay type!")
        self.msg_cond_list_lock_.release()

    def subscribe_callback(self, args, topic, content):
        #print "get topic:", topic
        for topic_tup in self.topic_list_:
            if topic == topic_tup[0]:
                topic_tup[1](content)

    # value_type: int, float, str, bool, list, dict
    def get_dict_key_value(self, dict_ins, key, value_type=None):
        if key in dict_ins:
            value = dict_ins.get(key)
            if value_type is not None:
                if isinstance(value, value_type) is False:
                    value = None
        else:
            value = None
        return value

    def json_loads(self, json_str):
        # print "load json:",json_str
        try:
            strings = json.loads(json_str, encoding='utf-8')
        except:
            print("Parsing json string error! json_str:", json_str)
            return
        else:
            return strings

    def json_dumps(self, dict_str):
        # print "dict:",dict_str
        try:
            # note [ensure_ascii=False] must be set, then utf-8 chinese code can be use
            json_str = json.dumps(dict_str, ensure_ascii=False)
        except:
            log.error("Dict to json error! dict:", dict_str)
            return
        else:
            return json_str

    def get_params(self, cmd):
        try:
            if 'params' in cmd:
                param_json = cmd.get('params')
        except:
            log.error("Cmd params error! cmd:", cmd)
            return
        else:
            param = self.json_loads(param_json)
            return param

    def push_msg_cond(self, msg_cond):
        self.msg_cond_list_lock_.acquire()
        self.msg_cond_list_.append(msg_cond)
        self.msg_cond_list_lock_.release()

    def delete_msg_cond(self, msg_cond):
        self.msg_cond_list_lock_.acquire()
        self.msg_cond_list_.remove(msg_cond)
        self.msg_cond_list_lock_.release()

    def get_seq(self):
        self.send_seq_lock_.acquire()
        seq = self.send_seq_
        self.send_seq_ = self.send_seq_ + 1
        self.send_seq_lock_.release()
        return seq

    def send_msg(self, cmd, params, wait=True):
        ret = 0

        if self.adapter_name_ is None or self.actuator_name_ is None:
            return -1, None

        timestamp_handle_start = int(time.time() * 1000)
        seq = self.get_seq()
        if params is None:
            params = {}
        params['seq'] = seq
        params_json = self.json_dumps(params)

        actuator_cmd = self.actuator_name_ + ':' + cmd
        self.proxy_.send(self.adapter_name_, actuator_cmd, params_json)

        if wait is False:
            return 0, None
        msg_cond = MsgCondition(seq, self.adapter_name_, actuator_cmd)
        self.push_msg_cond(msg_cond)
        retry_cnt = 0
        # print("wait ack...")
        while msg_cond.status == "init" and retry_cnt < 3:
            with msg_cond.condition_:
                #msg_cond.condition_.wait(MSG_ACK_TIMEOUT_MS/1000)
                msg_cond.condition_.wait(MSG_WAIT_INFINITE)
            if msg_cond.status == "init":
                log.error("retry send cmd!")
                self.proxy_.send_cmd(self.adapter_name_, actuator_cmd, params_json)
                retry_cnt = retry_cnt + 1

        if msg_cond.status == "ack":
            with msg_cond.condition_:
                msg_cond.condition_.wait(MSG_WAIT_INFINITE)
                ret = msg_cond.code

        if msg_cond.status != "res":
            ret = -1

        timestamp_handle_end = int(time.time() * 1000)
        #print("{}:cmd[{}] handle time:{}".format(self.name_, cmd,
        #                                         timestamp_handle_end - timestamp_handle_start))
        if msg_cond.result_data is not None:
            result_data = msg_cond.result_data
        else:
            result_data = None

        self.delete_msg_cond(msg_cond)

        return ret, result_data

    def subscribe(self, topic, callback):
        topic_list = []
        topic_name = "{0}:{1}:{2}".format(self.adapter_name_, self.actuator_name_, topic)
        topic_list.append(topic_name)
        self.topic_list_.append((topic_name, callback))
        self.topic_socket_.subscribe(topic_list)
        log.info("{}:subscribe topic:{}".format(self.name_, topic_name))
