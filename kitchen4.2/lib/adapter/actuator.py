#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import os
import threading
import Queue
import time
import json

abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../comm")
sys.path.append(abs_file + "/../log")
from rlog import rlog

from proxy_client import ProxyClient
from proxy_client import Socket
from proxy_client import PS_Socket

# Define Error code
MOD_ERR_NUM = 700
MOD_ERR_SELF_OFFSET = 20
E_OK = 0
E_MOD_IN_EXCEPTION_MODE = MOD_ERR_NUM + 9
E_MOD_PARAM = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 1
E_MOD_STATUS = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 2
E_MOD_ABORT = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 3
E_MOD_TIMEOUT = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 4
E_MOD_NOT_SUPPORT_CMD = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 5
E_MOD_ABORT_FAILED = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 6
E_MOD_RESET_FAILED = MOD_ERR_NUM + MOD_ERR_SELF_OFFSET + 7

# Define status
ADAPTER_STATUS_UNINIT = "uninitialized"
ADAPTER_STATUS_IDLE = "idle"
ADAPTER_STATUS_BUSY = "busy"
ADAPTER_STATUS_ERROR = "error"

config_json_file_path = "/opt/knowin/configs/config.json"

ABNORMAL_TOPIC_NAME = "abnormal"
kTopicException = "topic_exception"
kTopicHeartbeat = "topic_heartbeat"

log = rlog()

class Status:
    uninit = ADAPTER_STATUS_UNINIT
    idle = ADAPTER_STATUS_IDLE
    busy = ADAPTER_STATUS_BUSY
    error = ADAPTER_STATUS_ERROR

class ActuatorNameIns:
    def __init__(self, name, ins):
        self.name_ = name
        self.ins_ = ins


class ErrorInfo:
    def __init__(self, code, code_string):
        self.code = code
        self.str = code_string


class ActuatorCmdType:
    def __init__(self, src, cmd, params, params_seq, timestamp, org_cmd):
        self.cmd_src = src
        self.cmd = cmd
        self.params = params
        self.params_seq = params_seq
        self.timestamp_receive = timestamp
        self.org_cmd = org_cmd


class ExceptionCmdType:
    def __init__(self, cmd, seq, proxy_msg):
        self.cmd = cmd
        self.seq = seq
        self.proxy_msg = proxy_msg


class Adapter:
    def __init__(self, name):
        self.name_ = name
        self.actuator_list_ = []
        self.proxy_ = None
        self.abnormal_topic_ = None
        self.exception_socket_ = None
        self.exception_send_seq_ = 0
        self.heartbeat_send_seq_ = 0
        self.data_condition_ = threading.Condition()
        self.status_ = ADAPTER_STATUS_UNINIT
        self.cmd_mask_exception_list_ = ['getcmdlist', 'getstatus', 'getappinfo', 'setloglevel', 'getsysteminfo']
        # create proxy
        unit_dict = JsonUtils.get_config_unit('application', 'proxy', 'proxy')
        if unit_dict is None:
            proxy_ip = None
        else:
            proxy_ip = JsonUtils.get_dict_key_value(unit_dict, 'ip', (str, unicode))
        if proxy_ip is None:
            log.error("proxy config error! the program can not run!")
            exit()
        client = ProxyClient(self.name_, self, self.proxy_callback, proxy_addr=proxy_ip)
        client.connect()
        self.proxy_ = client
        log.info("proxy connect to {} succeed!".format(proxy_ip))
        self.abnormal_topic_ = PS_Socket(proxy_ip)
        log.info("socket publish has been connected!")
        self.topic_sub_ = PS_Socket(proxy_ip, self.exception_callback, self)
        topic_list = []
        topic_name = "{}".format(kTopicException);
        topic_name = topic_name.decode('utf-8');
        topic_list.append(topic_name)
        self.topic_sub_.subscribe(topic_list)
        log.info("socket subscribe has been connected!")

        # read log level
        unit_dict = JsonUtils.get_config_unit('application', 'actuator', name)
        if unit_dict is None:
            log_level = None
        else:
            log_level = JsonUtils.get_dict_key_value(unit_dict, 'loglevel', (str, unicode))
        if log_level is None:
            log_level = 'info'
        else:
            log_level = log_level.encode('utf-8')
        self.log_level_ = log_level
        log.warn("[adapter]read config log level [{}]".format(log_level))

        self.heartbeatHandle_th_ = threading.Thread(target=Adapter.heartbeat_handle_thread, args=(self,))
        self.heartbeatHandle_th_.setDaemon(True)
        self.heartbeatHandle_th_.start()

    def proxy_callback(self, adapter_ins, proxy_msg):
        log.info("receive msg: {}".format(proxy_msg))
        cmd = JsonUtils.get_dict_key_value(proxy_msg, "cmd", str)
        cmd_src = JsonUtils.get_dict_key_value(proxy_msg, "cmdsrc", str)
        params = JsonUtils.get_params(proxy_msg)

        params_seq = JsonUtils.get_dict_key_value(params, "seq", int);
        if params_seq is None:
            params_seq = 0

        is_cmd_accept = False
        if cmd is None or cmd_src is None or params is None:
            log.error("lack of some value!")
        else:
            string_list = cmd.split(":", 1)
            if 2 == len(string_list):
                actuator_str = string_list[0]
                cmd_str = string_list[1]

                if actuator_str == self.name_:
                    timestamp = int(time.time() * 1000)
                    msg = ActuatorCmdType(cmd_src, cmd_str, params, params_seq, timestamp, cmd)
                    self.adapter_cmd_handle(msg)
                    is_cmd_accept = True
                else:
                    for obj in self.actuator_list_:
                        if obj.name_ == actuator_str:
                            timestamp = int(time.time() * 1000)
                            msg = ActuatorCmdType(cmd_src, cmd_str, params, params_seq, timestamp, cmd)
                            self.actuator_cmd_handle(obj.ins_, msg)
                            is_cmd_accept = True
                            break

        if is_cmd_accept is False:
            log.info("This message has no actuator accept!")
            timestamp = int(time.time() * 1000)
            msg = ActuatorCmdType(cmd_src, cmd, params, params_seq, timestamp, cmd)
            error_info = ErrorInfo(E_MOD_PARAM, "none actuator accept")
            self.reply_ack(msg, E_MOD_PARAM)
            self.reply_result(msg, error_info, None)

    def exception_callback(self, adapter_ins, topic, content):
        if kTopicException == topic:
            content_dic = JsonUtils.json_loads(content)
            if content_dic is not None:
                err_level = JsonUtils.get_dict_key_value(content_dic, 'level', (str, unicode))
            else:
                err_level = None

            if 'abort' == err_level:
                log.info("receive abort msg: {}".format(content))
                # lookup adapter items
                for obj in self.actuator_list_:
                    self.exception_cmd_handle(obj.ins_, "exceptionreport", content)

    def heartbeat_handle_thread(self):
        log.info("heartbeat handle thread running!")
        while True:
            error_code = 0
            status = self.get_status()
            if ADAPTER_STATUS_IDLE == status:
                # check exception mode
                for obj in self.actuator_list_:
                    if obj.ins_.is_exception_mode() is True:
                        status = ADAPTER_STATUS_ERROR
                        error_code = E_MOD_IN_EXCEPTION_MODE
                        break
            self.heartbeat_report(error_code, status)
            time.sleep(0.5)

    def reg_actuator(self, actuator_ins):
        ins = ActuatorNameIns(actuator_ins.name_, actuator_ins)
        self.actuator_list_.append(ins)
        actuator_ins.set_adapter(self)

    def reply_ack(self, msg, err_code):
        reply_dic = {
            'type': "ack",
            'seq': msg.params_seq,
            'code': err_code,
        }
        reply_json = JsonUtils.json_dumps(reply_dic)
        self.proxy_.send(msg.cmd_src, msg.org_cmd, reply_json)
        log.info("reply ack: {}".format(reply_json))

    def reply_result(self, msg, error_info, result):
        timestamp_handle_end = int(time.time() * 1000)
        handle_duration = timestamp_handle_end - msg.timestamp_receive
        info = dict()
        info['timestamp'] = msg.timestamp_receive
        info['duration'] = handle_duration
        reply_dic = {
            'type': 'res',
            'seq': msg.params_seq,
            'code': error_info.code,
            'errmsg': error_info.str,
            'info': info
        }
        if result is not None:
            reply_dic['data'] = result
        reply_json = JsonUtils.json_dumps(reply_dic)
        self.proxy_.send(msg.cmd_src, msg.org_cmd, reply_json)
        log.info("reply result: {}".format(reply_json))

    def adapter_cmd_handle(self, msg):
        error_code = E_OK
        err_info = ErrorInfo(0, "")
        # reply ack firstly
        self.reply_ack(msg, error_code)

        if "setloglevel" == msg.cmd:
            log_level = JsonUtils.get_dict_key_value(msg.params, "level", (str, unicode));
            if log_level is not None:
                log.warn("change log level to [{}]".format(log_level))
                log_level = log_level.encode('utf-8')
                log.set_priority(log_level)
                self.log_level_ = log_level
            else:
                log.error("set log level param error! param: {}".format(msg.params))
                err_info = ErrorInfo(E_MOD_PARAM, "set log level param error!")

            self.reply_result(msg, err_info, None)
        elif "getappinfo" == msg.cmd:
            reply_data = {
            'app': self.name_,
            'level': self.log_level_
            }
            self.reply_result(msg, err_info, reply_data)
        else:
            log.error("command not support! cmd= {}".format(msg.cmd))
            err_info = ErrorInfo(E_MOD_PARAM, "command not support! cmd= {}".format(msg.cmd))
            self.reply_result(msg, err_info, None)


    def actuator_cmd_handle(self, actuator_ins, msg):
        error_code = E_OK
        # reply ack firstly
        self.reply_ack(msg, error_code)

        if "resetstatus" == msg.cmd:
            exception_msg = ExceptionCmdType("reset", 0, msg)
            actuator_ins.push_exception_cmd(exception_msg)
        elif "getsysteminfo" == msg.cmd:
            version, compiler_date = actuator_ins.get_version_info()
            reply_data = {
            'name': actuator_ins.name_,
            'version': version,
            'date': compiler_date
            }
            err_info = ErrorInfo(0, "")
            self.reply_result(msg, err_info, reply_data)
        else:
            is_exception_mode = actuator_ins.is_exception_mode()
            if is_exception_mode is False:
                is_has_handle = actuator_ins.sync_cmd_handle(msg)
                if is_has_handle is False:
                    actuator_ins.push_async_cmd(msg)
            else:
                if self.is_mask_exception_cmd(msg.cmd):
                    is_has_handle = actuator_ins.sync_cmd_handle(msg)
                    if is_has_handle is False:
                        err_info = ErrorInfo(E_MOD_NOT_SUPPORT_CMD, "Not support command in exception mode")
                        self.reply_result(msg, err_info, None)
                else:
                    # the actuator receive or report a exception, reject handle commands
                    err_info = ErrorInfo(E_MOD_STATUS, "exception mode")
                    self.reply_result(msg, err_info, None)

    def heartbeat_report(self, err_code, status):
        seq = self.heartbeat_send_seq_
        self.heartbeat_send_seq_ += 1
        report_dic = {
            'module': self.name_,
            'type': "rpt",
            'seq': seq,
            'code': err_code,
            'status': status,
        }
        report_json = JsonUtils.json_dumps(report_dic)
        self.abnormal_topic_.publish(kTopicHeartbeat, report_json)
        log.debug("heartbeat:{}".format(report_json))

    def exception_get_seq(self):
        self.data_condition_.acquire()
        seq = self.exception_send_seq_
        self.exception_send_seq_ = seq + 1
        self.data_condition_.release()
        return seq

    def exception_report(self, module_name, msg, err_code):
        seq = self.exception_get_seq()
        report_dic = {
            'type': "rpt",
            'level': "abort",
            'module': module_name,
            'errmsg': msg,
            'seq': seq,
            'code': err_code,
            'stamp': int(time.time() * 1000)
        }
        report_json = JsonUtils.json_dumps(report_dic)
        self.abnormal_topic_.publish(kTopicException, report_json)
        log.error("exception report: {}".format(report_json))

    def exception_cmd_handle(self, actuator_ins, cmd, params):
        if "exceptionreport" == cmd:
            exception_msg = ExceptionCmdType('exceptionreport', 0, None)
            actuator_ins.push_exception_cmd(exception_msg)
        else:
            log.error("Unknow cmd: {}".format(cmd))

    def set_status(self, status):
        with self.data_condition_:
            self.status_ = status

    def get_status(self):
        with self.data_condition_:
            status = self.status_
        return status

    def is_mask_exception_cmd(self, cmd):
        ret = False
        for cmd_mask in self.cmd_mask_exception_list_:
            if cmd_mask == cmd:
                ret = True
                break
        return ret

    def insert_exception_cmd_mask(self, cmd):
        self.cmd_mask_exception_list_.append(cmd)
        log.info("insert [{}] to exception mask list!".format(cmd))

    def abnormal_report(self, actuator_name, level, msg, code):
        seq = self.exception_get_seq()
        report_dic = {
            'type': "rpt",
            'level': level,
            'module': actuator_name,
            'errmsg': msg,
            'seq': seq,
            'code': code,
            'stamp': int(time.time() * 1000)
        }

        report_json = JsonUtils.json_dumps(report_dic)
        self.abnormal_topic_.publish(kTopicException, report_json)
        log.error("abnormal report: {}".format(report_json))

    def get_config_unit(self, function_name, module_name, unit_name):
        return JsonUtils.get_config_unit(function_name, module_name, unit_name)

    def get_dict_key_value(self, dict_ins, key, value_type):
        return JsonUtils.get_dict_key_value(dict_ins, key, value_type)

    def json_loads(self, json_str):
        return JsonUtils.json_loads(json_str)

    def json_dumps(self, dict_str):
        return JsonUtils.json_dumps(dict_str)

    def get_params(self, cmd):
        return JsonUtils.get_params(cmd)

class JsonUtils:
    @staticmethod
    def get_config_unit(function_name, module_name, unit_name):
        is_exist = os.path.exists(config_json_file_path)
        if is_exist is False:
            log.error("can not find config file! path: {}".format(config_json_file_path))
            return None

        file_config = open(config_json_file_path, "r")
        json_value = file_config.read()

        config_dict_list = JsonUtils.json_loads(json_value)

        if config_dict_list is None:
            return None

        if isinstance(config_dict_list, list) is False:
            return None

        for config_dict in config_dict_list:
            if isinstance(config_dict, dict) is False:
                continue
            if 'name' not in config_dict:
                continue
            if function_name != config_dict.get('name'):
                continue
            if module_name not in config_dict:
                continue
            unit_dict_list = config_dict.get(module_name)

            if isinstance(unit_dict_list, list) is False:
                continue
            for unit_dict in unit_dict_list:
                if isinstance(unit_dict, dict) is True:
                    if 'name' in unit_dict:
                        if unit_name == unit_dict.get('name'):
                            return unit_dict
        return None

    # value_type: int, float, str, bool, list, dict
    @staticmethod
    def get_dict_key_value(dict_ins, key, value_type):
        if key in dict_ins:
            value = dict_ins.get(key)
            if isinstance(value, value_type) is False:
                value = None
        else:
            value = None
        return value

    @staticmethod
    def json_loads(json_str):
        try:
            strings = json.loads(json_str, encoding='utf-8')
        except:
            log.error("Parsing json string error! json_str: {}".format(json_str))
            return
        else:
            return strings

    @staticmethod
    def json_dumps(dict_str):
        try:
            # note [ensure_ascii=False] must be set, then utf-8 chinese code can be use
            json_str = json.dumps(dict_str, ensure_ascii=False)
        except:
            log.error("Dict to json error! dict: {}".format(dict_str))
            return
        else:
            return json_str

    @staticmethod
    def get_params(cmd):
        try:
            if 'params' in cmd:
                param_json = cmd.get('params')
        except:
            log.error("Cmd params error! cmd: {}".format(cmd))
            return
        else:
            param = JsonUtils.json_loads(param_json)
            return param


class Actuator:
    def __init__(self, name):
        self.name_ = name
        # async process handle thread
        self.adapter_ = None
        self.asyn_msg_queue_ = Queue.Queue(10)
        self.exception_msg_queue_ = Queue.Queue(10)
        self.data_condition_ = threading.Condition()
        self.is_exception_mode_ = False
        self.version_ = ""
        self.compiler_date_ = ""
        self.asyncCmdHandle_th_ = threading.Thread(target=Actuator.asyn_cmd_handle_thread, args=(self,))
        self.asyncCmdHandle_th_.setDaemon(True)
        self.asyncCmdHandle_th_.start()
        self.exCmdHandle_th_ = threading.Thread(target=Actuator.exception_cmd_handle_thread, args=(self,))
        self.exCmdHandle_th_.setDaemon(True)
        self.exCmdHandle_th_.start()

    def set_adapter(self, adapter_ins):
        self.adapter_ = adapter_ins

    def reply_result(self, msg, error_info, result):
        if self.adapter_ is not None:
            self.adapter_.reply_result(msg, error_info, result)

    def exception_report(self, module_name, msg, err_code):
        if self.adapter_ is not None:
            self.adapter_.exception_report(module_name, msg, err_code)

    def push_async_cmd(self, msg):
        self.asyn_msg_queue_.put(msg)

    def push_exception_cmd(self, msg):
        self.exception_msg_queue_.put(msg)

    def asyn_cmd_handle_thread(self):
        while True:
            # get cmd from async queue
            #
            msg = self.asyn_msg_queue_.get(True)
            is_has_handle = self.async_cmd_handle(msg)
            if is_has_handle is False:
                error_info = ErrorInfo(E_MOD_PARAM, "not support command")
                self.reply_result(msg, error_info, None)

    def set_exception_mode(self, is_exception):
        self.data_condition_.acquire()
        self.is_exception_mode_ = is_exception
        self.data_condition_.release()

    def is_exception_mode(self):
        self.data_condition_.acquire()
        is_exception = self.is_exception_mode_
        self.data_condition_.release()
        return is_exception

    def abnormal_report(self, level, msg, code):
        if self.adapter_ is not None:
            self.adapter_.abnormal_report(self.name_, level, msg, code)

    def set_version_info(self, version, compiler_date):
        self.data_condition_.acquire()
        self.version_ = version
        self.compiler_date_ = compiler_date
        self.data_condition_.release()

    def get_version_info(self):
        self.data_condition_.acquire()
        version = self.version_
        compiler_date = self.compiler_date_
        self.data_condition_.release()

        return version, compiler_date

    def insert_exception_cmd_mask(self, cmd):
        if self.adapter_ is not None:
            self.adapter_.insert_exception_cmd_mask(cmd)

    def exception_cmd_handle_thread(self):
        log.info("actuator exception msg handle thread running!")
        while True:
            # get cmd from exception queue
            #
            msg = self.exception_msg_queue_.get(True)
            error_code = E_OK
            if "exceptionreport" == msg.cmd:
                log.info("get {} command!".format(msg.cmd))
                self.set_exception_mode(True)
                is_success_handle = self.abort_handle()
                if not is_success_handle:
                    error_code = E_MOD_ABORT_FAILED
            elif "reset" == msg.cmd:
                log.info("get {} command!".format(msg.cmd))
                is_success_handle = self.reset_handle()
                if is_success_handle is False:
                    error_code = E_MOD_RESET_FAILED
                else:
                    self.set_exception_mode(False)
                error_info = ErrorInfo(error_code, "")
                self.adapter_.reply_result(msg.proxy_msg, error_info, None)
            else:
                log.error("exception unknow cmd!")

    def sync_cmd_handle(self, msg):
        log.info("[BASE] syncCmdHandle:{} handle!".format(msg.cmd))
        return True

    def async_cmd_handle(self, msg):
        log.info("[BASE] asyncCmdHandle:{} handle!".format(msg.cmd))
        return True

    def abort_handle(self):
        log.info("[BASE] abort handle!")
        return True

    def reset_handle(self):
        log.info("[BASE] reset handle!")
        return True
