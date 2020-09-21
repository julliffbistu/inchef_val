#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rostopic, rosnode
import roslaunch, rospkg, os
import subprocess
# from std_srvs.srv import Empty, EmptyResponse
# from kitchen.srv import Nodelist, NodelistResponse
import psutil
import time
import operator
import sys

abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../lib/log")
sys.path.append(abs_file + "/../lib/comm")
from rlog import rlog
log = rlog()
log.setModuleName("maintenanceNode")
log.set_priority("info")
version = '4.2.1.0'

class Process:
    def __init__(self):
        self.pid = 0
        self.name = ''
        self.cmdline = ''
        self.cpu_percent = 0
        self.mem_percent = 0

class Maintenance():
    def __init__(self):
        rospy.init_node("maintenance_node", anonymous=False)
        rospy.set_param("~version", version)
        # rospy.Service("~get_nodelist", Nodelist, self.getNodelistCallback)
        # rospy.Service("~get_topsysload", Empty, self.getTopSysloadCallback)
        rospy.Timer(rospy.Duration(60.0), self.timerCallback)
        self.getNodelist()

    def timerCallback(self,event):
        log.info("Total cpu percent = {}".format(psutil.cpu_percent(percpu=True)))
        log.info("Total mem percent = {}".format(psutil.virtual_memory().percent))
        log.info("Total swp percent = {}".format(psutil.swap_memory().percent))
        log.info("------------")

    def getNodelist(self):
        log.info("Waiting 30s to get nodes list")
        rospy.rostime.wallsleep(30.0)
        node_list = rosnode.get_node_names()
        log.info("All nodes list: {}".format(node_list))
        log.info("------------")

    # def getNodelistCallback(self, req):
    #     node_list = rosnode.get_node_names()
    #     log.info("All nodes list: {}".format(node_list))
    #     log.info("------------")
    #     # return EmptyResponse()
    #     rep = NodelistResponse()
    #     rep.nodes = node_list
    #     return rep

    # def getTopSysloadCallback(self,req):
    #     pids = psutil.pids()
    #     processes = [psutil.Process(pid) for pid in pids]
    #     for p in processes:
    #         p.cpu_percent()
    #     time.sleep(1.0)
    #     process_list = []
    #     for process in processes:
    #         p = Process()
    #         p.name = process.name()
    #         p.cmdline = process.cmdline()
    #         p.cpu_percent = process.cpu_percent()
    #         p.mem_percent = process.memory_percent()
    #         process_list.append(p)
    #     cmpkey = operator.attrgetter("cpu_percent")
    #     process_list.sort(key=cmpkey, reverse=True)
    #     for p in process_list[:10]:
    #         if len(p.cmdline) >= 1:
    #             log.info("cpu_percent={:.2f}, mem_percent={:.2f}, cmdline={}, name={}".format(p.cpu_percent, p.mem_percent, p.cmdline[0], p.name))
    #         else:
    #             log.info("cpu_percent={:.2f}, mem_percent={:.2f}, cmdline=None, name={}".format(p.cpu_percent, p.mem_percent, p.name))
    #     log.info("------------")
    #     return EmptyResponse()

if __name__ == '__main__':
    Maintenance()
    rospy.spin()
    
