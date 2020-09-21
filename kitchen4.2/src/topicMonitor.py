#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rostopic, rosnode
from sensor_msgs.msg import LaserScan
import roslaunch, rospkg, os
import subprocess
import sys
import time
abs_file = os.path.abspath(os.path.dirname(__file__))
sys.path.append(abs_file + "/../../../lib/comm")
sys.path.append(abs_file + "/../../../lib/log")
sys.path.append(abs_file + "/../lib/log")
sys.path.append(abs_file + "/../lib/comm")
from rlog import rlog
log = rlog()
log.setModuleName("topicMonitor")
log.set_priority("info")

topic = ''
node = ''
rate_threshold = 0
retry_time = 0
rt = None
# waiting_time = 0
relaunch_list = []
monitor_time = ''

def relaunchStatistics():
    launch = {'Id': 0, 'Time': ''}
    if relaunch_list == []:
        launch['Time'] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        relaunch_list.append(launch)
    else:
        launch['Id'] = relaunch_list[-1]['Id'] + 1
        launch['Time'] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        relaunch_list.append(launch)
    log.warn("Relaunch statistics from {} is:".format(monitor_time))
    for launch in relaunch_list:
        log.warn("{}".format(launch))

def getNvidiaStatus():
    cmd = "nvidia-smi -L"
    r = os.popen(cmd)
    text = r.read()
    r.close()
    # print(text)
    return text

def checkTopic():
    rospy.rostime.wallsleep(1.0)
    rate = rt.get_hz(topic)
    if rate == None:
        log.warn("No message in topic {}".format(topic))
        return False
    if rate[0] < rate_threshold:
        log.warn("Topic {} hz is {} < {}".format(topic, rate[0], rate_threshold))
        return False
    else:
        log.info("Topic {} hz is {} > {}".format(topic, rate[0], rate_threshold))
        return True

def relaunch():
    subprocess.Popen("ps aux | grep \"bash -c\" | grep kinect2_bridge.launch | awk \'{print $2}\' | xargs kill -9", shell=True)
    log.warn("Waiting 15s to relaunching kinect")
    rospy.rostime.wallsleep(15)
    subprocess.Popen("gnome-terminal -- bash -c \"roslaunch kinect2_bridge kinect2_bridge.launch ; exec bash;\"", shell=True)
    # subprocess.Popen("ps aux | grep \"bash -c\" | grep usb_cam-test.launch | awk \'{print $2}\' | xargs kill -9", shell=True)
    # log.warn("Waiting 15s to relaunching usb_cam")
    # rospy.rostime.wallsleep(15)
    # subprocess.Popen("gnome-terminal -- bash -c \"roslaunch usb_cam usb_cam-test.launch ; exec bash;\"", shell=True)
    relaunchStatistics()

if __name__ == '__main__':
    delay = 0
    monitor_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    rospy.init_node("topic_monitor", anonymous=False)
    topic = rospy.get_param("~topic","/kinect2/qhd/image_color_rect")
    node = rospy.get_param("~node","/kinect2_bridge")    
    # topic = rospy.get_param("~topic","/usb_cam/image_raw")
    # node = rospy.get_param("~node","/usb_cam")
    rate_threshold = rospy.get_param("~rate_threshold",1)
    retry_time = rospy.get_param("~retry_time",2)
    waiting_time = rospy.get_param("~waiting_time",5)

    rt = rostopic.ROSTopicHz(window_size=-1, filter_expr=None, use_wtime=False)
    while rostopic.get_topic_class(topic, blocking=False) == (None, None, None) and not rospy.is_shutdown(): # pause hz until topic is published
        log.warn("Topic {} has not been published! Try to relaunch!".format(topic))
        relaunch()
        rospy.rostime.wallsleep(waiting_time)
        
    rospy.Subscriber(topic, rospy.AnyMsg, rt.callback_hz, callback_args=topic, tcp_nodelay=False)

    while not rospy.is_shutdown():
        if not checkTopic():
            delay = delay + 1
            log.warn("{}".format(getNvidiaStatus()))
            log.warn("Topic {} delayed!".format(topic))
            if delay > retry_time:
                relaunch()
                delay = 0
            rospy.rostime.wallsleep(1.0)
        else:
            # rospy.rostime.wallsleep(1.0)
            delay = 0
