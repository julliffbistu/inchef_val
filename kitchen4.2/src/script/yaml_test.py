# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher('/rail/position_temp', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = [1.0,1.0,0.0]
        hello = Float32MultiArray(data = hello_str)

        pub.publish(hello)
        print(hello)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass