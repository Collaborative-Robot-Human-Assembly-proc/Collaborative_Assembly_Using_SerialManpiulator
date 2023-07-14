#!/usr/bin/env python3

import subprocess 
import rospy
import std_msgs
import signal
import os

p = 0

def callback(data):
    if data.data == 1:
        global p
        p=subprocess.run("roslaunch robot2_moveit_config demo_hw.launch ",shell=True)

        print(p)
    elif data.data == 0:
        print(p)
        p.send_signal(signal.SIGTERM)

if __name__ == '__main__':


    try:

        rospy.init_node('hardware_intializer')
        rospy.Subscriber('/hardware',std_msgs.msg.Int16,callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
