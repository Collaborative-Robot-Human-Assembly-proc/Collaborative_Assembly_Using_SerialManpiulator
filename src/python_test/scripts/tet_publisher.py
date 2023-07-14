#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

import numpy as np
import random
from python_test.srv import RealSense, RealSenseRequest, RealSenseResponse

class publishers(object):
    def __init__(self):
        print("Publishers node initiated")
    def pub1(self):
        simple_publisher = rospy.Publisher('kinect_pose', Point, queue_size=1)
        rospy.init_node('node_1', anonymous=False)
        rate = rospy.Rate(10)

        # The string to be published on the topic
        point=Point()


        point.x=float(input("x: "))
        point.y=float(input("y: "))
        point.z=float(input("z: "))
        simple_publisher.publish(point)
        print("sending info...")

        rate.sleep()

    def pub2(self):
        simple_publisher = rospy.Publisher('kinect_pose', Point, queue_size=1)
        rospy.init_node('node_1', anonymous=False)
        rate = rospy.Rate(10)

        # The string to be published on the topic
        point=Point()
        
        point.x=random.uniform(-0.1, 0.1)
        point.y=random.uniform(-0.1, 0.1)
        point.z=random.uniform(0.5, 0.7)    

        simple_publisher.publish(point)
        print("x:{}\ty:{}\tz:{}".format(point.x,point.y,point.z))
        rate.sleep()




    def pub3(self):

        rospy.init_node('node_1', anonymous=False)
        rospy.wait_for_service('object_detection_service')
        proxy = rospy.ServiceProxy('object_detection_service', RealSense)

        request = RealSenseRequest()
        rate = rospy.Rate(10)

        # The string to be published on the topic

        request.location.pose.position.x=float(input("x: "))
        request.location.pose.position.y=float(input("y: "))
        request.location.pose.position.z=float(input("z: "))

        print(request.location)  

        response = proxy(request)
        rospy.signal_shutdown("Response sent, shutting down node")


        #print(response.feedback)


if __name__ == '__main__':
    publishers=publishers()
    # publishers.pub3()

    try:
        while not rospy.is_shutdown():
            publishers.pub1()
    except rospy.ROSInterruptException:
        pass
