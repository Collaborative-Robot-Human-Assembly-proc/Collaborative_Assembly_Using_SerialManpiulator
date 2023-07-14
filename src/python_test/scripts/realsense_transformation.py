#!/usr/bin/env python3
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs
from python_test.srv import RealSense, RealSenseRequest, RealSenseResponse

import numpy as np
import tf.transformations as tf

class realsense(object):

    def __init__(self):
        self.node=rospy.init_node("rs_rs", anonymous=True)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_max_acceleration_scaling_factor(0.02)
        self.move_group.set_max_velocity_scaling_factor(0.02)
        self.service = rospy.ServiceProxy('realsense_service', RealSense)

        self.target_link = 'base_link'
        self.source_link = 'camera_depth_optical_frame'

        # Links of end effector are: 
        # 1- tool0_gripper_inner
        # 2- tool0_gripper_outer
        # 3- tool0_magnetic_tip
        # 4- tool0_suction_cup

        self.referred_link = "tool0_gripper_outer"



    def begin_to_detect(self):
        self.move_group.set_named_target('begin_to_look')
        self.move_group.go(wait=True)

        point_response = self.service(1)


        camera_to_base = self.tf_buffer.lookup_transform(self.target_link, self.source_link, rospy.Time(0))




        camera_point = geometry_msgs.msg.PoseStamped()
        camera_point.header.stamp = rospy.Time.now()
        camera_point.header.frame_id = self.source_link
        pitch = np.deg2rad(-90)
        yaw = np.deg2rad(90)
        quat = tf.quaternion_from_euler(0, pitch, yaw)
        print(point_response.location)
        if point_response.location.header.frame_id =="gear":
            camera_point.pose.position.x = point_response.location.pose.position.x
            camera_point.pose.position.y = point_response.location.pose.position.y
            camera_point.pose.position.z = point_response.location.pose.position.z
            camera_point.pose.orientation.x = quat[0]
            camera_point.pose.orientation.y = quat[1]
            camera_point.pose.orientation.z = quat[2]
            camera_point.pose.orientation.w = quat[3]
            base_point = tf2_geometry_msgs.do_transform_pose(camera_point, camera_to_base)
    


            try:

                self.move_group.set_end_effector_link(self.referred_link)
                # self.move_group.set_named_target('begin_magnetic_tip')
                # self.move_group.go(wait=True)



                self.move_group.set_pose_target(base_point)
                success = self.move_group.go(wait=True)
                if success:
                    print("-----------------\nSUCCESS\n-----------------")
                self.move_group.stop()
                self.move_group.clear_pose_targets()


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("-----------------\nEXCEPTION\n-----------------")
            # rospy.loginfo('Pose of the object in the world reference frame is:\n %s', camera_point)
            # rospy.loginfo('Pose of the object in the logical camera reference frame is:\n %s', base_point)

        else:
            pass


if __name__== '__main__':

    r=realsense()
    while not rospy.is_shutdown():
        r.begin_to_detect()