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



class pose_estimation(object):

    def __init__(self):
        self.node=rospy.init_node("node_name", anonymous=False)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.target_link = 'base_link'
        self.source_link = 'camera_link'

        # Links of end effector are: 
        # 1- tool0_gripper_inner
        # 2- tool0_gripper_outer
        # 3- tool0_magnetic_tip
        # 4- tool0_suction_cup
        
        self.referred_link = "tool0_magnetic_tip"

        self._subscriber()

    def _subscriber(self):
        rospy.Subscriber('kinect_pose', Point, self._subscriber_info_callback,queue_size=1)
        rospy.spin()


    def _subscriber_info_callback(self,point):
        self.move_group.set_named_target('begin_to_look')
        self.move_group.go(wait=True)

        camera_to_base = self.tf_buffer.lookup_transform(self.target_link, self.source_link, rospy.Time(0))


        camera_point = geometry_msgs.msg.PoseStamped()
        camera_point.header.stamp = rospy.Time.now()
        camera_point.header.frame_id = self.source_link
        camera_point.pose.position.x = point.x #point.z
        camera_point.pose.position.y = point.y #-point.x
        camera_point.pose.position.z = point.z #point.y
        camera_point.pose.orientation.w = 1
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
        rospy.loginfo('Pose of the object in the world reference frame is:\n %s', camera_point)
        rospy.loginfo('Pose of the object in the logical camera reference frame is:\n %s', base_point)



if __name__== '__main__':

    pose_estimation()