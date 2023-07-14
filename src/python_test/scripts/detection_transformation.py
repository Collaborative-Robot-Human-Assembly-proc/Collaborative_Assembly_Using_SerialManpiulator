#!/usr/bin/env python3
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

from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest, SetIOSignalResponse
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
        self.move_group.set_max_acceleration_scaling_factor(0.02) #0.02
        self.move_group.set_max_velocity_scaling_factor(0.02)
        self.service = rospy.ServiceProxy('realsense_service', RealSense)

        self.target_link = 'world'
        self.source_link = 'camera_color_optical_frame'
        rospy.wait_for_service('/rws/set_io_signal')
        self.proxy = rospy.ServiceProxy('/rws/set_io_signal', SetIOSignal)

        # Links of end effector are: 
        # 1- tool0_gripper_inner
        # 2- tool0_gripper_outer
        # 3- tool0_magnetic_tip
        # 4- tool0_suction_cup

        self.referred_link = "tool0_gripper_outer"


        self.quaternions = tf.quaternion_from_euler(0,-np.pi/2,np.pi)







    def begin_to_detect(self):

        self.move_group.set_named_target('begin_to_look')
        self.move_group.go(wait=True)
        self.proxy(signal='valve',value='0')
        rospy.sleep(3)

        point_response = self.service(1)
        self.target_link = 'world'
        
        self.move_group.set_pose_reference_frame(self.target_link)
        self.move_group.set_end_effector_link(self.referred_link)

        camera_to_base = self.tf_buffer.lookup_transform(self.target_link, self.source_link, rospy.Time(0))

        camera_point = geometry_msgs.msg.PoseStamped()
        camera_point.header.stamp = rospy.Time.now()
        camera_point.header.frame_id = self.source_link

        # print(point_response.location)
        camera_point.pose.position.x = point_response.location.pose.position.x
        camera_point.pose.position.y = point_response.location.pose.position.y
        camera_point.pose.position.z = point_response.location.pose.position.z
        camera_point.pose.orientation.x = self.quaternions[0]
        camera_point.pose.orientation.y = self.quaternions[1]
        camera_point.pose.orientation.z = self.quaternions[2]
        camera_point.pose.orientation.w = self.quaternions[3]
        base_point = tf2_geometry_msgs.do_transform_pose(camera_point, camera_to_base)
        print(base_point)

        try:
            if (np.absolute(point_response.location.pose.position.x)>0 or \
                np.absolute(point_response.location.pose.position.y)>0 or \
                np.absolute(point_response.location.pose.position.z)>0):
                # if self.referred_link == 'tool0_gripper_inner':
                #     self.move_group.set_named_target('all_zero')
                #     self.move_group.go(wait=True)
                
                # elif self.referred_link == 'tool0_gripper_outer':
                #     pass
                
                # elif self.referred_link == 'tool0_magnetic_tip':
                #     self.move_group.set_named_target('begin_magnetic_tip')
                #     self.move_group.go(wait=True)
                
                # elif self.referred_link == 'tool0_suction_cup':
                #     self.move_group.set_named_target('begin_suction_cup')
                #     self.move_group.go(wait=True)


                new_eef_pose = geometry_msgs.msg.Pose()
                new_eef_pose.orientation =  copy.deepcopy(base_point.pose.orientation)
                new_eef_pose.position =  copy.deepcopy(base_point.pose.position)
                new_eef_pose.position.z+=0.17



                self.move_group.set_pose_target(new_eef_pose)

                success = self.move_group.go(wait=True)




                new_eef_pose.position =  copy.deepcopy(base_point.pose.position)
                # print(new_eef_pose)

                # x,y --> ompl / z --> pilz (LIN)
                # (ompl, pilz_industrial_motion_planner)
                # (RRTConnect, LIN)
                self.move_group.set_planning_pipeline_id('pilz_industrial_motion_planner')
                self.move_group.set_planner_id('LIN')

                self.move_group.set_pose_target(new_eef_pose)

                success = self.move_group.go(wait = True)
                if success:
                    print("-----------------\nSUCCESS\n-----------------")
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                rospy.sleep(1)
                self.proxy(signal='valve',value='1')

                
                new_eef_pose.position =  copy.deepcopy(base_point.pose.position)
                new_eef_pose.position.z+=0.20
                
                self.move_group.set_pose_target(new_eef_pose)
                success = self.move_group.go(wait=True)
                
                
                self.move_group.set_planning_pipeline_id('ompl')
                self.move_group.set_planner_id('RRTConnect')
                

                self.move_group.set_named_target('middle_right')
                self.move_group.go(wait=True)
                rospy.sleep(3)
                self.proxy(signal='valve',value='0')


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("-----------------\nEXCEPTION\n-----------------")
        # rospy.loginfo('Pose of the object in the world reference frame is:\n %s', camera_point)
        # rospy.loginfo('Pose of the object in the logical camera reference frame is:\n %s', base_point)


if __name__== '__main__':

    r=realsense()
    try:
        while not rospy.is_shutdown():
            idx = int(input("Enter what frame: "))
            if idx == 1:
                r.referred_link = 'tool0_gripper_outer'
            if idx == 2:
                r.referred_link = 'tool0_gripper_inner'
            if idx == 3:
                r.referred_link = 'tool0_magnetic_tip'
            if idx == 4:
                r.referred_link = 'tool0_suction_cup'
            else:
                pass
            r.begin_to_detect()

    except (rospy.service.ServiceException, rospy.ROSInterruptException):
        moveit_commander.roscpp_shutdown()
        print('stop')
