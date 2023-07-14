#!/usr/bin/env python3

import sys
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import importlib.util
spec = importlib.util.spec_from_file_location("rospy", "/opt/ros/noetic/lib/python3/dist-packages/rospy/__init__.py")
foo = importlib.util.module_from_spec(spec)
sys.modules["rospy"] = foo
spec.loader.exec_module(foo)
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs.msg
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest, SetIOSignalResponse
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, PlanningOptions
import numpy as np
import tf2_ros
import tf2_geometry_msgs

import tf.transformations as tf
import std_msgs

def all_poses():
    # First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place',
                    anonymous=True)

    # Instantiate a MoveGroupCommander object.  This object is an interface
    # to one group of joints.  In this case the group refers to the joints of
    # robot1. This interface can be used to plan and execute motions on robot1.

    robot1_group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    
    robot1_group.set_max_acceleration_scaling_factor(1)
    robot1_group.set_max_velocity_scaling_factor(1)
    rospy.loginfo('Execute Trajectory server is available for robot1')

    robot1_group.set_planning_time(0.2)
    # robot1_group.set_num_planning_attempts(1)
    robot1_group.allow_replanning(True)  # allow for replanning if obstacles are encountered
    robot1_group.allow_looking(True)


    while not rospy.is_shutdown():
        success = False

        robot1_group.set_named_target("middle_right")
        while not success:
            current_state = np.asarray(robot1_group.get_current_joint_values())
            success=robot1_group.go(wait=True)
            current_time = rospy.Time.now()
            while np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values()))<0.01:
                robot1_group.go(wait=False)
                rospy.sleep(0.5)
                if np.abs((current_time - rospy.Time.now()).to_sec()) > 10:
                    success = True
                    print("EXITING, TIME LIMIT REACHED")
                    break

        rospy.loginfo("location 1---------------------middle_right")
        
        success = False

        robot1_group.set_named_target("front")
        while not success:
            current_state = np.asarray(robot1_group.get_current_joint_values())
            success=robot1_group.go(wait=True)
            current_time = rospy.Time.now()
            while np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values()))<0.01:
                robot1_group.go(wait=False)
                rospy.sleep(0.5)
                if np.abs((current_time - rospy.Time.now()).to_sec()) > 10:
                    success = True
                    print("EXITING, TIME LIMIT REACHED")
                    break

        rospy.loginfo("location 2---------------------front")


        success = False

        robot1_group.set_named_target("middle_left")
        while not success:
            current_state = np.asarray(robot1_group.get_current_joint_values())
            success=robot1_group.go(wait=True)
            current_time = rospy.Time.now()
            while np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values()))<0.01:
                robot1_group.go(wait=False)
                rospy.sleep(0.5)
                if np.abs((current_time - rospy.Time.now()).to_sec()) > 10:
                    success = True
                    print("EXITING, TIME LIMIT REACHED")
                    break

        rospy.loginfo("location 3---------------------middle_left")

        success = False

        robot1_group.set_named_target("front")
        while not success:
            current_state = np.asarray(robot1_group.get_current_joint_values())
            success=robot1_group.go(wait=True)
            current_time = rospy.Time.now()
            while np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values()))<0.01:
                robot1_group.go(wait=False)
                rospy.sleep(0.5)
                if np.abs((current_time - rospy.Time.now()).to_sec()) > 10:
                    success = True
                    print("EXITING, TIME LIMIT REACHED")
                    break

        rospy.loginfo("location 4---------------------front")

class hand_tracking(object):
    def __init__(self):

        # First initialize moveit_commander and rospy.
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('tracker',anonymous=True)


        # Instantiate a MoveGroupCommander object.  This object is an interface
        # to one group of joints.  In this case the group refers to the joints of
        # robot1. This interface can be used to plan and execute motions on robot1.

        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.set_max_velocity_scaling_factor(1)
        rospy.loginfo('Execute Trajectory server is available for robot1')

        self.move_group.set_planning_time(0.2)
        # robot1_group.set_num_planning_attempts(1)
        self.move_group.allow_replanning(True)  # allow for replanning if obstacles are encountered
        self.move_group.allow_looking(True)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot_client = actionlib.SimpleActionClient(
        'execute_trajectory',
        moveit_msgs.msg.ExecuteTrajectoryAction)
        self.robot_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()

        self.target_link = 'world'
        self.source_link = 'safety_color_optical_frame'
        self.move_group.set_pose_reference_frame(self.target_link)


        # Links of end effector are: 
        # 1- tool0_gripper_inner
        # 2- tool0_gripper_outer
        # 3- tool0_magnetic_tip
        # 4- tool0_suction_cup

        self.referred_link = 'tool0_gripper_outer'

        self.move_group.set_end_effector_link(self.referred_link)


        self.quaternions = tf.quaternion_from_euler(0,0,0)
        self.position = geometry_msgs.msg.PoseStamped()
        self.base_point = self.move_group.get_current_joint_values()
        self.subscriber = rospy.Subscriber('pose_estimation',geometry_msgs.msg.PoseStamped,self._callback,queue_size=1)
        rospy.sleep(1)



    def _callback(self,location):
        self.position = location
        # print(location)
        # print("callback")
        camera_to_base = self.tf_buffer.lookup_transform(self.target_link, self.source_link, rospy.Time(0))

        camera_point = geometry_msgs.msg.PoseStamped()
        camera_point.header.stamp = rospy.Time.now()
        camera_point.header.frame_id = self.source_link

        

        camera_point.pose.position.x = self.position.pose.position.x
        camera_point.pose.position.y = self.position.pose.position.y
        camera_point.pose.position.z = self.position.pose.position.z
        # camera_point.pose.orientation.x = self.quaternions[0]
        # camera_point.pose.orientation.y = self.quaternions[1]
        # camera_point.pose.orientation.z = self.quaternions[2]
        # camera_point.pose.orientation.w = self.quaternions[3]
        self.base_point = tf2_geometry_msgs.do_transform_pose(camera_point, camera_to_base)


    def track(self):

        reached = False
        # rospy.Subscriber('mouse_location',geometry_msgs.msg.PoseStamped,self._callback,queue_size=1)

        while not reached:
            success = False


            current_location = self.move_group.get_current_pose(self.referred_link)
            
            current_location = np.asarray([current_location.pose.position.x,
            current_location.pose.position.y,
            current_location.pose.position.z])
            

            point_received = np.asarray([self.base_point.pose.position.x,
            self.base_point.pose.position.y,
            self.base_point.pose.position.z])

            current_time = rospy.Time.now()
            
            while np.linalg.norm(current_location-point_received)<0.2:
                
                # rospy.Subscriber('mouse_location',geometry_msgs.msg.PoseStamped,self._callback,queue_size=1)
                
                point_received = np.asarray([self.base_point.pose.position.x,
                    self.base_point.pose.position.y,
                    self.base_point.pose.position.z])

                print(f'looped time: {np.abs((current_time - rospy.Time.now()).to_sec())}',end='\r')

                if np.abs((current_time - rospy.Time.now()).to_sec()) > 4:
                    reached = True
                    self.move_group.set_named_target('all_zero')
                    self.move_group.go(wait=True)
                    print("EXITING, REACHED HAND")
                    break


            if reached:
                break

            else:

                while not success:
                    # print(base_point)
                    # rospy.Subscriber('mouse_location',geometry_msgs.msg.PoseStamped,self._callback,queue_size=1)
                    self.move_group.set_position_target([self.base_point.pose.position.x
                        ,self.base_point.pose.position.y
                        ,self.base_point.pose.position.z])
                    current_state = np.asarray(self.move_group.get_current_joint_values())
                    success=self.move_group.go(wait=True)
                    

                    current_time = rospy.Time.now()
                    while np.linalg.norm(current_state - np.asarray(self.move_group.get_current_joint_values()))<0.01:
                        # rospy.Subscriber('mouse_location',geometry_msgs.msg.PoseStamped,self._callback,queue_size=1)
                        self.move_group.set_position_target([self.base_point.pose.position.x
                            ,self.base_point.pose.position.y
                            ,self.base_point.pose.position.z])
                        self.move_group.go(wait=False)

                        rospy.sleep(1)
                        print(f'waiting looped time: {np.abs((current_time - rospy.Time.now()).to_sec())}',end='\r')

                        if np.abs((current_time - rospy.Time.now()).to_sec()) > 15:
                            success = True
                            reached = True
                            print("EXITING, TIME LIMIT REACHED")
                            break



if __name__ == '__main__':
    try:
        # all_poses()
        hand_track = hand_tracking()
        hand_track.track()
    except rospy.ROSInterruptException:
        pass
