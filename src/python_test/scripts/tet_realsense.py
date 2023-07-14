#!/usr/bin/env python3
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs
from python_test.msg import realsense
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import *
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest, SetIOSignalResponse



iteration = 1

def cameraInitialPose():

    global iteration
    if iteration == 1:
        moveit_commander.roscpp_initialize(sys.argv)

        robot1_group = moveit_commander.MoveGroupCommander("gripper")


        robot1_client = actionlib.SimpleActionClient(
            'execute_trajectory',
            moveit_msgs.msg.ExecuteTrajectoryAction)
        robot1_client.wait_for_server()
        

        rospy.loginfo('Execute Trajectory server is available for robot1')

        robot1_group.set_named_target("horizontal_camera_pose_2")
        _, plan, _, _ = robot1_group.plan()
        robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        robot1_goal.trajectory = plan
        robot1_client.send_goal(robot1_goal)
        robot1_client.wait_for_result()
        iteration += 1
        delay = rospy.Rate(0.25)
        delay.sleep()


    #print("check 1")

    #moveit_commander.roscpp_initialize(sys.argv)

    #object_pose = geometry_msgs.msg.PoseStamped()
    #object_pose.header.stamp = rospy.Time.now()
    #object_pose.header.frame_id = "base_link"
    #object_pose.pose.position.x = 0.464
    #object_pose.pose.position.y = 0.029
    #object_pose.pose.position.z = 0.500
    
    #object_pose.pose.orientation.x = -0.012
    #object_pose.pose.orientation.y = 0.711
    #object_pose.pose.orientation.z = 0
    #object_pose.pose.orientation.w = 0.703

    #move_group = moveit_commander.MoveGroupCommander("camera")
    #move_group.set_pose_target(object_pose)

    # `go()` returns a boolean indicating whether the planning and execution was successful.
    #success = move_group.go(wait=True)
    #if success:
    #    print("\n\nSSSSSSSSSSSSSSSSSSSSSSSSUUUUUUUUUUCCCCCCCCCCCCCCCCEEEEEEEEEEESSS\n\n")
    # Calling `stop()` ensures that there is no residual movement
    #move_group.stop()
    #move_group.clear_pose_targets()



def subscriberInfoCallback(point):

    if point.name == "gear1":



        transformation_message = tf_buffer.lookup_transform('base_link', 'camera_link', rospy.Time())
        print(transformation_message)

        x = transformation_message.transform.translation.x
        y = transformation_message.transform.translation.y
        z = transformation_message.transform.translation.z

        x_rot = transformation_message.transform.rotation.x
        y_rot = transformation_message.transform.rotation.y
        z_rot = transformation_message.transform.rotation.z
        w_rot = transformation_message.transform.rotation.w
        
        cam_to_tip=[0.036, -0.010, -0.087]

        camera_point = np.array([
            [point.pose.position.z-0.047],
            [-point.pose.position.x+0.037],
            [point.pose.position.y+0.072],
            [1]])

        roll,pitch,yaw = euler_from_quaternion([x_rot,y_rot,z_rot,w_rot])
        cx=np.cos(roll);    cy=np.cos(np.pi/2);    cz=np.cos(yaw);
        sx=np.sin(roll);    sy=np.sin(np.pi/2);    sz=np.sin(yaw);
        print(roll)
        print(pitch)
        print(yaw)


        Homogenous_matrix=np.array([
            [(cx*cy)      ,(cx*sy*sz-sx*cz)    ,(cx*sy*cz+sx*sz)      ,(x)],
            [(sx*cy)      ,(sx*sy*sz+cx*cz)    ,(sx*sy*cz-cx*sz)      ,(y)],
            [(-sy)        ,(cy*sz)             ,(cy*cz)               ,(z)],
            [0            ,(0)                 ,(0)                   ,(1)]])



        p1 = np.matmul(Homogenous_matrix,camera_point)

        # Check if the logical camera has seen our box which has the name 'object'.
        # Create a pose stamped message type from the camera image topic.
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.stamp = rospy.Time.now()
        object_pose.header.frame_id = "base_link"
        object_pose.pose.position.x = p1[0][0]
        object_pose.pose.position.y = p1[1][0]
        object_pose.pose.position.z = p1[2][0]
        
        rot_ = quaternion_from_euler(0,1.5707,0)

        object_pose.pose.orientation.x = rot_[0]
        object_pose.pose.orientation.y = rot_[1]
        object_pose.pose.orientation.z = rot_[2]
        object_pose.pose.orientation.w = rot_[3]

        move(object_pose)

def move(object_pose):
    global iteration

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "camera"
    move_group = moveit_commander.MoveGroupCommander(group_name)
#    move_group.set_end_effector_link('link_6')




    while True:

       # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        try:

            object_world_pose = tf_buffer.transform(object_pose, "base_link")
            move_group.set_pose_target(object_world_pose)
            # `go()` returns a boolean indicating whether the planning and execution was successful.
            success = move_group.go(wait=True)

            if success:
                print("-----------------\nSUCCESS\n-----------------")
                iteration +=1
            # Calling `stop()` ensures that there is no residual movement
            move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets().
            move_group.clear_pose_targets()
            #if iteration == 3:
            delay = rospy.Rate(0.5)
            delay.sleep()
            proxy = rospy.ServiceProxy('/rws/set_io_signal', SetIOSignal)
            proxy(signal='valve',value='0')

            #moveit_commander.roscpp_shutdown()


            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("-----------------\nEXCEPTION\n-----------------")
            continue
    print("Successfully Transformed Pose")

    #rospy.signal_shutdown('Successfully transformed pose.')


if __name__== '__main__':
  # Initialize ROS node to transform object pose.
  rospy.init_node('transform_object_pose',
                    anonymous=True)
  rospy.wait_for_service('/rws/set_io_signal')
  proxy = rospy.ServiceProxy('/rws/set_io_signal', SetIOSignal)
  proxy(signal='valve',value='1')


  cameraInitialPose()

  
  # Declate a TF buffer globally.
  tf_buffer = tf2_ros.Buffer(rospy.Duration(0))
  tf_listener = tf2_ros.TransformListener(tf_buffer)

  # Subscribe to the logical camera topic.
  rospy.Subscriber('/topic_1', realsense, subscriberInfoCallback,queue_size=1)
  rospy.spin()