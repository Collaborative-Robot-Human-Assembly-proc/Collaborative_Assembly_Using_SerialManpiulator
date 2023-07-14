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
from python_test.msg import realsense
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion,quaternion_from_euler

def subscriberInfoCallback(point):
    transformation_message = tf_buffer.lookup_transform('base_link', 'camera_link', rospy.Time())
    print(transformation_message)
    x = transformation_message.transform.translation.x
    y = transformation_message.transform.translation.y
    z = transformation_message.transform.translation.z

    x_rot = transformation_message.transform.rotation.x
    y_rot = transformation_message.transform.rotation.y
    z_rot = transformation_message.transform.rotation.z
    w_rot = transformation_message.transform.rotation.w

    camera_point = np.array([
        [point.x],
        [point.y],
        [point.z],
        [1]])
    roll,pitch,yaw = euler_from_quaternion([x_rot,y_rot,z_rot,w_rot])
    cx=np.cos(roll);    cy=np.cos(pitch);    cz=np.cos(yaw);
    sx=np.sin(roll);    sy=np.sin(pitch);    sz=np.sin(yaw);


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
    object_pose.pose.position.x = p1[0][0] #message.pose.position.z
    object_pose.pose.position.y = p1[1][0]
    object_pose.pose.position.z = p1[2][0] #message.pose.position.y
    rot_ = quaternion_from_euler(0,0,0)
    print(rot_)
    object_pose.pose.orientation.x = rot_[0]
    object_pose.pose.orientation.y = rot_[1]
    object_pose.pose.orientation.z = rot_[2]
    object_pose.pose.orientation.w = rot_[3]

    move(object_pose)


def move(object_pose):

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "camera"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    



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
                print("\n\nSSSSSSSSSSSSSSSSSSSSSSSSUUUUUUUUUUCCCCCCCCCCCCCCCCEEEEEEEEEEESSS\n\n")
            # Calling `stop()` ensures that there is no residual movement
            move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets().
            move_group.clear_pose_targets()
            #moveit_commander.roscpp_shutdown()


            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("\nASASDASDAS\nAKSNDOANSD\n-----------------")
            continue
    rospy.loginfo('Pose of the object in the world reference frame is:\n %s', object_world_pose)
    rospy.loginfo('Pose of the object in the logical camera reference frame is:\n %s', object_pose)
    #   rospy.signal_shutdown('Successfully transformed pose.')


if __name__== '__main__':
  # Initialize ROS node to transform object pose.
  rospy.init_node('transform_object_pose',
                    anonymous=True)
  
  # Declate a TF buffer globally.
  tf_buffer = tf2_ros.Buffer(rospy.Duration(0))
  tf_listener = tf2_ros.TransformListener(tf_buffer)

  # Subscribe to the logical camera topic.
  rospy.Subscriber('kp', Point, subscriberInfoCallback,queue_size=1)

  rospy.spin()
