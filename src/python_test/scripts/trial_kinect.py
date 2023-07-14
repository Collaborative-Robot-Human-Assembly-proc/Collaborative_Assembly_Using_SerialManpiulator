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

        self._dummy_function()

    def _dummy_function(self):
        self._subscriber()
        rospy.spin()


    def _subscriber(self):
        print("mm")
        rospy.Subscriber('kinect_pose', Point, self._subscriber_info_callback,queue_size=1)

    def _subscriber_info_callback(self,point):

        
      # Check if the logical camera has seen our box which has the name 'object'.
        # Create a pose stamped message type from the camera image topic.
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.stamp = rospy.Time.now()
        object_pose.header.frame_id = "base_link"
        object_pose.pose.position.x = point.x #point.z
        object_pose.pose.position.y = point.y #-point.x
        object_pose.pose.position.z = point.z #point.y
        #object_pose.pose.orientation.w = 1




        try:
            self.object_world_pose = self.tf_buffer.transform(object_pose, "base_link")
            self.move_group.set_pose_target(self.object_world_pose)
            # `go()` returns a boolean indicating whether the planning and execution was successful.
            success = self.move_group.go(wait=True)
            if success:
                print("-----------------\nSUCCESS\n-----------------")
            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets().
            self.move_group.clear_pose_targets()
            #moveit_commander.roscpp_shutdown()


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("-----------------\nEXCEPTION\n-----------------")
        #rospy.loginfo('Pose of the object in the world reference frame is:\n %s', self.object_world_pose)
        #rospy.loginfo('Pose of the object in the logical camera reference frame is:\n %s', object_pose)



if __name__== '__main__':

    pose_estimation()