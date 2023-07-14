#!/usr/bin/env python3

import rospy
import moveit_commander

import sys

import numpy as np
import random
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

def collision_1():
	# Initialize a node
	rospy.init_node('add_collision_object_node', anonymous=True)

	moveit_commander.roscpp_initialize(sys.argv)

	scene = moveit_commander.PlanningSceneInterface()
	rate = rospy.Rate(10)
	# Define the pose of the box
	i = -0.3
	c = "left"
	while True:
		
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = "base_link"   # Replace "base_link" with your robot's base frame ID
		box_pose.pose.position.x = 0.4 #random.uniform(0.1, 0.5)   # Replace with the x position of the box
		
		if c == "left":
			i+=0.00025
			if i > 0.3:
				c = "right"
		elif c == "right":
			i-=0.00025
			if i < -0.3:
				c="left"
		
		box_pose.pose.position.y = i    # Replace with the y position of the box
		box_pose.pose.position.z =  0.3 #random.uniform(-0.2, 0.5)     # Replace with the z position of the box
		box_pose.pose.orientation.w = 1.0

		# Define the size of the box
		box_size = (0.05, 0.05, 0.5)   # Replace with the size of the box in meters

		# Add the box to the planning scene
		scene.add_box("box1", box_pose, box_size)
		#scene.add_box("box2", box_pose2, box_size)
		#rate.sleep()
	# Get the names of all the known objects in the scene
	object_names = scene.get_known_object_names()
	print(object_names)




def collision_2():
	# Initialize a node
	rospy.init_node('add_collision_object_node', anonymous=True)

	# Create a planning scene interface instance
	scene = moveit_commander.PlanningSceneInterface()
	rate = rospy.Rate(10000)
	# Define the pose of the box
	i = -0.3
	c = "left"
	while True :
		
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = "base_link"   # Replace "base_link" with your robot's base frame ID
		box_pose.pose.position.x = 0.4 #random.uniform(0.1, 0.5)   # Replace with the x position of the box
		
		if c == "left":
			i+=0.0005
			if i > 0.5:
				c = "right"
		elif c == "right":
			i-=0.0005
			if i < -0.5:
				c="left"
		
		box_pose.pose.position.y = i    # Replace with the y position of the box
		box_pose.pose.position.z =  0.3 #random.uniform(-0.2, 0.5)     # Replace with the z position of the box
		box_pose.pose.orientation.w = 1.0

		box_pose2 = geometry_msgs.msg.PoseStamped()
		box_pose2.header.frame_id = "base_link"   # Replace "base_link" with your robot's base frame ID
		box_pose2.pose.position.x = 0.4    # Replace with the x position of the box
		box_pose2.pose.position.y = 0   # Replace with the y position of the box
		box_pose2.pose.position.z = 0.5+i    # Replace with the z position of the box
		box_pose2.pose.orientation.w = 1.0
		# Define the size of the box
		box_size = (0.05, 0.05, 0.5)   # Replace with the size of the box in meters
		box_size2 = (0.05, 0.5, 0.05)   # Replace with the size of the box in meters

		# Add the box to the planning scene
		scene.add_box("box1", box_pose, box_size)
		scene.add_box("box2", box_pose2, box_size2)
		rate.sleep()
	# Get the names of all the known objects in the scene
	object_names = scene.get_known_object_names()
	print(object_names)



if __name__ == '__main__':
    try:
        # collision_1()
        collision_2()
    except rospy.ROSInterruptException:
        pass