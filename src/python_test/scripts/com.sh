#!/usr/bin/env bash
ssh amir-ubuntu
roscore
ssh amir-ubuntu
export ROS_MASTER_URI=http://192.168.1.11:11311
rosrun rospy_tutorials talker.py
