#!/usr/bin/env bash
#source /opt/ros/noetic/setup.bash
#source /home/amirwael/trial_2_ws/devel/setup.bash
rosservice call /rws/sm_addin/stop_egm "{}"
rosservice call /rws/stop_rapid "{}"
rosservice call /rws/set_motors_off "{}" 
