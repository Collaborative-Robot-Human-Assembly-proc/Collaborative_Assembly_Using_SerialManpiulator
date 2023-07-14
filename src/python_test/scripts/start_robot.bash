#!/usr/bin/env bash
#source /opt/ros/noetic/setup.bash
#source /home/amirwael/trial_2_ws/devel/setup.bash
rosservice call /rws/set_motors_on "{}" 
rosservice call /rws/pp_to_main "{}"
rosservice call /rws/start_rapid "{}"
rosrun python_test egm_config.py
rosservice call /rws/sm_addin/start_egm_joint "{}" 
