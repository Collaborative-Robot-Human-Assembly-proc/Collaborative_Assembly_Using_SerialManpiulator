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
def move_go():
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

    # robot1_group.set_planning_time(0.1)
    # robot1_group.set_num_planning_attempts(1)
    # robot1_group.allow_replanning(True)  # allow for replanning if obstacles are encountered
    robot1_group.allow_looking(True)


    while not rospy.is_shutdown():
        success = False

        robot1_group.set_named_target("middle_right")
        while success==False:
            success=robot1_group.go(wait=True)
        rospy.loginfo("location 1---------------------middle_right")
        
        success = False
        
        robot1_group.set_named_target("middle_left")
        while success==False:

            success=robot1_group.go(wait=True)
        rospy.loginfo("location 2---------------------middle_left")

def move_go_2():
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

    
    robot1_group.set_max_acceleration_scaling_factor(0.02)
    robot1_group.set_max_velocity_scaling_factor(1)
    rospy.loginfo('Execute Trajectory server is available for robot1')

    robot1_group.set_planning_time(0.1)
    # robot1_group.set_num_planning_attempts(1)
    robot1_group.allow_replanning(True)  # allow for replanning if obstacles are encountered
    robot1_group.allow_looking(True)


    while not rospy.is_shutdown():
        success = False

        robot1_group.set_named_target("middle_right")
        print(robot1_group.get_current_joint_values())
        while success==False:
            success=robot1_group.go(wait=True)
        rospy.loginfo("location 1---------------------middle_right")
        
        success = False
        
        robot1_group.set_named_target("front")
        while success==False:

            success=robot1_group.go(wait=True)
        rospy.loginfo("location 2---------------------front")

        success = False
        
        robot1_group.set_named_target("middle_left")
        while success==False:

            success=robot1_group.go(wait=True)
        rospy.loginfo("location 3---------------------middle_left")

        success = False
        
        robot1_group.set_named_target("front")
        while success==False:

            success=robot1_group.go(wait=True)
        rospy.loginfo("location 4---------------------front")

def move_go_3():
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

    
    robot1_group.set_max_acceleration_scaling_factor(0.2)
    robot1_group.set_max_velocity_scaling_factor(1)
    rospy.loginfo('Execute Trajectory server is available for robot1')

    robot1_group.set_planning_time(0.1)
    # robot1_group.set_num_planning_attempts(1)
    robot1_group.allow_replanning(True)  # allow for replanning if obstacles are encountered
    robot1_group.allow_looking(True)



    success = False

    robot1_group.set_named_target("middle_right")
    print(robot1_group.get_current_joint_values())
    while success==False:
            success=robot1_group.go(wait=True)
    rospy.loginfo("location 1---------------------middle_right")
        
    success = False
        
    robot1_group.set_named_target("front")
    while success==False:

            success=robot1_group.go(wait=True)
    rospy.loginfo("location 2---------------------front")

    success = False
        
    robot1_group.set_named_target("middle_left")
    while success==False:

            success=robot1_group.go(wait=True)
    rospy.loginfo("location 3---------------------middle_left")

    success = False
        
    robot1_group.set_named_target("front")
    while success==False:

            success=robot1_group.go(wait=True)
    rospy.loginfo("location 4---------------------front")
############################################################################################################################

def move_action():
    # First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place',
                    anonymous=True)


    robot1_group = moveit_commander.MoveGroupCommander("manipulator")
    planning_options = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()


    # Action clients to the ExecuteTrajectory action server.
    robot1_client = actionlib.SimpleActionClient(
        'execute_trajectory',
        moveit_msgs.msg.ExecuteTrajectoryAction)
    robot1_client.wait_for_server()
    
    robot1_group.set_max_acceleration_scaling_factor(0.1)
    robot1_group.set_max_velocity_scaling_factor(0.1)
    rospy.loginfo('Execute Trajectory server is available for robot1')

    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()

    planning_options = moveit_msgs.msg.PlanningOptions()
    # planning_options.planning_scene_monitor = True
    # planning_options.planning_scene_monitor_options = moveit_msgs.msg.PlanningSceneMonitorOptions()
    # planning_options.planning_scene_monitor_options.max_allowed_dt = rospy.Duration(0.1)
    planning_options.plan_only = False
    planning_options.replan = True
    planning_options.replan_attempts = 10
    planning_options.replan_delay = 5  # Set the delay before replanning to 2 seconds



    # client = actionlib.SimpleActionClient('/move_group/', moveit_msgs.msg.MoveGroupAction)
    # client.wait_for_server()


    # # # Set the planning time and preemption time in the planning options
    # goal = MoveGroupGoal()
    # # # planning_options = moveit_msgs.msg.PlanningOptions()
    # # # planning_options.planning_scene_monitor = True
    # # # planning_options.planning_scene_monitor_options = moveit_msgs.msg.PlanningSceneMonitorOptions()
    # # # planning_options.planning_scene_monitor_options.max_allowed_dt = rospy.Duration(0.1)
    # goal.planning_options.plan_only = False
    # goal.planning_options.look_around = True
    # goal.planning_options.look_around_attempts = 1
    # goal.planning_options.max_safe_execution_cost = 0
    # goal.planning_options.replan = True
    # goal.planning_options.replan_attempts = 1
    # goal.planning_options.replan_delay = 0 # Set the delay before replanning to 2 seconds
    # client.send_goal(goal)

    
    while not rospy.is_shutdown():

        robot1_group.set_named_target("middle_right")
        _, plan, _, _ = robot1_group.plan()
        robot1_goal.trajectory = plan
        robot1_client.send_goal(robot1_goal)
        success = robot1_client.wait_for_result()
        rospy.loginfo("location 1---------------------middle_right")




        robot1_group.set_named_target("middle_left")
        _, plan, _, _ = robot1_group.plan()
        robot1_goal.trajectory = plan
        robot1_client.send_goal(robot1_goal)
        robot1_client.wait_for_result()
        rospy.loginfo("location 2---------------------middle_left")





def move_to_point():
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

    
    robot1_group.set_max_acceleration_scaling_factor(0.02)
    robot1_group.set_max_velocity_scaling_factor(1)
    rospy.loginfo('Execute Trajectory server is available for robot1')

    robot1_group.set_planning_time(0.1)
    # robot1_group.set_num_planning_attempts(1)
    # planner_id = "RRTConnect"
    # robot1_group.set_planner_id(planner_id)   
    # robot1_group.allow_replanning(True)  # allow for replanning if obstacles are encountered
    # robot1_group.allow_looking(True)

    while not rospy.is_shutdown():
        success = False

        robot1_group.set_position_target([0.0065306, -0.46417, 1.2048])
        while success==False:
            success=robot1_group.go(wait=True)
        rospy.loginfo("location 1---------------------middle_right")
        
        success = False
    
        robot1_group.set_position_target([0.1065306, 0, 1.2048])
        while success==False:

            success=robot1_group.go(wait=True)
        rospy.loginfo("location 2---------------------middle_front")

        success = False
        
        robot1_group.set_position_target([0.0065306, 0.46417, 1.2048])
        while success==False:

            success=robot1_group.go(wait=True)
        rospy.loginfo("location 3---------------------middle_left")

        success = False
        
        robot1_group.set_position_target([0.1065306, 0, 1.2048])
        while success==False:

            success=robot1_group.go(wait=True)
        rospy.loginfo("location 4---------------------middle_front")

def move_toss():
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

    robot1_group.set_planning_time(0.1)
    robot1_group.set_num_planning_attempts(1)
    # robot1_group.allow_replanning(True)  # allow for replanning if obstacles are encountered
    robot1_group.allow_looking(True)
    i=-0.5
    c = "left"
    while not rospy.is_shutdown():
        print(i)

        if c == "left":
            i+=0.01
            if i > 0.5:
                c = "right"
        elif c == "right":
            i-=0.01
            if i < -0.5:
                c="left"

        robot1_group.set_pose_target([0.0065306, i, 1.2048,0,0,0,1])
        success=robot1_group.go(wait=True)
    

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

    
    robot1_group.set_max_acceleration_scaling_factor(0.1)
    robot1_group.set_max_velocity_scaling_factor(0.1)
    rospy.loginfo('Execute Trajectory server is available for robot1')

    robot1_group.set_planning_time(0.2)
    # robot1_group.set_num_planning_attempts(1)
    robot1_group.allow_replanning(True)  # allow for replanning if obstacles are encountered
    robot1_group.allow_looking(True)


    while not rospy.is_shutdown():
        success = False
        tick = 0

        robot1_group.set_named_target("middle_right")
        while not success:
            success=robot1_group.go(wait=True)
            current_state = np.asarray(robot1_group.get_current_joint_values())

            if not success:
                print(np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values())))
                if np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values()))<0.01:
                    tick += 1
                    print(tick)
                    if tick == 100:
                        success = True
                        print("EXITTTTTTTTTTTTTTTTTTTTT")
                else:
                    tick = 0
        rospy.loginfo("location 1---------------------middle_right")
        
        success = False
        tick = 0
        
        robot1_group.set_named_target("front")
        while not success:
            success=robot1_group.go(wait=True)
            current_state = np.asarray(robot1_group.get_current_joint_values())

            if not success:
                print(np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values())))

                if np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values()))<0.01:
                    tick += 1
                    print(tick)
                    if tick == 100:
                        success = True
                        print("EXITTTTTTTTTTTTTTTTTTTTT")
                else:
                    tick = 0
        rospy.loginfo("location 2---------------------front")


        success = False
        tick = 0
        
        robot1_group.set_named_target("middle_left")
        while not success:
            success=robot1_group.go(wait=True)
            current_state = np.asarray(robot1_group.get_current_joint_values())

            if not success:
                print(np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values())))

                if np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values()))<0.01:
                    tick += 1
                    if tick == 100:
                        success = True
                else:
                    tick = 0
        rospy.loginfo("location 3---------------------middle_left")

        success = False
        tick = 0
        
        robot1_group.set_named_target("front")
        while not success:
            success=robot1_group.go(wait=True)
            current_state = np.asarray(robot1_group.get_current_joint_values())

            if not success:
                print(np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values())))

                if np.linalg.norm(current_state - np.asarray(robot1_group.get_current_joint_values()))<0.01:
                    tick += 1
                    print(tick)
                    if tick == 100:
                        success = True
                        print("EXITTTTTTTTTTTTTTTTTTTTT")
                else:
                    tick = 0
        rospy.loginfo("location 4---------------------front")

if __name__ == '__main__':
    try:
         #move_to_point()
         
         #move_go_3()
         #move_go_3()
         move_go_2()
        # all_poses()
    except rospy.ROSInterruptException:
        pass
