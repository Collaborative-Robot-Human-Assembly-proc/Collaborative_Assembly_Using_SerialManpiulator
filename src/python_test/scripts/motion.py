#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest, SetIOSignalResponse
import numpy as np


def simple_pick_place():
    # First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place',
                    anonymous=True)


    robot1_group = moveit_commander.MoveGroupCommander("manipulator",robot_description = "robot_description")
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    # Action clients to the ExecuteTrajectory action server.
    robot1_client = actionlib.SimpleActionClient(
        'execute_trajectory',
        moveit_msgs.msg.ExecuteTrajectoryAction)
    robot1_client.wait_for_server()
    

    rate = rospy.Rate(10)


    rospy.loginfo('Execute Trajectory server is available for robot1')
    
    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_group.set_named_target("middle_right")
    robot1_group.set_max_acceleration_scaling_factor(0.1)
    robot1_group.set_max_velocity_scaling_factor(0.1)

    # while not rospy.is_shutdown():
        
    #     reached = ""

        # while reached != "done":
        #     robot1_group.set_named_target("middle_right")
        #     named_target_angles = robot1_group.get_current_joint_values()
        #     print(named_target_angles)
        #     print("+=======================++++++++++++++++++++")
        #     counter = 0
        #     while not rospy.is_shutdown():
        #         _, plan, _, _ = robot1_group.plan()
        #         print("PLAN")
        #         if counter == 0:
        #             final_location = np.asarray(plan.joint_trajectory.points[-1].positions)
        #             print(final_location)
        #             print("ASSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSA")
        #             counter += 1

        #     # success = robot1_group.go(wait=True)

        #         robot1_goal.trajectory.joint_trajectory.header= plan.joint_trajectory.header
        #         robot1_goal.trajectory.joint_trajectory.joint_names= plan.joint_trajectory.joint_names
        #         robot1_goal.trajectory.joint_trajectory.points=plan.joint_trajectory.points[0:20]



        #         robot1_group.set_orientation_target(plan.joint_trajectory.points[0:20],"link_6")
        #         success = robot1_group.go(wait=True)
        #         current_state = np.asarray(robot.get_current_state().joint_state.position)
        #         print(current_state)
        #         sq_diff = np.linalg.norm(final_location - current_state)
        #         if sq_diff<0.01:
        #             reached = "done"
        #             rospy.loginfo("location 1---------------------")
        #             break


            
        # reached = ""

    while not rospy.is_shutdown():
        
        reached = ""

        while reached != "done":
            robot1_group.set_named_target("middle_right")
            named_target_angles = robot1_group.get_current_joint_values()
            print(named_target_angles)
            print("+=======================++++++++++++++++++++")
            counter = 0
            while not rospy.is_shutdown():
                _, plan, _, _ = robot1_group.plan()
                print("PLAN")
                if counter == 0:
                    final_location = np.asarray(plan.joint_trajectory.points[-1].positions)
                    print(final_location)
                    print("ASSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSA")
                    counter += 1

            # success = robot1_group.go(wait=True)

                robot1_goal.trajectory.joint_trajectory.header= plan.joint_trajectory.header
                robot1_goal.trajectory.joint_trajectory.joint_names= plan.joint_trajectory.joint_names
                robot1_goal.trajectory.joint_trajectory.points=plan.joint_trajectory.points[0:20]

                manga=moveit_msgs.msg.ExecuteTrajectoryGoal()
                manga.trajectory = plan

                robot1_client.send_goal(robot1_goal)
                success = robot1_client.wait_for_result()
                current_state = np.asarray(robot.get_current_state().joint_state.position)
                print(current_state)
                sq_diff = np.linalg.norm(final_location - current_state)
                # if np.allclose(current_state, named_target_angles, atol=0.1):
                if sq_diff<0.01:
                    reached = "done"
                    rospy.loginfo("location 1---------------------")
                    break
            # a, plan, _, _ = robot1_group.plan()
            # if a == True:
            #     robot1_goal.trajectory = plan
            #     robot1_client.send_goal(robot1_goal)
            #     # robot1_client.wait_for_result()

            
        reached = ""

        while reached != "done":
            robot1_group.set_named_target("middle_left")
            success = robot1_group.go(wait=True)
            if success:
                reached = "done"
                rospy.loginfo("location 2---------------------")
        #     robot1_group.set_named_target("middle_left")
        #     a, plan, _, _ = robot1_group.plan()
        #     if a == True:
        #         robot1_goal.trajectory = plan
        #         robot1_client.send_goal(robot1_goal)
        #         # robot1_client.wait_for_result()


    # moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        simple_pick_place()
    except rospy.ROSInterruptException:
        pass