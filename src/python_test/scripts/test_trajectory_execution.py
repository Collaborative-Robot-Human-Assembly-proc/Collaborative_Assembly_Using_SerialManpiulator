#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest, SetIOSignalResponse


def simple_pick_place():
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

    delay = rospy.Rate(0.2)
    delay2 = rospy.Rate(0.5)


    # Action clients to the ExecuteTrajectory action server.
    robot1_client = actionlib.SimpleActionClient(
        'execute_trajectory',
        moveit_msgs.msg.ExecuteTrajectoryAction)
    robot1_client.wait_for_server()
    
    #rospy.wait_for_service('/rws/set_io_signal')
    #proxy = rospy.ServiceProxy('/rws/set_io_signal', SetIOSignal)

    rospy.loginfo('Execute Trajectory server is available for robot1')

    robot1_group.set_named_target("middle_right")
    _, plan, _, _ = robot1_group.plan()
    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_goal.trajectory = plan
    robot1_client.send_goal(robot1_goal)
    robot1_client.wait_for_result()
    delay.sleep()
    rospy.loginfo("location 1---------------------")
    delay.sleep()

    #proxy(signal='valve',value='0')
    



    robot1_group.set_named_target("all_zero")
    robot1_group.set_max_acceleration_scaling_factor(1)
    _, plan, _, _ = robot1_group.plan()
    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_goal.trajectory = plan
    robot1_client.send_goal(robot1_goal)
    robot1_client.wait_for_result()
    delay.sleep()
    rospy.loginfo("location 222---------------------")


    robot1_group.set_named_target("middle_left")
    robot1_group.set_max_acceleration_scaling_factor(1)

    _, plan, _, _ = robot1_group.plan()
    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_goal.trajectory = plan
    robot1_client.send_goal(robot1_goal)
    #rospy.loginfo(robot1_client.wait_for_result())
    robot1_client.wait_for_result()
    delay.sleep()
    rospy.loginfo("location 333---------------------")
    #proxy(signal='valve',value='1')


    
    



    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        simple_pick_place()
    except rospy.ROSInterruptException:
        pass