#!/usr/bin/env python3

from python_test.srv import StartRobot, StartRobotRequest, StartRobotResponse
import rospy
import subprocess

def process_service_request(request):
	response=StartRobotResponse()
	if request.initiation==1:
		subprocess.run("/home/amirwael/trial_2_ws/src/python_test/scripts/start_robot.bash")
		response.succession="Robot Started Successfully."
		rospy.loginfo(response.succession)
	return response



def start_server():
    service = rospy.Service('initiate_robot_motion', StartRobot, process_service_request)
    rospy.init_node('start_robot_node', anonymous=False)
    rospy.loginfo('Starting robot server is now available')
    rospy.spin()



if __name__ == "__main__":
    start_server()
