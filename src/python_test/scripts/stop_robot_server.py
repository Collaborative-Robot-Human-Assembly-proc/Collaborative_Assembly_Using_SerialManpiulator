#!/usr/bin/env python3

from python_test.srv import StopRobot, StopRobotRequest, StopRobotResponse
import rospy
import subprocess

def process_service_request(request):
	response=StopRobotResponse()
	if request.initiation==1:
		subprocess.run("/home/amirwael/trial_2_ws/src/python_test/scripts/stop_robot.bash")
		response.succession="Robot Stopped Successfully."
		rospy.loginfo(response.succession)
	return response



def stop_server():
    service = rospy.Service('abort_robot_motion', StopRobot, process_service_request)
    rospy.init_node('abort_robot_node', anonymous=False)
    rospy.loginfo('Stopping robot server is now available')
    rospy.spin()



if __name__ == "__main__":
    stop_server()
