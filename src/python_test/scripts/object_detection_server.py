#!/usr/bin/env python3

from python_test.srv import ObjectDetection, ObjectDetectionRequest, ObjectDetectionResponse
import rospy
import subprocess

def process_service_request(request):
	response=ObjectDetectionResponse()
	if request.initiation==1:
		subprocess.run("/home/amirwael/trial_2_ws/src/python_test/scripts/object_detection.bash")
		response.succession="Object Detected."
		rospy.loginfo(response.succession)
	return response



def start_server():
    service = rospy.Service('object_detection_node', ObjectDetection, process_service_request)
    rospy.init_node('start_robot_node', anonymous=False)
    rospy.loginfo('Starting robot server is now available')
    rospy.spin()



if __name__ == "__main__":
    start_server()
