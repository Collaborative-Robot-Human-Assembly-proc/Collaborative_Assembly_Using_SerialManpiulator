#!/usr/bin/env python3

from python_test.srv import PoseEstimation, PoseEstimationRequest, PoseEstimationResponse
import rospy
import subprocess

def process_service_request(request):
	response=PoseEstimationResponse()
	if request.initiation==1:
		print("Beginning Pose Estimation")
		subprocess.run("/home/amirwael/trial_2_ws/src/python_test/scripts/pose_estimation.bash")
		response.succession="Robot pose estimation processing."
		rospy.loginfo(response.succession)
	return response



def poseEstimationMode():
	rospy.init_node('pose_estimation_node', anonymous=False)
	service = rospy.Service('pose_estimation_initiation', PoseEstimation, process_service_request)
	rospy.loginfo('Pose Estimation robot server is now available')
	rospy.spin()


if __name__ == "__main__":
	poseEstimationMode()
