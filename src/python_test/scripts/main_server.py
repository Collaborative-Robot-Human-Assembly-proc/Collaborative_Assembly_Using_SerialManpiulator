#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from python_test.srv import StartRobot, StartRobotRequest, StartRobotResponse
from python_test.srv import StopRobot, StopRobotRequest, StopRobotResponse
from python_test.srv import PoseEstimation, PoseEstimationRequest, PoseEstimationResponse
from python_test.srv import Assemble, AssembleRequest, AssembleResponse
from python_test.srv import ObjectDetection, ObjectDetectionRequest, ObjectDetectionResponse

class serviceClientClass(object):
	def __init__(self,service):
		rospy.loginfo("Main Server initiated.")
		
		if service.data=="robot start":
			self.startRobot()
		
		elif service.data=="robot stop":
			self.stopRobot()
		
		elif service.data=="pose estimation":
			self.poseEstimationMode()
		
		elif service.data=="process one":
			self.assembleMode()

		elif service.data=="object detection":
			self.objectDetectionMode()


	def startRobot(self):

		rospy.loginfo("Waiting for service: Start Robot...")
		rospy.wait_for_service('initiate_robot_motion')
		proxy=rospy.ServiceProxy('initiate_robot_motion', StartRobot)
		proxy_response=proxy(1)
		rospy.loginfo(proxy_response.succession)
	
	def stopRobot(self):

		rospy.loginfo("Waiting for service: Stop Robot...")
		rospy.wait_for_service('abort_robot_motion')
		proxy=rospy.ServiceProxy('abort_robot_motion', StopRobot)
		proxy_response=proxy(1)
		rospy.loginfo(proxy_response.succession)

	def poseEstimationMode(self):

		rospy.loginfo("Waiting for service: Pose Estimation mode...")
		rospy.wait_for_service('pose_estimation_initiation')
		proxy=rospy.ServiceProxy('pose_estimation_initiation', PoseEstimation)
		proxy_response=proxy(1)
		rospy.loginfo(proxy_response.succession)	

	def assembleMode(self):

		rospy.loginfo("Waiting for service: Process one mode...")
		rospy.wait_for_service('assemble_initiation')
		proxy=rospy.ServiceProxy('assemble_initiation', Assemble)
		proxy_response=proxy(1)
		rospy.loginfo(proxy_response.succession)


	def objectDetectionMode(self):

		rospy.loginfo("Waiting for service: Object Detection Mode...")
		rospy.wait_for_service('object_detection_initiation')
		proxy=rospy.ServiceProxy('object_detection_initiation', ObjectDetection)
		proxy_response=proxy(1)
		rospy.loginfo(proxy_response.succession)


def subCallback(message):
	rospy.loginfo(message.data+" is called...........")
	serviceClientClass(message)


def sub_fn():
	rospy.init_node("main_client",anonymous=True)
	rospy.Subscriber('recognizer_1/output',String,subCallback)
	rospy.spin()







if __name__ == "__main__":
	
	try:
		sub_fn()
	except	rospy.ROSInterruptException:
		pass