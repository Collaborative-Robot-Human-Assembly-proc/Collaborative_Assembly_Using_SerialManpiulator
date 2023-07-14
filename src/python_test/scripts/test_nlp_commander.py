#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from python_test.srv import StartRobot, StartRobotRequest, StartRobotResponse
from python_test.srv import StopRobot, StopRobotRequest, StopRobotResponse
from python_test.srv import Assemble, AssembleRequest, AssembleResponse

class serviceClientClass(object):
	def __init__(self,service):
		rospy.loginfo("Service Client initiated.")
		
		if service.data=="start":
			self.startRobot()
		elif service.data=="stop":
			self.stopRobot()
		elif service.data=="assemble":
			self.assembleMode()


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

	def assembleMode(self):

		rospy.loginfo("Waiting for service: assemble mode...")
		rospy.wait_for_service('assemble_initiation')
		proxy=rospy.ServiceProxy('assemble_initiation', Assemble)
		proxy_response=proxy(1)
		rospy.loginfo(proxy_response.succession)


def subCallback(message):
	rospy.loginfo(message.data+" is called...........")
	serviceClientClass(message)


def sub_fn():
	rospy.init_node("main_client",anonymous=True)
	sub=rospy.Subscriber("nlp_processing",String,subCallback)
	rospy.spin()







if __name__ == "__main__":
	
	try:
		sub_fn()
	except	rospy.ROSInterruptException:
		pass