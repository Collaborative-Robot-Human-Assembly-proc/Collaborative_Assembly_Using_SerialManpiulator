#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import subprocess
class speechRecognitonClass(object):
	def __init__(self):
		rospy.loginfo("speech_recognition_commander_node initiated.")
		rospy.init_node("speech_recognition_commander_node")
		self.sub=rospy.Subscriber("nlp_processing",String,self.subscriber_callback)
		self.commands_list = []
		rospy.spin()

	def __del__(self):
		print("speech_recognition_commander_node closed.")


	def subscriber_callback(self,message):
		if message.data=="robot start":
			one_time_process_name = ['rosrun','python_test','start_robot.bash']
			subprocess.Popen(one_time_process_name)

		elif message.data=="robot stop":
			one_time_process_name = ['rosrun','python_test','start_robot.bash']
			subprocess.Popen(one_time_process_name)
		
		else:
			for index, sublist in enumerate(self.commands_list):
				for element in sublist:
					if element == message.data:
						print("Command Already Running")
						break

					else:
						if message.data=="pose estimation":
							process_name = ['rosrun','python_test','motion_6.py']
							self.commands_list.append([])
							subprocess.Popen(process_name)

						
						elif message.data=="process one":
							process_name = ['rosrun','python_test','collision.py']
							self.commands_list.append([])
							subprocess.Popen(process_name)



						else:
							print("Wrong Input Word....")

if __name__ == "__main__":
	try:
		speechRecognitonClass()
	except rospy.ROSInterruptException:
		pass