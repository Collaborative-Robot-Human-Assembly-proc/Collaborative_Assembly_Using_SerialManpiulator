#!/usr/bin/env python3
import sys
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import rospy
from std_msgs.msg import String


	
def publish_fn():
	rospy.init_node("speech_recognition_node_pb",anonymous=True)
	pub=rospy.Publisher("speech_recognition",String,queue_size=10)
	rate=rospy.Rate(1)
	while not rospy.is_shutdown():
		######################## ENTER WORD HERE ##########################

		rospy.loginfo("Speech Recognition Node")
		terms = ["robot start","robot stop","pose estimation","process one"]#term we want to search for
		nlp_sentence = input("Word: ") #read input from user
		words = nlp_sentence.split() #split the sentence into individual words
		for keyword in terms:
			if keyword in words: #see if one of the words in the sentence is the word we want
				message=String(keyword)
				pub.publish(message)
				rospy.loginfo(message.data+" is sent.")
				rate.sleep()
	else:
		print("\nInterruption is done, Exiting.....")



if __name__=="__main__":
	try:
		publish_fn()
	except rospy.ROSInterruptException:
		pass
