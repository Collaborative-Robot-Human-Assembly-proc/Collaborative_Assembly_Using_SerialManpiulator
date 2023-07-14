#!/usr/bin/env python3

from python_test.srv import Assemble, AssembleRequest, AssembleResponse
import rospy
import subprocess

def process_service_request(request):
	response=AssembleResponse()
	if request.initiation==1:
		subprocess.run("/home/amirwael/trial_2_ws/src/python_test/scripts/assemble.bash")

		response.succession="Robot Assemble Successfully."
		rospy.loginfo(response.succession)
	return response



def assemble():
    service = rospy.Service('assemble_initiation', Assemble, process_service_request)
    rospy.init_node('assemble_node', anonymous=False)
    rospy.loginfo('Assemble robot server is now available')
    rospy.spin()



if __name__ == "__main__":
    assemble()
