#!/usr/bin/env python

import rospy
import sys
import actionlib
from infrastructure_msgs.msg import TestParametersAction, TestParametersGoal, TestParametersFeedback, TestParametersResult

class TestParameters():
    
    def __init__(self):   
        #initializing actionservers
		self.test_parameters = actionlib.SimpleActionServer("set_test_parameters", TestParametersAction, self.set_parameters_callback, False) 
		self.test_parameters.start()
 
    def set_parameters_callback(self, goal):

		#ANY CHECKS FOR PARAMETERS AND EXECUTION OF PARAMETERS ON HARDWARE HERE
		self.test_parameters.publish_feedback(TestParametersFeedback(status="EXAMPLE: CHANGED OBJECT"))

		self.test_parameters.set_succeeded(TestParametersResult(result=0), text="SUCCESS")
		#Or if failed:
		#self.test_parameters.set_aborted(TestParameters(result = 100), text="FAILED")

if __name__ == '__main__':
    rospy.init_node("test_parameters", argv=sys.argv)
    begin = TestParameters()
    rospy.spin()
