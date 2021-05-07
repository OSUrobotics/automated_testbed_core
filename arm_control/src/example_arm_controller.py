#! /usr/bin/env python
import sys
import rospy 
import actionlib
import time
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult

class ExampleArmController():
	
	def __init__(self):   
		#initializing actionservers
		self.start_arm = actionlib.SimpleActionServer("start_arm_sequence", StageAction, self.start_arm_sequence_callback, False) 
		self.start_arm.start()
		
 
	def start_arm_sequence_callback(self, goal):

	
		self.start_arm.publish_feedback(StageFeedback(status="EXAMPLE: GRABBING OBJECT"))
		###do any arm calls or work here


		time.sleep(3)
		self.start_arm.set_succeeded(StageResult(result = 0), text="SUCCESS")



if __name__ == '__main__':
	rospy.init_node("example_arm_controller", argv=sys.argv)
	begin = ExampleArmController()
	rospy.spin()
