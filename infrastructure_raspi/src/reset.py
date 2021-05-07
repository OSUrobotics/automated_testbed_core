#!/usr/bin/env python

import rospy
import sys
import actionlib
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult

class Reset():
    
    def __init__(self):   
		#initializing actionservers
		self.start_reset = actionlib.SimpleActionServer("reset_hardware", StageAction, self.reset_callback, False) 
		self.start_reset.start()

    def reset_callback(self, goal):

		###ALL HARDWARE RELATED CODE GOES HERE
		self.start_reset.publish_feedback(StageFeedback(status="EXAMPLE: WINDING OBJECT IN"))
		self.start_reset.set_succeeded(StageResult(result = 0), text="SUCCESS")


if __name__ == '__main__':
    rospy.init_node("reset", argv=sys.argv)
    begin = Reset()
    rospy.spin()
