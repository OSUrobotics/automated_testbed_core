#!/usr/bin/env python

import rospy
import sys
import actionlib
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult

class DataCollection():
	
	def __init__(self):   
		#initializing actionservers
		self.start_collection = actionlib.SimpleActionServer("start_data_collection", StageAction, self.start_collection_callback, False) 
		self.stop_collection = actionlib.SimpleActionServer("stop_data_collection", StageAction, self.stop_collection_callback, False) 
		self.start_collection.start()
		self.stop_collection.start()

		self.collection_flag = False

 
	def start_collection_callback(self, goal):

		if(self.collection_flag == True):
			self.start_collection.set_aborted(StageResult(result = 100), text="ABORT, PREVIOUS DATA COLLECTION WAS NOT STOPPED")
			return

	
		self.start_collection.publish_feedback(StageFeedback(status="STARTING ROSBAGS"))
		self.start_collection.publish_feedback(StageFeedback(status="STARTING RECORDING"))
		###START ROSBAGS AND CAMERA HERE



		self.collection_flag = True
		print("MADE IT")
		self.start_collection.set_succeeded(StageResult(result = 0), text="SUCCESS")





	def stop_collection_callback(self, goal):

		if(self.collection_flag == False):
			self.stop_collection.set_aborted(StageResult(result = 100), text="ABORT, DATA COLLECTION WAS NOT STARTED")
			return
		

	
		self.stop_collection.publish_feedback(StageFeedback(status="STOPPING ROSBAGS"))
		self.stop_collection.publish_feedback(StageFeedback(status="STOPPING RECORDING"))
		###STOP ROSBAGS AND CAMERA HERE



		
		self.collection_flag = False
		print("MADE IT 2")
		self.stop_collection.set_succeeded(StageResult(result = 0), text="SUCCESS")
  



if __name__ == '__main__':
	rospy.init_node("data_collection", argv=sys.argv)
	begin = DataCollection()
	rospy.spin()
