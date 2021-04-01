#!/usr/bin/env python

import rospy
import sys
import actionlib
import time
import roslaunch
import numpy as np
import cv2 as cv
import os
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult




class DataCollection():
	
	def __init__(self):   
		#initializing actionservers
		self.start_collection = actionlib.SimpleActionServer("start_data_collection", StageAction, self.start_collection_callback, False) 
		self.stop_collection = actionlib.SimpleActionServer("stop_data_collection", StageAction, self.stop_collection_callback, False) 
		self.start_collection.start()
		self.stop_collection.start()

		self.collection_flag = False
		self.trial_count = 0
		self.video_path = os.path.dirname(os.path.realpath(__file__))
		self.video_path = os.path.split(self.video_path)[0] + "/stored_data/recorded_video/"
		name_parameter = rospy.get_param("test_name", "infrastructure_trial_")
		record_parameter = rospy.get_param("record_video", False) 
		print(name_parameter)
		print(record_parameter)

		if(record_parameter == True):
			while(not rospy.is_shutdown()):
				if(self.collection_flag == True):
					cap = cv.VideoCapture(0)
					fourcc = cv.VideoWriter_fourcc(*'XVID')
					name = str(self.video_path) + name_parameter + str(self.trial_count) + ".avi"
					out = cv.VideoWriter(name, fourcc, 30, (640, 480))

					self.start_collection.publish_feedback(StageFeedback(status="STARTING RECORDING"))
					while(self.collection_flag == True and cap.isOpened() and not rospy.is_shutdown()):
						ret, frame = cap.read()
						if(not ret):
							print("ERROR, CANT GET FRAME")
							break
						out.write(frame)
					self.stop_collection.publish_feedback(StageFeedback(status="STOPPING RECORDING"))
					cap.release()
					out.release()

				else:
					time.sleep(1)


		 
	def start_collection_callback(self, goal):

		if(self.collection_flag == True):
			self.start_collection.set_aborted(StageResult(result = 100), text="ABORT, PREVIOUS DATA COLLECTION WAS NOT STOPPED")
			return

		self.collection_flag = True
		self.trial_count += 1
		print("MADE IT")
		time.sleep(5)
		self.start_collection.set_succeeded(StageResult(result = 0), text="SUCCESS")
			


	def stop_collection_callback(self, goal):

		if(self.collection_flag == False):
			self.stop_collection.set_aborted(StageResult(result = 100), text="ABORT, DATA COLLECTION WAS NOT STARTED")
			return
		
		self.collection_flag = False
		print("MADE IT 2")
		time.sleep(5)
		self.stop_collection.set_succeeded(StageResult(result = 0), text="SUCCESS")
  



if __name__ == '__main__':
	rospy.init_node("data_collection", argv=sys.argv)
	begin = DataCollection()
	rospy.spin()
