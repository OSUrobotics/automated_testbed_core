#! /usr/bin/env python
import rospy 
import actionlib
import infrastructure_msgs.msg

class SingleStageActionServer(object):
    # create messages that are used to publish feedback/result
	_feedback = infrastructure_msgs.msg.StageFeedback()
	_result = infrastructure_msgs.msg.StageResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, infrastructure_msgs.msg.StageAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()
      
	def execute_cb(self, goal):
		if self._as.is_preempt_requested():
			rospy.loginfo('%s: Preempted' % self._action_name)
			self._as.set_preempted()

		if goal.stage_start == True:
			self._result.stage_result = True



			#YOUR CODE HERE








			rospy.loginfo('Succeeded')
			self._as.set_succeeded(self._result)

		else:
			self._result.stage_result = False
			rospy.loginfo('Failed') 
			self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
	rospy.init_node('single_stage_as_py')
	server = SingleStageActionServer(rospy.get_name())
	rospy.spin()
