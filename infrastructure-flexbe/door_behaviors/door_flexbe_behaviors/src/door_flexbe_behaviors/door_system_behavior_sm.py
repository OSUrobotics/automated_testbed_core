#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from infrastructure_flexbe_states.test_control_state import TestControlState
from infrastructure_flexbe_states.trial_control_state import TrialControlState
from infrastructure_flexbe_states.generic_pubsub import GenericPubSub
from infrastructure_flexbe_states.generic_pub import GenericPub
from infrastructure_flexbe_states.data_collection_ac import DataCollectionActionClient
from infrastructure_flexbe_states.stage_ac import StageActionClient
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Sep 18 2020
@author: Keegan Nave
'''
class Door_System_BehaviorSM(Behavior):
	'''
	Use to run the arm and door at the same time, waits for reply from single_stage_as in the hardware_flexbe_link package
	'''


	def __init__(self):
		super(Door_System_BehaviorSM, self).__init__()
		self.name = 'Door_System_Behavior'

		# parameters of this behavior
		self.add_parameter('number_of_trials', 1)
		self.add_parameter('number_of_tests', 1)
		self.add_parameter('pub_reset_topic', 'reset_start')
		self.add_parameter('sub_reset_topic', 'reset_complete')
		self.add_parameter('pub_reset_int', 1)
		self.add_parameter('start_data_topic', 'start_collection')
		self.add_parameter('data_int', 1)
		self.add_parameter('action_server_topic', 'single_stage_as')
		self.add_parameter('data_collection_server_topic', 'data_collection_as')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:840 y:92, x:57 y:454
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:107 y:34
			OperatableStateMachine.add('Test Control',
										TestControlState(num_trials=self.number_of_trials, num_tests=self.number_of_tests),
										transitions={'continue': 'Trial Control', 'failed': 'failed', 'completed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'completed': Autonomy.Off},
										remapping={'number_of_trials': 'number_of_trials'})

			# x:298 y:127
			OperatableStateMachine.add('Trial Control',
										TrialControlState(),
										transitions={'continue': 'Data Collection AC', 'failed': 'failed', 'completed': 'Test Control'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'completed': Autonomy.Off},
										remapping={'number_of_trials': 'number_of_trials'})

			# x:335 y:559
			OperatableStateMachine.add('Test Station PubSub',
										GenericPubSub(pubtopic=self.pub_reset_topic, subtopic=self.sub_reset_topic, pubint=self.pub_reset_int),
										transitions={'completed': 'Trial Control', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:817 y:202
			OperatableStateMachine.add('Data Capture Start Pub',
										GenericPub(pubtopic=self.start_data_topic, pubint=self.data_int),
										transitions={'completed': 'Stage AC', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:565 y:128
			OperatableStateMachine.add('Data Collection AC',
										DataCollectionActionClient(topic=self.data_collection_server_topic),
										transitions={'completed': 'Data Capture Start Pub', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:669 y:407
			OperatableStateMachine.add('Stage AC',
										StageActionClient(topic=self.action_server_topic),
										transitions={'completed': 'Test Station PubSub', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
