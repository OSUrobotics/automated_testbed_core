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
from infrastructure_flexbe_states.stage_ac import StageActionClient
from infrastructure_flexbe_states.generic_pubsub import GenericPubSub
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Sep 18 2020
@author: Keegan Nave
'''
class Single_Stage_System_BehaviorSM(Behavior):
	'''
	This is a template for a single stage, single action server arm movement connected to a generic pubsub that could communicate with a testing station.
	'''


	def __init__(self):
		super(Single_Stage_System_BehaviorSM, self).__init__()
		self.name = 'Single_Stage_System_Behavior'

		# parameters of this behavior
		self.add_parameter('number_of_trials', 1)
		self.add_parameter('number_of_tests', 1)
		self.add_parameter('action_topic', 'single_stage_as')
		self.add_parameter('pub_topic', 'reset_start')
		self.add_parameter('sub_topic', 'reset_complete')
		self.add_parameter('pub_int', 1)

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
										transitions={'continue': 'Single Stage Action Client', 'failed': 'failed', 'completed': 'Test Control'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'completed': Autonomy.Off},
										remapping={'number_of_trials': 'number_of_trials'})

			# x:517 y:302
			OperatableStateMachine.add('Single Stage Action Client',
										StageActionClient(topic=self.action_topic),
										transitions={'completed': 'Test Station PubSub', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:343 y:437
			OperatableStateMachine.add('Test Station PubSub',
										GenericPubSub(pubtopic=self.pub_topic, subtopic=self.sub_topic, pubint=self.pub_int),
										transitions={'completed': 'Trial Control', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
