#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from infrastructure_flexbe_states.parameter_ac import ParameterActionClient
from infrastructure_flexbe_states.stage_ac import StageActionClient
from infrastructure_flexbe_states.test_control_state import TestControlState
from infrastructure_flexbe_states.trial_control_state import TrialControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 3/22/21
@author: Keegan Nave
'''
class System_Behaviour_PiSM(Behavior):
	'''
	Newest replacement behaviour for use with the raspberry Pi's. Certain states are not yet implemented fully.
	'''


	def __init__(self):
		super(System_Behaviour_PiSM, self).__init__()
		self.name = 'System_Behaviour_Pi'

		# parameters of this behavior
		self.add_parameter('number_of_trials', 1)
		self.add_parameter('number_of_tests', 1)
		self.add_parameter('start_data_collection_topic', 'start_data_collection')
		self.add_parameter('parameter_topic', 'set_test_parameters')
		self.add_parameter('reset_topic', 'reset_hardware')
		self.add_parameter('stop_data_collection_topic', 'stop_data_collection')
		self.add_parameter('arm_control_topic', 'start_arm_sequence')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:840 y:92, x:36 y:676
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

			# x:800 y:212
			OperatableStateMachine.add('Set Test Parameters',
										ParameterActionClient(topic=self.parameter_topic),
										transitions={'completed': 'User Arm Control', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:562 y:122
			OperatableStateMachine.add('Start Data Collection',
										StageActionClient(topic=self.start_data_collection_topic),
										transitions={'completed': 'Set Test Parameters', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:380 y:374
			OperatableStateMachine.add('Stop Data Collection',
										StageActionClient(topic=self.stop_data_collection_topic),
										transitions={'completed': 'Trial Control', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:298 y:127
			OperatableStateMachine.add('Trial Control',
										TrialControlState(),
										transitions={'continue': 'Start Data Collection', 'failed': 'failed', 'completed': 'Test Control'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'completed': Autonomy.Off},
										remapping={'number_of_trials': 'number_of_trials'})

			# x:851 y:308
			OperatableStateMachine.add('User Arm Control',
										StageActionClient(topic=self.arm_control_topic),
										transitions={'completed': 'Reset', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:670 y:421
			OperatableStateMachine.add('Reset',
										StageActionClient(topic=self.reset_topic),
										transitions={'completed': 'Stop Data Collection', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
