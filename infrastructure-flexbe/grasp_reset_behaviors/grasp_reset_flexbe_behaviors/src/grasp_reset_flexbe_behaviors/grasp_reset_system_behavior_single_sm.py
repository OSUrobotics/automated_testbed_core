#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from grasp_reset_flexbe_behaviors.test_control_behavior_gr_sm import Test_Control_Behavior_GRSM
from grasp_reset_flexbe_behaviors.trial_control_behavior_gr_sm import Trial_Control_Behavior_GRSM
from grasp_reset_flexbe_behaviors.reset_control_behavior_gr_sm import Reset_Control_Behavior_GRSM
from infrastructure_flexbe_states.stage_ac import StageActionClient
from infrastructure_flexbe_states.data_collection_ac import DataCollectionActionClient
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Sep 18 2020
@author: Keegan Nave
'''
class Grasp_Reset_System_Behavior_SingleSM(Behavior):
	'''
	Behavior that runs the testbed but instead of multi stage arm movement it is single stage with one action server.
	'''


	def __init__(self):
		super(Grasp_Reset_System_Behavior_SingleSM, self).__init__()
		self.name = 'Grasp_Reset_System_Behavior_Single'

		# parameters of this behavior
		self.add_parameter('action_topic', 'single_stage_as')
		self.add_parameter('data_collection_server_topic', 'data_collection_as')

		# references to used behaviors
		self.add_behavior(Test_Control_Behavior_GRSM, 'Test_Control_Behavior_GR')
		self.add_behavior(Trial_Control_Behavior_GRSM, 'Trial_Control_Behavior_GR')
		self.add_behavior(Reset_Control_Behavior_GRSM, 'Reset_Control_Behavior_GR')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:120 y:522, x:536 y:519
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:228 y:54
			OperatableStateMachine.add('Test_Control_Behavior_GR',
										self.use_behavior(Test_Control_Behavior_GRSM, 'Test_Control_Behavior_GR'),
										transitions={'continue': 'Trial_Control_Behavior_GR', 'failed': 'failed', 'tests_completed': 'finished'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'tests_completed': Autonomy.Inherit},
										remapping={'number_of_trials': 'number_of_trials'})

			# x:583 y:54
			OperatableStateMachine.add('Trial_Control_Behavior_GR',
										self.use_behavior(Trial_Control_Behavior_GRSM, 'Trial_Control_Behavior_GR'),
										transitions={'continue': 'Data Collection AC', 'failed': 'failed', 'trials_complete': 'Test_Control_Behavior_GR'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'trials_complete': Autonomy.Inherit},
										remapping={'number_of_trials': 'number_of_trials', 'rotation': 'rotation'})

			# x:718 y:336
			OperatableStateMachine.add('Reset_Control_Behavior_GR',
										self.use_behavior(Reset_Control_Behavior_GRSM, 'Reset_Control_Behavior_GR'),
										transitions={'continue': 'Trial_Control_Behavior_GR', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'rotation': 'rotation'})

			# x:737 y:220
			OperatableStateMachine.add('Single Stage Action Client',
										StageActionClient(topic=self.action_topic),
										transitions={'completed': 'Reset_Control_Behavior_GR', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:826 y:95
			OperatableStateMachine.add('Data Collection AC',
										DataCollectionActionClient(topic=self.data_collection_server_topic),
										transitions={'completed': 'Single Stage Action Client', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
