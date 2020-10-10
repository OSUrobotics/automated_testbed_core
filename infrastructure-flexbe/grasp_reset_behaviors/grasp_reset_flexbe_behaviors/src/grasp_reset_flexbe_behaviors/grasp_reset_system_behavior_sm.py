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
from infrastructure_flexbe_states.stage_ac import StageActionClient
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 13 2020
@author: Keegan
'''
class Grasp_Reset_System_BehaviorSM(Behavior):
	'''
	(EXPERIMENTAL DONT USE) This behavior uses the action server method of control of the robot arm during the grasp reset trials. Breaks up arm movement and testbed reset into stages. I dont think its the best method
	'''


	def __init__(self):
		super(Grasp_Reset_System_BehaviorSM, self).__init__()
		self.name = 'Grasp_Reset_System_Behavior'

		# parameters of this behavior
		self.add_parameter('position_topic', '')
		self.add_parameter('approach_topic', '')
		self.add_parameter('grasp_topic', '')
		self.add_parameter('manipulate_topic', '')
		self.add_parameter('release_topic ', '')
		self.add_parameter('retreat_topic', '')

		# references to used behaviors
		self.add_behavior(Test_Control_Behavior_GRSM, 'Test_Control_Behavior_GR')
		self.add_behavior(Trial_Control_Behavior_GRSM, 'Trial_Control_Behavior_GR')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:839 y:80, x:9 y:580
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:382 y:17
			OperatableStateMachine.add('Test_Control_Behavior_GR',
										self.use_behavior(Test_Control_Behavior_GRSM, 'Test_Control_Behavior_GR'),
										transitions={'continue': 'Trial_Control_Behavior_GR', 'failed': 'failed', 'tests_completed': 'finished'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'tests_completed': Autonomy.Inherit},
										remapping={'number_of_trials': 'number_of_trials'})

			# x:382 y:144
			OperatableStateMachine.add('Trial_Control_Behavior_GR',
										self.use_behavior(Trial_Control_Behavior_GRSM, 'Trial_Control_Behavior_GR'),
										transitions={'continue': 'Position', 'failed': 'failed', 'trials_complete': 'Test_Control_Behavior_GR'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'trials_complete': Autonomy.Inherit},
										remapping={'number_of_trials': 'number_of_trials', 'rotation': 'rotation'})

			# x:164 y:448
			OperatableStateMachine.add('Position',
										StageActionClient(topic=self.position_topic),
										transitions={'completed': 'Approach', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:342 y:449
			OperatableStateMachine.add('Approach',
										StageActionClient(topic=self.approach_topic),
										transitions={'completed': 'Grasp', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:542 y:447
			OperatableStateMachine.add('Grasp',
										StageActionClient(topic=self.grasp_topic),
										transitions={'completed': 'Manipulate', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:730 y:452
			OperatableStateMachine.add('Manipulate',
										StageActionClient(topic=self.manipulate_topic),
										transitions={'completed': 'Release', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:911 y:451
			OperatableStateMachine.add('Release',
										StageActionClient(topic=self.release_topic),
										transitions={'completed': 'Retreat', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1109 y:456
			OperatableStateMachine.add('Retreat',
										StageActionClient(topic=self.retreat_topic),
										transitions={'completed': 'Trial_Control_Behavior_GR', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
