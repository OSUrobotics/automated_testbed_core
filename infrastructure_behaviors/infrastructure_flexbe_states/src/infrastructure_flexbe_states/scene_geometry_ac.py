#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from arm_control_pkg.msg import SceneConstraintAction, SceneConstraintGoal


class SceneGeometryActionClient(EventState):
    '''
    Actionlib_comm_state is an actionlib state that will make a call to an action server.
    This is intended to serve as a template for future states that need to communicate with 
    action server. Currently this is set up to communicate with the data collection action 
    server and doesnt do anything other than return true or false.
    TODO: Proper error checking
     
    <= continue             All actions completed, data collection started
    <= failed               Data control failed to initialize or call something TODO: Proper error checking          

    '''e


    def __init__(self, instruction):
        # See example_state.py for basic explanations.
        super(SceneGeometryActionClient, self).__init__(outcomes = ['completed', 'failed'])

        self._topic = 'scene_constraint_as'
        self._client = ProxyActionClient({self._topic: SceneConstraintAction})
        self._instruction = instruction
		# It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'failed'

        #otherwise get the result and perform next steps
        if(self._client.has_result(self._topic)):
            result = self._client.get_result(self._topic)
            status = result.constraint_result
            if(status == True):
                print("returned True")
                return 'completed'
            else:
                print("returned False")
                return 'failed'


    def on_enter(self, userdata):
        #Creating the goal to send
        goal = SceneConstraintGoal()
        goal.constraint_request = self._instruction

        #error checking in case communication cant be established
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the start data collection command:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
		# Makes sure that the action is not running when leaving this state.
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')

