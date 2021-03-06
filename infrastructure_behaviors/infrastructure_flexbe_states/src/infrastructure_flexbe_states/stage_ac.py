#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from infrastructure_msgs.msg import StageAction, StageGoal


class StageActionClient(EventState):
    '''
    stage_ac_state is an action client state that will make a call to an action server.
    This is intended to serve as a template for future states that need to communicate with 
    action server. The topic given should correspond to an action server and it right now only 
    works for StageAction action messages but in the future it will have any custom messages needed
    once they are more finalized. Currently just sends true or false to an action server and
    either returns complete or failed based on the response.
    TODO: Proper error checking other custom messages
     
    <= continue             All actions completed, data collection started
    <= failed               Data control failed to initialize or call something TODO: Proper error checking          

    '''


    def __init__(self, topic):
        # See example_state.py for basic explanations.
        super(StageActionClient, self).__init__(outcomes = ['completed', 'failed'])

        self._topic = topic
        self._client = ProxyActionClient({self._topic: StageAction})

		# It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'failed'

        #otherwise get the result and perform next steps
        if(self._client.has_result(self._topic)):
            result = self._client.get_result(self._topic)
            status = result.result
            if(status == 0):
                print("Completed Succesfully")
                return 'completed'
            else:
                print("Error Thrown")
                return 'failed'


    def on_enter(self, userdata):
        #Creating the goal to send
        goal = StageGoal()
        goal.start = 0

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


        
