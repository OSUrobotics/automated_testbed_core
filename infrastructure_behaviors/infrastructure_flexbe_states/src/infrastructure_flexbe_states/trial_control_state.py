#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

class TrialControlState(EventState):
        '''
        Trial control takes in the trial information from Test control and on a succesful completion starts
        the Data Control. If all trials are completed then it will loop back to Test control. Direction is
        used for determining a successful and unsuccseful outcome for testing purposed but will need to be
        replaced with a different measure of success once it becomes more fleshed out. 
        TODO: More complex information for trials 

        -- rotation  int       TEMPORARY: gives rotation

        ># number_of_trials     Trial information (currently just an int)

        <= continue             All actions completed
        <= failed               Trial control failed to initialize or call something TODO: Proper error checking
        <= completed            All Trials have been succesfully completed, go back to Test control           

        '''

        def __init__(self):
            # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
            super(TrialControlState, self).__init__(outcomes = ["continue", "failed", "completed"], input_keys=["number_of_trials"])

            # Store state parameters for later use.
            self._number_of_trials = None


        def execute(self, userdata):
            #if trials remain return continue, if not return complete, if direction is 0 return failed
            if(self._number_of_trials > 0):
                #print(self._number_of_trials)
                self._number_of_trials -= 1
                return "continue"

            elif(self._number_of_trials <= 0):
                self._number_of_trials = None
                return "completed"

            else:
                return "failed"


        def on_enter(self, userdata):
            #Initializes class variable from userdata, has to be done outside of constructor 
            if(self._number_of_trials is None and userdata.number_of_trials is not None):
                self._number_of_trials = userdata.number_of_trials
