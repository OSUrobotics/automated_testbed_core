#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

class GenericSub(EventState):
        '''
        GenericPubSub state subscribes to a given topic (Int32 message only) and will not move forward until a response is recieved.
        This is meant to be a template to be modified to your needs it is not super useful in its current
        state. Intended mostly for communicating with arduino    

        '''

        def __init__(self, subtopic):
            # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
            super(GenericSub, self).__init__(outcomes = ["completed", "failed"])

            # Store state parameters for later use.
            self._subtopic = subtopic

            self._sub = ProxySubscriberCached({self._subtopic: Int32})
            #rospy.init_node('reset_control', anonymous=True) 

        def execute(self, userdata):
            #publish to arduino

            #while(1):
            if self._sub.has_msg(self._subtopic):
                msg = self._sub.get_last_msg(self._subtopic)
                print(msg)
                # in case you want to make sure the same message is not processed twice:
                self._sub.remove_last_msg(self._subtopic)
                return "completed"            

