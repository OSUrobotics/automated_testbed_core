#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

class GenericPub(EventState):
        '''
        ArduinoPubSub state publishes an integer (Int32) once to the topic given.
        This is meant to be a template to be modified to your needs it is not super useful in its current
        state. Intended mostly for communicating with arduino    

        '''

        def __init__(self, pubtopic, pubint):
            # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
            super(GenericPub, self).__init__(outcomes = ["completed", "failed"])

            # Store state parameters for later use.
            self._pubtopic = pubtopic
            self._pubint = pubint

            self._pub = ProxyPublisher({self._pubtopic : Int32})
            #rospy.init_node('reset_control', anonymous=True) 

        def execute(self, userdata):
            return "completed"


        def on_enter(self, userdata):
            #self._rotation = 5
            #Initializes class variable from userdata, has to be done outside of constructor 
            self._pub.publish(self._pubtopic , self._pubint)

            