#!/usr/bin/env python
"""
The template module.

This module has three purposes:
    1. Provide basic classes to be used as is to make simple mockups of publishers, services and actions.
    2. Provide base classes for more complex mockups.
    3. Showcase simple examples of mockups.
"""

import rospy
import threading

#############
# Publisher #
#############

class TemplatePublisher(object):
    def __init__(self, topic, msg_type, msg_gen, queue_size=10, rate=10):
        """Initialize a new publisher.
        :param topic: The name of the topic, on which the messages should be published. (string)
        :param msg_type: The type of the message. (class)
        :param msg_gen: A function generating the messages to be published. (function)
        :queue_size: Like rospy.Publisher
        :rate: Rate at which the messages should be published in hz. (int)
        """
        self.pub = rospy.Publisher(topic, msg_type, queue_size=queue_size)
        self.rate = rospy.Rate(rate)  # 10hz
        self.msg_gen = msg_gen
        threading.Thread(target=self.publish).start()

    def publish(self):
        """Start publishing."""
        while not rospy.is_shutdown():
            self.pub.publish(self.msg_gen())
            self.rate.sleep()


###########
# Service #
###########

class TemplateService(object):
    def __init__(self, name, srv, srv_resp):
        """Initialize a new service.
        :param name: The name of the service. (string)
        :param srv: The type of the service. (class)
        """
        self.name = name
        self.srv = srv
        self.srv_resp = srv_resp
        self.service = rospy.Service(name, srv, self.execute)

    def execute(self, req):
        """Return an empty response."""
        rospy.loginfo("Service {} received request:\n{}".format(rospy.resolve_name(self.name), req))
        return self.srv_resp()


##########
# Action #
##########

import actionlib

class TemplateActionServer(object):
    def __init__(self, action_name, action):
        """Initialitze a new action server.
        :param action_name: name of the action, topics of the action will be named "<node_name>/<action_name>"
        :param action: msg object of the action, imported from respective msgs package
        """
        self.server = actionlib.SimpleActionServer(action_name, action, self.execute, False)
        self.server.start()

    def feedback(self):
        pass

    def execute(self, goal):
        """Execute whatever action the server should perform. This method can be overridden to have more diverse behaviour.
        :param goal: The requested goal for the action.
        """
        self.server.set_succeeded()


###########
# EXAMPLE #
###########

# This is merely an example of how a main method of a real mockup could look
# from example_msgs.msg import ExampleMsg
# from example_srvs.srv import ExampleSrv, ExampleSrvResponse
# from example_actions.msg import ExampleAction
#
# if __name__ == '__main__':
#     rospy.init_node('template')
#     publisher = TemplatePublisher("~example_topic", ExampleMsg, lambda: ExampleMsg())
#     service = TemplateService("~example_service", ExampleSrv)
#     server = TemplateActionServer("~do_example", ExampleAction)
#     rospy.spin()
