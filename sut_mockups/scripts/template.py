#!/usr/bin/env python
"""
The template module.

This module has three purposes:
    1. Provide basic classes to be used as is to make simple mockups of publishers, services and actions.
    2. Provide base classes for more complex mockups.
    3. Showcase simple examples of mockups.
"""

import rospy

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
        :rate: Rate at which the messages shoul be published in hz. (int)
        """
        pub = rospy.Publisher(topic, msg_type, queue_size=queue_size)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            pub.publish(msg_gen())
            rate.sleep()


###########
# Service #
###########

class TemplateService(object):
    def __init__(self, name, srv):
        """Initialize a new service.
        :param name: The name of the service. (string)
        :param srv: The type of the service. (class)
        """
        self.srv = srv
        s = rospy.Service(name, srv, self.execute)

    def execute(self, req):
        """Return an empty response.

        For this to work you have to make sure, that additionally to the self.srv object it's
        corresponding Response is also available in the global scope. (eg. Empty and EmptyResponse)
        :param req: The request object.
        """
        return globals()[self.srv.__name__+"Response"]()


##########
# Action #
##########

import actionlib

class TemplateActionServer(object):
    def __init__(self, action_name, action):
        """
        Initialitze a new action server.
        :param action_name: name of the action, topics of the action will be named "<node_name>/<action_name>"
        :param action: msg object of the action, imported from respective msgs package
        """
        self.server = actionlib.SimpleActionServer(action_name, action, self.execute, False)
        self.server.start()


    def execute(self, goal):
        """
        Execute whatever action the server should perform. This method can be overridden to have more diverse behaviour.
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
