#!/usr/bin/env python

# This import is not needed in this module, but will be in every module running a ROS node.
# import rospy
import actionlib

class TemplateActionServer(object):
    def __init__(self, action_name, action):
        """
        Initialitze a new action server.
        :param action_name: name of the action, topics of the action will be named "<node_name>/<action_name>"
        :param action: msg object of the action, imported from respective msgs package
        """
        self.server = actionlib.SimpleActionServer('~' + action_name, action, self.execute, False)
        self.server.start()


    def execute(self, goal):
        """
        Execute whatever action the server should perform. This method can be overridden to have more diverse behaviour.
        """
        self.server.set_succeeded()

# This is merely an example of how a main method of a real mockup could look
# if __name__ == '__main__':
#     rospy.init_node('template')
#     server = TemplateActionServer("example", ExampleAction)
#     rospy.spin()