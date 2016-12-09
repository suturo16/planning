#!/usr/bin/env python

import rospy
import actionlib
import constants
from template import TemplateActionServer
from suturo_manipulation_msgs.msg import MoveRobotAction

class MoveRobotServer(TemplateActionServer):
    def __init__(self):
        super(MoveRobotServer, self).__init__(constants.move_robot, MoveRobotAction)

if __name__ == '__main__':
    rospy.init_node(constants.graspkard, anonymous=True)
    server = MoveRobotServer()
    rospy.spin()