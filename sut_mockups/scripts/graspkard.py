#!/usr/bin/env python

import rospy
import actionlib
import constants
from template import TemplateActionServer
from suturo_manipulation_msgs.msg import MoveRobotAction, MoveRobotFeedback

class MovementServer(TemplateActionServer):
    def __init__(self):
        super(MovementServer, self).__init__("~" + "movement_server", MoveRobotAction)

    def execute(self, goal):
        r = rospy.Rate(1)
        feedback = MoveRobotFeedback()
        feedback.current_value = 1
        feedback.alteration_rate = 1
        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                break
            if feedback.current_value > 0:
                feedback.current_value -= 1
            self.server.publish_feedback(feedback)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node(constants.graspkard, anonymous=True)
    server = MovementServer()
    rospy.spin()
