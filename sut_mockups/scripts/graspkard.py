#!/usr/bin/env python

import rospy
import actionlib
import constants
import random
from template import TemplateActionServer
from suturo_manipulation_msgs.msg import MoveRobotAction, MoveRobotFeedback

class MovementServer(TemplateActionServer):
    def __init__(self):
        super(MovementServer, self).__init__("~" + "movement_server", MoveRobotAction)

    def execute(self, goal):
        rospy.loginfo("Got goal.")
        r = rospy.Rate(1)
        feedback = MoveRobotFeedback()
        feedback.current_value = 0.35
        # feedback.alteration_rate = 0.5
        param_name = constants.get_param_name(constants.graspkard, "instant")
        if rospy.get_param(param_name, True):
            while True:
                if self.server.is_preempt_requested():
                    self.server.set_preempted()
                    break
                feedback.current_value = 0
                feedback.alteration_rate = 0
                self.server.publish_feedback(feedback)
                r.sleep()
        else:
            while True:
                if self.server.is_preempt_requested():
                    self.server.set_preempted()
                    break
                if feedback.current_value > 0:
                    feedback.alteration_rate = round(random.uniform(0.05, 0.08), 2)
                    feedback.current_value -= feedback.alteration_rate
                self.server.publish_feedback(feedback)
                r.sleep()

if __name__ == '__main__':
    rospy.init_node(constants.graspkard, anonymous=True)
    server = MovementServer()
    rospy.spin()
