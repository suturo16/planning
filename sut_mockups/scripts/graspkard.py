#!/usr/bin/env python

import rospy
import actionlib
import constants
from template import TemplateActionServer
from suturo_manipulation_msgs.msg import MoveRobotAction, MoveRobotFeedback

class MoveRobotServer(TemplateActionServer):
    def __init__(self):
        super(MoveRobotServer, self).__init__("~" + constants.move_robot, MoveRobotAction)

    def execute(self, goal):
        r = rospy.Rate(1)
        feedback = MoveRobotFeedback()
        feedback.current_value = 5
        feedback.alteration_rate = 1
        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                break
            if feedback.current_value > 0:
                feedback.current_value -= 1
            self.server.publish_feedback(feedback)
            r.sleep()


class PlaceObjectServer(TemplateActionServer):
    def __init__(self):
        super(PlaceObjectServer, self).__init__("~" + "place_object", MoveRobotAction)

    def execute(self, goal):
        r = rospy.Rate(1)
        feedback = MoveRobotFeedback()
        feedback.current_value = 5
        feedback.alteration_rate = 1
        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                break
            if feedback.current_value > 0:
                feedback.current_value -= 1
            self.server.publish_feedback(feedback)
            r.sleep()


class GripperServer(TemplateActionServer):
    def __init__(self):
        super(GripperServer, self).__init__("~" + "gripper", MoveRobotAction)

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
    server = MoveRobotServer()
    server = PlaceObjectServer()
    server = GripperServer()
    rospy.spin()