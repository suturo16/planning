#!/usr/bin/env python

import rospy
import constants
from std_msgs.msg import String
from template import TemplatePublisher

if __name__ == '__main__':
    rospy.init_node(constants.pepper_command, anonymous=True)
    pub = TemplatePublisher("pepper_command", String, lambda: String("right"), rate=1)
    rospy.spin()
