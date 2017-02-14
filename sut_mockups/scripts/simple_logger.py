#!/usr/bin/env python

import rospy
import constants
from template import TemplateService
from suturo_knowledge_msgs.srv import LogAction, LogActionResponse, LogExperimentDescription, LogExperimentDescriptionResponse

if __name__ == '__main__':
    rospy.init_node('knowledge')
    service = TemplateService(constants.log_action, LogAction, LogActionResponse)
    service = TemplateService(constants.log_experiment_description, LogExperimentDescription, LogExperimentDescriptionResponse)
    rospy.spin()