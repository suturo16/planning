#!/usr/bin/env python

import rospy
import constants
from template import TemplateService
from suturo_perception_msgs.srv import RunPipeline, RunPipelineResponse

if __name__ == '__main__':
    rospy.init_node('percepteros')
    service = TemplateService("~" + constants.set_pipeline, RunPipeline, RunPipelineResponse)
    rospy.spin()