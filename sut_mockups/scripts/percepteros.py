#!/usr/bin/env python

import rospy
import constants
from template import TemplateService, TemplatePublisher
from suturo_perception_msgs.msg import ObjectDetection
from suturo_perception_msgs.srv import RunPipeline, RunPipelineResponse
from object_detection_pub import detection_gen

if __name__ == '__main__':
    rospy.init_node('percepteros')
    service = TemplateService("~" + constants.set_pipeline, RunPipeline, RunPipelineResponse)
    pub = TemplatePublisher("~" + constants.object_detection, ObjectDetection, detection_gen(), rate=5)
    rospy.spin()