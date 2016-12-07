#!/usr/bin/env python
import rospy
from std_srvs.srv import *
from constants import *

class ParamServer():
    def __init__(self):
        rospy.init_node(PARAM_NODE, anonymous=True)
        rospy.Service('~all_success', Empty, self.all_success)
        rospy.Service('~all_fail', Empty, self.all_fail)

        self.all_success(Empty())

        rospy.spin()


    def all_success(self, request):
        for node in nodes2names.keys():
            for name in nodes2names[node]:
                rospy.set_param(get_param_name(node, name), 1)

        return EmptyResponse()


    def all_fail(self, request):
        for node in nodes2names.keys():
            for name in nodes2names[node]:
                rospy.set_param(get_param_name(node, name), 0)

        return EmptyResponse()



if __name__ == '__main__':
    ParamServer()
