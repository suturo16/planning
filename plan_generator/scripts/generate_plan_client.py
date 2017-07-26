#!/usr/bin/env python

import sys
import rospy
from plan_generator.srv import *

def generate_plan_client(x, y):
    rospy.wait_for_service('generate_plan')
    try:
        generate_plan = rospy.ServiceProxy('generate_plan', GeneratePlan)
        resp1 = generate_plan(x, y)
        return resp1.plan
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = str(sys.argv[1])
        y = str(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print ("Requesting plan...")
    print ("Plan found: ")
    print (generate_plan_client(x, y))