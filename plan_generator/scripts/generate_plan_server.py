#!/usr/bin/env python

from plan_generator.srv import *
import rospy
import sys
from downward.driver.main import main

def handle_generate_plan(req):
    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    sys.argv = ['fast-downward.py', req.domain, req.task, '--search', 'astar(lmcut())']
    print(sys.argv)
    main()
    plan = open('sas_plan', 'r').read()
    return GeneratePlanResponse(plan)

def generate_plan_server():
    rospy.init_node('generate_plan_server')
    s = rospy.Service('generate_plan', GeneratePlan, handle_generate_plan)
    print("Ready to generate a plan.")
    rospy.spin()

if __name__ == "__main__":
    generate_plan_server()