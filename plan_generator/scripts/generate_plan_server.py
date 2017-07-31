#!/usr/bin/env python

from plan_generator.srv import *
import rospy
import sys
from downward.driver.main import main
import json_transformation

def handle_generate_plan(req):
    sys.argv = ['fast-downward.py', req.domain, req.task, '--search', 'astar(lmcut())']
    print(sys.argv)
    main()
    plan = open('sas_plan', 'r').read()
    json_plan = json_transformation.transform_plan_to_json_string(plan)
    
    return GeneratePlanResponse(json_plan)

def generate_plan_server():
    rospy.init_node('generate_plan_server')
    s = rospy.Service('generate_plan', GeneratePlan, handle_generate_plan)
    print("Ready to generate a plan.")
    rospy.spin()


    

if __name__ == "__main__":
    generate_plan_server()
