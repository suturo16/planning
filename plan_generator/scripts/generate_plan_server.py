#!/usr/bin/env python

from plan_generator.srv import *
import rospy
import sys
import os
from downward.driver.main import main
import json_transformation
import rospkg

def handle_generate_plan(req):

    # get absolute path to input files
    rospack = rospkg.RosPack()
    path = rospack.get_path('plan_generator') + '/pddl/'
    
    domain_path = path + req.domain
    task_path = path + "task.pddl"
    
    create_task_file(task_path,req.task)
    
    
    # initialize planner and run it
    sys.argv = ['fast-downward.py', domain_path, task_path, '--search', 'astar(lmcut())']
    main()
    
    # transform plan output to json format
    plan = open('sas_plan', 'r').read()
    json_plan = json_transformation.transform_plan_to_json_string(plan)
    
    # remove output files of plan generation
    os.remove('sas_plan')
    os.remove('output.sas')
    
    return GeneratePlanResponse(json_plan)

def generate_plan_server():
    rospy.init_node('generate_plan_server')
    s = rospy.Service('generate_plan', GeneratePlan, handle_generate_plan)
    print("Ready to generate a plan.")
    rospy.spin()
   
   
def create_task_file(path, task):
  text_file = open(path, "w")
  text_file.write(task)
  text_file.close()


    

if __name__ == "__main__":
    generate_plan_server()
