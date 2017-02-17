#!/usr/bin/env python

MAP_FRAME = "/map"

# Nodes
PARAM_NODE = "sut_mock"

graspkard = "graspkard"
simple_logger = "simple_logger"
percepteros = "percepteros"

# Topics
pepper_command = "pepper_command"
object_detection = "object_detection"

# Services
log_action = "log_action"
log_experiment_description = "log_experiment_description"

set_pipeline = "set_pipeline"

# Actions
move_robot = "move_robot"

# Convenience dictionary
nodes2names = {
    graspkard: [move_robot],
    simple_logger: [log_action, log_experiment_description],
    percepteros: [object_detection, set_pipeline],
}


def get_param_name(node, name):
    return "/".join([PARAM_NODE, node, name])
