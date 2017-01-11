#!/usr/bin/env python

MAP_FRAME = "/map"

# Nodes
PARAM_NODE = "sut_mock_params"

graspkard = "graspkard"
simple_logger = "simple_logger"

# Topics
pepper_command = "pepper_command"

# Services
log_action = "log_action"
log_experiment_description = "log_experiment_description"

# Actions
move_robot = "move_robot"

# Convenience dictionary
nodes2names = {
    graspkard: [move_robot],
    simple_logger: [log_action, log_experiment_description],
}


def get_param_name(node, name):
    return "/".join([PARAM_NODE, node, name])
