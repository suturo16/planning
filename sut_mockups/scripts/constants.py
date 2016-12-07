#!/usr/bin/env python

MAP_FRAME = "/map"

# Nodes
PARAM_NODE = "sut_mock_params"

graspkard = "graspkard"

# Topics
pepper_command = "pepper_command"

# Services
# None at this point

# Actions
move_robot = "move_robot"

# Convenience dictionary
nodes2names = {
    graspkard: [move_robot]
}


def get_param_name(node, name):
    return "/".join([PARAM_NODE, node, name])
