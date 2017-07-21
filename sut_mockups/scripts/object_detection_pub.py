#!/usr/bin/env python

import rospy
import random
import math
from template import TemplatePublisher
from constants import percepteros, object_detection, get_param_name
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from suturo_perception_msgs.msg import ObjectDetection
from time import sleep

# TODO(cpo): Use something like dir(ObjectDetection) to dynamically get the types

class DetectedObject():
    def __init__(self, name, type, width, height, depth, pose):
        self.name = name
        self.type = type
        self.width = width
        self.height = height
        self.depth = depth
        self.pose = pose

    def to_msg(self, seq):
        return ObjectDetection(
            name = self.name,
            type = self.type,
            width = self.width,
            height = self.height,
            depth = self.depth,
            pose = PoseStamped(
                header = Header(
                    seq = seq,
                    stamp = rospy.Time.now(),
                    frame_id = 'map'
                ),
                pose = self.pose
            )
        )

def detection_gen():
    param_name = get_param_name(percepteros, object_detection)
    
    default_types = ['BOX',
                     'CONE',
                     'CYLINDER',
                     'DROPZONE',
                     'MISC',
                     'SPHERE',
                     'KNIFE',
                     'PLATE',
                     'SPATULA']
    default_type_map = {'BOX': 'box',
                     'CONE': 'cone',
                     'CYLINDER': 'cylinder',
                     'DROPZONE': 'dropzone',
                     'MISC': 'misc',
                     'SPHERE': 'sphere',
                     'KNIFE': 'cakeKnife',
                     'PLATE': 'dinnerPlateForCake',
                     'SPATULA': 'cakeSpatula'}
    data = {}
    global seq
    seq = 0

    def _create_obj(_type):
        num = len(data[_type])
        return DetectedObject(
            name = default_type_map[_type],
            type = getattr(ObjectDetection, _type, random.randint(len(default_types)+5, 100)),
            width = round(random.uniform(0.01, 0.5), 2),
            height = round(random.uniform(0.01, 0.5), 2),
            depth = round(random.uniform(0.01, 0.5), 2),
            pose = Pose(
                position = Point(
                    # Generate a random number three times and apply the resulting list
                    *[round(random.uniform(0, 10), 2) for _ in range(3)]
                ),
                orientation = Quaternion(
                    # Same as above, but 4 times
                    # *[round(random.random(), 2) for _ in range(4)]
                    0, 0, 0, 1
                )
            )
        )

    def _setup_data(_types, _obj_count):
        for t in _types:
            # Create type entry
            if t not in data:
                data[t] = []
            # Extend underful obj list
            while not len(data[t]) >= _obj_count:
                data[t].append(_create_obj(t))
            # Shorten overful obj list
            while not len(data[t]) <= _obj_count:
                data[t].pop()


    def _generator():
        # Which types should be published
        types = rospy.get_param(param_name + "/types", default_types)

        # How many different objects of each type should be published
        obj_count = rospy.get_param(param_name + "/obj_count", 1)

        # Fill the data dict
        _setup_data(types, obj_count)

        # Count up seq
        global seq
        seq += 1

        # Return one of the objects at random
        # return random.choice(
        #     data[random.choice(data.keys())]
        # ).to_msg(seq)
        return random.choice(data[data.keys()[(seq-1) % len(default_types)]]).to_msg(seq)

    return _generator




if __name__ == '__main__':
    rospy.init_node(percepteros)
    pub = TemplatePublisher("~" + object_detection, ObjectDetection, detection_gen(), rate=0.5)
    rospy.spin()