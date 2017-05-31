#!/usr/bin/env python 

import rospy
import tf
import thread


def broadcast():
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 1, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 1),
                     rospy.Time.now(),
                     "spatula_shovel",
                     "spatula1")


transforms = [
    {
        'name':'spatula1_shovel',
        'parent': 'spatula1'
    },
    {
        'name':'spatula1_handle',
        'parent':'spatula1'
    },
    
    {
        'name':'r_wrist_roll_link',
        'parent':'base_link'
    },
    {
        'name':'l_wrist_roll_link',
        'parent':'base_link'
    },
    {
        'name':'base_link',
        'parent':'odom_combined'
    },


    {
        'name':'next2cake',
        'parent':'box1',
        'pos':(0,1,0)
    },
    {
        'name':'deliver',
        'parent':'map',
        'pos':(1,1,2)
    },
    {
        'name':'map',
        'parent':'odom_combined',
        'pos':(0,1,0)
    }
]

def start_broadcast():
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        for t in transforms:
            pos = t['pos'] if 'pos' in t else (1,0,0)
            rot = t['rot'] if 'rot' in t else (0,0,0,1)
            
            br.sendTransform(pos, rot, rospy.Time.now(), t['name'], t['parent'])
        rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('pr2_tf_broadcaster')
    thread.start_new_thread(start_broadcast, ())
    rospy.spin()