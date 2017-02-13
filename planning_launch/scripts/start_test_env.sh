roslaunch planning_launch mockups_w_knowledge.launch &
sleep 5s
rostopic pub percepteros/object_detection suturo_perception_msgs/ObjectDetection "name: 'cylinder'
pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: 'odom_combined'
  pose:
    position: {x: 4.0, y: 6.0, z: 0.5}
    orientation: {x: 3.0, y: 8.0, z: 6.0, w: 4.0}
type: 1
width: 7.0
height: 5.0
depth: 88.0" -r 15
