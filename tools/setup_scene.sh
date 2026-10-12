#!/bin/bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
INIT="{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
for i in $(seq 1 60); do
  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$INIT" > /dev/null 2>&1
  s=$(ros2 lifecycle get /bt_navigator 2>/dev/null)
  echo "init try $i: $s"
  case "$s" in active*) break;; esac
  sleep 1
done
# one more initial pose now that Nav2 is up
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$INIT" > /dev/null 2>&1
sleep 2
ros2 run ros_gz_sim create -file ~/ros2_ws/src/warehouse_amr/worlds/pick_shelf.sdf -name pick_shelf -x 0.0 -y -3.0 -z 0.0
ros2 run ros_gz_sim create -file ~/ros2_ws/src/warehouse_amr/worlds/pick_item.sdf -name pick_item -x -0.08 -y -3.0 -z 0.582
echo "SCENE READY"
