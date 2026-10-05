#!/bin/bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
for i in $(seq 1 20); do
  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" > /dev/null 2>&1
  sleep 3
  s=$(ros2 lifecycle get /bt_navigator 2>/dev/null)
  echo "init try $i: $s"
  case "$s" in active*) break;; esac
done
ok=0
for i in 1 2 3; do
  out=$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -0.8, y: -3.0}, orientation: {w: 1.0}}}}" 2>&1)
  echo "dock attempt $i: $(echo "$out" | grep -E 'status|rejected' | head -1)"
  echo "$out" | grep -q SUCCEEDED && { ok=1; break; }
done
[ $ok -eq 1 ] || { echo "DOCK FAILED"; exit 1; }
ros2 run ros_gz_sim create -file ~/ros2_ws/src/warehouse_amr/worlds/pick_shelf.sdf -name pick_shelf -x 0.0 -y -3.0 -z 0.0
ros2 run ros_gz_sim create -file ~/ros2_ws/src/warehouse_amr/worlds/pick_item.sdf -name pick_item -x -0.08 -y -3.0 -z 0.582
echo "SCENE READY"
