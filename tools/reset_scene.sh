#!/bin/bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1/3 re-seed AMCL from Gazebo truth (simulation only)"
python3 ~/ros2_ws/src/warehouse_amr/tools/reseed_amcl.py
sleep 2

echo "2/3 put the cube back on the shelf"
gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "pick_item", position: {x: -0.08, y: -3.0, z: 0.582}, orientation: {w: 1}'

gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "station_table" type: MODEL'
sleep 1
ros2 run ros_gz_sim create -file ~/ros2_ws/src/warehouse_amr/worlds/station_table.sdf -name station_table -x -3.8 -y 0.0 -z 0.0 > /dev/null 2>&1
echo "3/3 drive the robot to the shelf dock"
for i in 1 2 3; do
  out=$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: -0.8, y: -3.0, z: 0.0}, orientation: {w: 1.0}}}}" 2>&1)
  if echo "$out" | grep -q "SUCCEEDED"; then echo "robot at the shelf dock"; break; fi
  echo "dock attempt $i did not succeed, retrying"
  sleep 2
done

echo "cube now:"
gz topic -e -t /world/default/dynamic_pose/info -n 1 | grep -A6 'name: "pick_item"' | grep -E 'x:|y:|z:' | head -3
