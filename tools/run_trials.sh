#!/bin/bash
# Usage: run_trials.sh N   (shelf and robot must already be set up; MoveIt running)
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
N=${1:-5}
for i in $(seq 1 $N); do
  echo "=== trial $i of $N ==="
  gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 --req 'name: "pick_item" type: MODEL' > /dev/null 2>&1
  sleep 1
  X=$(python3 -c "import random; print(round(-0.08 + random.uniform(-0.01, 0.01), 3))")
  Y=$(python3 -c "import random; print(round(-3.0 + random.uniform(-0.01, 0.01), 3))")
  YAW=$(python3 -c "import random, math; print(round(math.radians(random.uniform(-10, 10)), 3))")
  ros2 run ros_gz_sim create -file ~/ros2_ws/src/warehouse_amr/worlds/pick_item.sdf -name pick_item -x $X -y $Y -z 0.582 -Y $YAW > /dev/null 2>&1
  sleep 2
  python3 ~/ros2_ws/src/warehouse_amr/tools/vision_pick.py 2>&1 | grep -E "DETECTED|re-detect|AT GRASP|RESULT|failed|Path blocked"
done
echo "=== done; results in ~/vision_trials.csv ==="
