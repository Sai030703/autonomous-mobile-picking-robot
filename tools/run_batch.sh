#!/bin/bash
# usage: ~/run_batch.sh 4     (needs T1-T6 running; orchestrator in T6)
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
N=${1:-3}
OUT=~/batch_results.txt
echo "run | order | result | seconds" | tee "$OUT"
for i in $(seq 1 $N); do
  ID="BATCH$(date +%H%M%S)"
  echo "=== run $i/$N: resetting scene"
  ~/reset_scene.sh > /dev/null 2>&1
  sleep 6
  : > /tmp/status_stream.txt
  ros2 topic echo /amr/status --field data > /tmp/status_stream.txt 2>/dev/null &
  EPID=$!
  sleep 2
  START=$(date +%s)
  ros2 topic pub --once /warehouse/order std_msgs/msg/String \
    "{data: '{\"orderId\": \"$ID\", \"item\": \"red_cube\", \"quantity\": 1}'}" > /dev/null
  RES="TIMEOUT"
  for t in $(seq 1 450); do
    sleep 2
    if [ $t -eq 30 ] && ! grep -q NAVIGATING_TO_SHELF /tmp/status_stream.txt; then
      echo "  warning: no NAVIGATING_TO_SHELF seen after 60 s; status stream tail:"
      tail -3 /tmp/status_stream.txt
      RES="NO_STATUS"; break
    fi
    R=$(awk '/NAVIGATING_TO_SHELF/{s=1} s && /DONE|FAILED/{print; exit}' /tmp/status_stream.txt | grep -oE "DONE|FAILED" | head -1)
    if [ -n "$R" ]; then RES=$R; break; fi
  done
  kill $EPID 2>/dev/null
  SEC=$(( $(date +%s) - START ))
  [ $SEC -lt 60 ] && RES="$RES (SUSPECT: <60s)"
  echo "$i | $ID | $RES | $SEC" | tee -a "$OUT"
  sleep 10
done
echo; echo "summary:"; grep -c " DONE |" "$OUT" | xargs echo "DONE count:"
