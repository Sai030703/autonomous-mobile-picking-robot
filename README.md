# Autonomous Mobile Picking Robot (ROS 2 simulation)

A simulated warehouse robot that takes an order, drives to a shelf, picks a cube with a UR5e arm using a wrist camera, stows it in an onboard tote, drives to a packing station, and unloads it onto a table. **Simulation only.** Built with ROS 2 Lyrical and Gazebo Jetty.

**Demo video:** VIDEO_LINK

## What it does

An order (`red_cube`, quantity 1) goes through an orchestrator with these states:

`NAVIGATING_TO_SHELF -> PICKING -> PLACING (into the tote) -> NAVIGATING_TO_STATION -> UNLOADING -> DONE / FAILED`

Each stage has retries (dock 3, pick 3, place 2, deliver 3, unload 2). Status messages are published on `/amr/status`. The state model is *inspired by* VDA5050; it is **not** VDA5050-compliant.

| Part | Implementation |
|---|---|
| Mobile base | Differential drive, 2D LiDAR, AMCL localization, Nav2 (NavFn planner, MPPI controller, collision monitor) |
| Arm / gripper | UR5e + Robotiq 2F-85, MoveIt 2 (Cartesian paths, IK, joint-space moves) |
| Perception | Wrist RGB-D camera, red-cube detection, grasp pose in the robot frame |
| Transport | Onboard tote on the base; the arm places the cube in the tote and takes it out at the station |
| Orchestration | Python orchestrator (`tools/orchestrator.py`) with retries and checks per stage |

## Self-checks

- **Grasp check:** after closing, the finger knuckle angles are compared with the open position. If they did not close on an object, the pick is reported as failed.
- **Unload check:** after release, the cube pose is read from Gazebo; an unload only counts as successful if the cube is on the table. This check caught a real failure (cube dropped on the floor because no table was in the scene).
- **Controller aborts:** a Cartesian move that is aborted by the arm controller is recomputed and retried once.

## Results

Controlled batch of 4 consecutive orders (`tools/run_batch.sh`, scene reset before each run):

| Stage | Result |
|---|---|
| Dock at shelf | 4 / 4 on the first attempt |
| Pick | 4 / 4 within retries (3 / 4 on the first attempt; the miss stopped the arm 5 cm high and the retry succeeded) |
| Place in tote | 4 / 4 on the first attempt |
| Deliver to station | **2 / 4 orders** (1 / 4 on the first attempt) |
| Unload onto table | 2 / 2 where reached (one needed the abort-retry) |
| **End to end (DONE)** | **2 / 4** |

Both failed orders failed in the delivery drive. Completed orders took roughly 2.7 to 4.6 minutes in simulation.

## Known limitations

- **Delivery navigation is unreliable.** In failed runs the planner starts from a pose inside a wall region, so AMCL's estimate drifts away from the true robot position during the drive to the station. The first navigation goal also often fails (empty plan, "failed to make progress") and a retry succeeds. I suspect the station table, which is in Gazebo but not in the saved map, but I did not confirm this. Adding the table to the map and relaxing the collision monitor and progress checker did **not** fix it, so those changes were reverted.
- **Simulation-only shortcuts.** Before each navigation goal the orchestrator re-seeds AMCL from Gazebo ground truth (`tools/reseed_amcl.py`), and the unload check reads the cube pose from Gazebo. A real robot has neither.
- **Pick is not perfectly repeatable.** Roughly one pick in four stops about 5 cm above the grasp height and needs a retry. A looser arm controller goal tolerance is a suspect; not verified.
- **One item, one cube, one shelf.** The warehouse shelves in the world are scenery (AWS RoboMaker small-warehouse assets, see their license); only one custom shelf holds the pick object. Quantity > 1 and several items are not implemented.
- **Cube lands off-centre.** It ends up on the table but usually 15-30 cm from the nominal release point.

## How to run

Build and source the workspace, then use six terminals (each: `source /opt/ros/lyrical/setup.bash && source ~/ros2_ws/install/setup.bash`):

1. `ros2 launch warehouse_amr spawn_amr.launch.py` (Gazebo; wait until loaded)
2. `ros2 control list_controllers` (all four controllers `active`)
3. `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=$HOME/ros2_ws/src/warehouse_amr/maps/warehouse_map.yaml params_file:=$HOME/ros2_ws/src/warehouse_amr/config/nav2_params.yaml autostart:=true`
4. `tools/setup_scene.sh` (initial pose, shelf and cube; wait for `SCENE READY`)
5. `ros2 launch warehouse_amr moveit.launch.py`
6. `python3 tools/orchestrator.py`

Then `tools/reset_scene.sh` (puts the cube on the shelf, spawns the station table, drives to the dock) and send an order:

```bash
ros2 topic pub --once /warehouse/order std_msgs/msg/String \
  "{data: '{\"orderId\": \"DEMO1\", \"item\": \"red_cube\", \"quantity\": 1}'}"
```

`tools/run_batch.sh N` repeats reset + order N times and writes `batch_results.txt`. `tools/snap_camera.py` saves wrist-camera frames.

