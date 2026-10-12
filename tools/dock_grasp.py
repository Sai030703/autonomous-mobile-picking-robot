"""Usage:
  dock_grasp.py plan [zoff]      print where the cube is relative to the robot, move nothing
  dock_grasp.py descend [zoff]   add shelf to MoveIt, go above the cube, descend (gripper stays open)
  dock_grasp.py grab             close in two steps, then lift 15 cm
zoff = how far base_footprint sits above the world floor (default 0)."""
import math
import re
import os
import subprocess
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

FRAME = 'base_footprint'
GROUP = 'ur_manipulator'
TIP = 'ur_tool0'
ARM = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
       'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
PAD = 0.03
APPROACH = 0.10
GRIPPER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gripper.py')

# World (= map) positions of the spawned models. Must match the spawn commands.
SHELF_W = (0.0, -3.0)
SHELF_Z = 0.275
SHELF_SIZE = (0.40, 0.80, 0.55)
CUBE_W = (-0.08, -3.0)
CUBE_Z = 0.58



def gz_robot_pose():
    import subprocess as sp
    out = sp.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                 capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "warehouse_amr"')
    chunk = out[i:i + 800]
    def grab(block, axis):
        return float(re.search(block + r' \{[^}]*?\b' + axis + r': ([-\d.eE+]+)', chunk, re.S).group(1))
    q = {a: grab('orientation', a) for a in 'xyzw'}
    yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
    return grab('position', 'x'), grab('position', 'y'), yaw

def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_robot(w, rob):
    rx, ry, yaw = rob
    dx, dy = w[0] - rx, w[1] - ry
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def down_quat(alpha):
    # tool pointing down (the orientation that worked before), turned by alpha about vertical
    a = 0.7071068
    c, s = math.cos(alpha / 2.0), math.sin(alpha / 2.0)
    return a * (c - s), a * (c + s), 0.0, 0.0


class DockGrasp(Node):
    def __init__(self):
        super().__init__('dock_grasp')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.cart_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.move_cli = ActionClient(self, MoveGroup, '/move_action')
        self.exe_cli = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def tf(self, target, source, wait=8.0):
        t0 = time.time()
        while time.time() - t0 < wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.buf.can_transform(target, source, rclpy.time.Time()):
                return self.buf.lookup_transform(target, source, rclpy.time.Time()).transform
        raise RuntimeError('no TF %s <- %s' % (target, source))

    def call(self, cli, req, label, timeout=20.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('service not available: ' + label)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            raise RuntimeError('no response from ' + label)
        return fut.result()

    def robot_pose(self):
        t = self.tf('map', FRAME)
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def tcp(self):
        zs = [self.tf(TIP, 'rq_robotiq_85_%s_finger_tip_link' % s).translation.z for s in ('left', 'right')]
        origin = sum(zs) / 2.0
        return origin + PAD, origin

    def add_shelf(self, x, y, z, yaw, height=None):
        co = CollisionObject()
        co.header.frame_id = FRAME
        co.id = 'pick_shelf'
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [SHELF_SIZE[0], SHELF_SIZE[1], height if height is not None else SHELF_SIZE[2]]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        req = ApplyPlanningScene.Request()
        req.scene.world.collision_objects.append(co)
        req.scene.is_diff = True
        print('shelf in planning scene:', self.call(self.scene_cli, req, '/apply_planning_scene').success)

    def set_box(self, obj_id, x, y, z, size, remove=False):
        co = CollisionObject()
        co.header.frame_id = FRAME
        co.id = obj_id
        if remove:
            co.operation = CollisionObject.REMOVE
        else:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = list(size)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            co.primitives.append(prim)
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD
        req = ApplyPlanningScene.Request()
        req.scene.world.collision_objects.append(co)
        req.scene.is_diff = True
        return self.call(self.scene_cli, req, '/apply_planning_scene').success

    def ik(self, pose):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = TIP
        r.pose_stamped.header.frame_id = FRAME
        r.pose_stamped.pose = pose
        r.avoid_collisions = True
        r.timeout.sec = 2
        res = self.call(self.ik_cli, req, '/compute_ik')
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            print('IK failed, error code', res.error_code.val)
            return None
        js = res.solution.joint_state
        return {n: p for n, p in zip(js.name, js.position) if n in ARM}

    def move(self, joints, plan_only=False):
        if not self.move_cli.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('move_group action server not available')
        goal = MoveGroup.Goal()
        rq = goal.request
        rq.group_name = GROUP
        rq.num_planning_attempts = 5
        rq.allowed_planning_time = 5.0
        rq.max_velocity_scaling_factor = 0.2
        rq.max_acceleration_scaling_factor = 0.2
        c = Constraints()
        for name, pos in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        rq.goal_constraints.append(c)
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        fut = self.move_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print('goal rejected')
            return False
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=90.0)
        code = rf.result().result.error_code.val
        print('pre-grasp move error code:', code, '(1 = success)')
        return code == MoveItErrorCodes.SUCCESS

    def line_to(self, pose, scale=0.1, _retry=True):
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.group_name = GROUP
        req.link_name = TIP
        req.waypoints = [pose]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
        res = self.call(self.cart_cli, req, '/compute_cartesian_path')
        print('cartesian path: %.0f%% achieved' % (res.fraction * 100))
        if res.fraction < 0.99:
            print('Path blocked. Nothing was moved.')
            return False
        if not self.exe_cli.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('/execute_trajectory not available')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        fut = self.exe_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print('execution rejected')
            return False
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=90.0)
        code = rf.result().result.error_code.val
        print('execution error code:', code, '(1 = success)')
        if code == MoveItErrorCodes.SUCCESS:
            return True
        if _retry:
            print('execution failed (code %s), recomputing and retrying once' % code)
            time.sleep(1.0)
            return self.line_to(pose, scale, _retry=False)
        return False


def grip(q):
    subprocess.run([sys.executable, GRIPPER, str(q)])
    time.sleep(2.5)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ''
    zoff = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    if mode not in ('plan', 'descend', 'look', 'grab'):
        print(__doc__)
        return
    rclpy.init()
    n = DockGrasp()

    if mode == 'grab':
        grip(0.2)
        grip(0.3)
        cur = n.tf(FRAME, TIP)
        pose = Pose()
        pose.position.x = cur.translation.x
        pose.position.y = cur.translation.y
        pose.position.z = cur.translation.z + 0.15
        pose.orientation = cur.rotation
        n.line_to(pose)
        return

    rob = gz_robot_pose() if os.environ.get('ORACLE') == '1' else n.robot_pose()
    print('pose source:', 'GAZEBO (oracle)' if os.environ.get('ORACLE') == '1' else 'AMCL')
    cx, cy = to_robot(CUBE_W, rob)
    sx, sy = to_robot(SHELF_W, rob)
    cz, shz = CUBE_Z - zoff, SHELF_Z - zoff
    alpha = -rob[2]
    alpha = (alpha + math.pi / 4) % (math.pi / 2) - math.pi / 4   # cube is symmetric every 90 deg
    tcp, origin = n.tcp()
    print('robot in map: x=%.3f y=%.3f yaw=%.1f deg' % (rob[0], rob[1], math.degrees(rob[2])))
    print('cube in robot frame:  x=%.3f y=%.3f z=%.3f' % (cx, cy, cz))
    print('shelf in robot frame: x=%.3f y=%.3f z=%.3f' % (sx, sy, shz))
    print('gripper turn about vertical: %.1f deg | fingertip origin %.3f below tool0, TCP %.3f' %
          (math.degrees(alpha), origin, tcp))
    print('pre-grasp tool0 height: %.3f m' % (cz + tcp + APPROACH))
    if mode == 'plan':
        print('plan only: nothing moved.')
        return

    n.add_shelf(sx, sy, shz, -rob[2])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = cx, cy, cz + tcp + APPROACH
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = down_quat(alpha)
    joints = n.ik(pose)
    if joints is None:
        print('Pre-grasp unreachable or in collision.')
        return
    grip(0.0)
    if not n.move(joints):
        return
    if mode == 'look':
        print('Look pose reached: gripper open, tool pointing down, 10 cm above the grasp height.')
        return
    cur = n.tf(FRAME, TIP)
    pose2 = Pose()
    pose2.position.x = cur.translation.x
    pose2.position.y = cur.translation.y
    pose2.position.z = cz + tcp
    pose2.orientation = cur.rotation
    n.line_to(pose2)
    print('Stopped above the cube with the gripper open. Look before running grab.')


if __name__ == '__main__':
    main()
