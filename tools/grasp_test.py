import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node

FRAME = 'base_footprint'
GROUP = 'ur_manipulator'
TIP = 'ur_tool0'
CUBE = (0.72, 0.0, 0.58)   # cube centre on the shelf top, base_footprint frame
PAD = 0.03                 # same pad allowance as reach_test.py (a guess)


class Grasp(Node):
    def __init__(self):
        super().__init__('grasp_test')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exe = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def tf(self, target, source, wait=8.0):
        t0 = time.time()
        while time.time() - t0 < wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.buf.can_transform(target, source, rclpy.time.Time()):
                return self.buf.lookup_transform(target, source, rclpy.time.Time()).transform
        raise RuntimeError('no TF %s <- %s' % (target, source))

    def line_to(self, pose, scale=0.1):
        if not self.cart.wait_for_service(timeout_sec=15.0):
            raise RuntimeError('/compute_cartesian_path not available')
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
        fut = self.cart.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        res = fut.result()
        if res is None:
            print('no response from /compute_cartesian_path')
            return False
        print('cartesian path: %.0f%% achieved, error code %d' % (res.fraction * 100, res.error_code.val))
        if res.fraction < 0.99:
            print('Path blocked (collision or limits). Nothing was moved.')
            return False
        if not self.exe.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('/execute_trajectory not available')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        fut = self.exe.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print('execution goal rejected')
            return False
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=90.0)
        code = rf.result().result.error_code.val
        print('execution error code:', code, '(1 = success)')
        return code == MoveItErrorCodes.SUCCESS


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('descend', 'lift'):
        print('Usage: grasp_test.py descend [offset_m]   (offset raises the grasp point above cube centre)')
        print('       grasp_test.py lift [height_m]')
        return
    mode = sys.argv[1]
    val = float(sys.argv[2]) if len(sys.argv) > 2 else None

    rclpy.init()
    n = Grasp()
    cur = n.tf(FRAME, TIP)
    print('tool0 now: x=%.3f y=%.3f z=%.3f' % (cur.translation.x, cur.translation.y, cur.translation.z))

    pose = Pose()
    pose.position.x = cur.translation.x
    pose.position.y = cur.translation.y
    pose.orientation = cur.rotation   # keep the tool pointing down

    if mode == 'descend':
        offset = val if val is not None else 0.0
        if abs(cur.translation.x - CUBE[0]) > 0.03 or abs(cur.translation.y - CUBE[1]) > 0.03:
            print('Gripper is not above the cube. Run reach_test.py first.')
            return
        zs = [n.tf(TIP, 'rq_robotiq_85_%s_finger_tip_link' % s).translation.z for s in ('left', 'right')]
        tcp = sum(zs) / 2.0 + PAD
        pose.position.z = CUBE[2] + offset + tcp
        print('fingertip origin %.3f m below tool0, TCP %.3f -> target tool0 z=%.3f' %
              (sum(zs) / 2.0, tcp, pose.position.z))
        if pose.position.z >= cur.translation.z:
            print('Target is not below the current pose. Aborting.')
            return
    else:
        pose.position.z = cur.translation.z + (val if val is not None else 0.15)

    if n.line_to(pose):
        e = n.tf(FRAME, TIP)
        print('tool0 now: x=%.3f y=%.3f z=%.3f' % (e.translation.x, e.translation.y, e.translation.z))
    rclpy.shutdown()


main()
