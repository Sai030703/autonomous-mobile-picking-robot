import csv
import math
import os
import re
import statistics
import subprocess
import sys
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

DOCK = (-0.8, -3.0, 0.0)
CUBE = (-0.08, -3.0)
# staging poses inside the free area found on the map, with different headings
STAGES = [(-1.6, -2.7, 0.0), (-1.6, -3.3, math.pi), (-1.6, -3.0, math.pi / 2)]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def gz_robot():
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "warehouse_amr"')
    if i < 0:
        raise RuntimeError('warehouse_amr not in Gazebo pose list')
    chunk = out[i:i + 800]

    def grab(block, axis):
        return float(re.search(block + r' \{[^}]*?\b' + axis + r': ([-\d.eE+]+)', chunk, re.S).group(1))

    q = {a: grab('orientation', a) for a in 'xyzw'}
    yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
    return grab('position', 'x'), grab('position', 'y'), yaw


def to_robot(p, pose):
    dx, dy = p[0] - pose[0], p[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


class Stats(Node):
    def __init__(self):
        super().__init__('dock_stats')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.cli = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def nav(self, x, y, yaw, timeout=150.0):
        if not self.cli.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/navigate_to_pose not available')
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        f = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=15.0)
        h = f.result()
        if h is None or not h.accepted:
            return None
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=timeout)
        return r.result().status if r.result() is not None else None

    def amcl(self):
        t0 = time.time()
        while time.time() - t0 < 8.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
                t = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
                q = t.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return t.translation.x, t.translation.y, yaw
        raise RuntimeError('no map->base_footprint TF')


def main():
    runs = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    rclpy.init()
    node = Stats()
    rows = []
    path = os.path.expanduser('~/dock_stats.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run', 'stage_ok', 'dock_ok', 'dock_err_mm', 'cube_fwd_err_mm',
                    'cube_side_err_mm', 'yaw_err_deg'])
        for i in range(runs):
            sx, sy, syaw = STAGES[i % len(STAGES)]
            s1 = node.nav(sx, sy, syaw)
            s2 = node.nav(*DOCK)
            time.sleep(3.0)
            a, g = node.amcl(), gz_robot()
            ca, cg = to_robot(CUBE, a), to_robot(CUBE, g)
            fwd, side = 1000 * (ca[0] - cg[0]), 1000 * (ca[1] - cg[1])
            yaw = math.degrees(wrap(a[2] - g[2]))
            derr = 1000 * math.hypot(g[0] - DOCK[0], g[1] - DOCK[1])
            ok1, ok2 = s1 == GoalStatus.STATUS_SUCCEEDED, s2 == GoalStatus.STATUS_SUCCEEDED
            print('run %d: stage %s dock %s | real dock error %.0f mm | AMCL cube error: '
                  'along pads %+.1f mm, across fingers %+.1f mm | yaw error %+.1f deg'
                  % (i + 1, 'ok' if ok1 else 'FAIL', 'ok' if ok2 else 'FAIL', derr, fwd, side, yaw))
            w.writerow([i + 1, ok1, ok2, round(derr, 1), round(fwd, 1), round(side, 1), round(yaw, 2)])
            f.flush()
            if ok2:
                rows.append((fwd, side, yaw))
    if len(rows) >= 2:
        for k, name in enumerate(['along pads (mm)', 'across fingers (mm)', 'yaw (deg)']):
            v = [r[k] for r in rows]
            print('%-20s mean %+.1f  std %.1f  max |.| %.1f  (n=%d)' %
                  (name, statistics.mean(v), statistics.pstdev(v), max(abs(x) for x in v), len(v)))
    print('saved', path)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
