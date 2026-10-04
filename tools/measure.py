import math
import re
import subprocess

import rclpy
import tf2_ros
from rclpy.node import Node


def gz_pose(name):
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "%s"' % name)
    if i < 0:
        raise RuntimeError('%s not found in Gazebo pose list' % name)
    chunk = out[i:i + 800]

    def grab(block, axis):
        m = re.search(block + r' \{[^}]*?\b' + axis + r': ([-\d.eE+]+)', chunk, re.S)
        return float(m.group(1))

    px, py, pz = (grab('position', a) for a in 'xyz')
    q = {a: grab('orientation', a) for a in 'xyzw'}
    yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                     1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
    return px, py, pz, yaw


def main():
    rclpy.init()
    n = Node('measure')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)

    def pos(link):
        while not buf.can_transform('base_footprint', link, rclpy.time.Time()):
            rclpy.spin_once(n, timeout_sec=0.1)
        t = buf.lookup_transform('base_footprint', link, rclpy.time.Time()).transform.translation
        return t.x, t.y, t.z

    tool = pos('ur_tool0')
    lt = pos('rq_robotiq_85_left_finger_tip_link')
    rt = pos('rq_robotiq_85_right_finger_tip_link')
    mid = ((lt[0] + rt[0]) / 2.0, (lt[1] + rt[1]) / 2.0)

    rx, ry, rz, ryaw = gz_pose('warehouse_amr')
    cx, cy, cz, cyaw = gz_pose('pick_item')
    dx, dy = cx - rx, cy - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    cube = (c * dx + s * dy, -s * dx + c * dy)

    # closing axis u (towards the robot's left) and pad axis v (pointing forward)
    ux, uy = lt[0] - rt[0], lt[1] - rt[1]
    nrm = math.hypot(ux, uy)
    ux, uy = ux / nrm, uy / nrm
    if uy < 0:
        ux, uy = -ux, -uy
    vx, vy = uy, -ux
    theta = math.atan2(-ux, uy)

    def off(ref):
        ox, oy = cube[0] - ref[0], cube[1] - ref[1]
        return 1000 * (ox * ux + oy * uy), 1000 * (ox * vx + oy * vy)

    print('real robot yaw (Gazebo): %.1f deg' % math.degrees(ryaw))
    print('cube in robot frame (Gazebo): x=%.3f y=%.3f' % cube)
    print('tool0 in robot frame (TF):    x=%.3f y=%.3f' % tool[:2])
    print('tip midpoint (TF):            x=%.3f y=%.3f' % mid)
    print('tip midpoint minus tool0:     x=%.1f mm y=%.1f mm' %
          (1000 * (mid[0] - tool[0]), 1000 * (mid[1] - tool[1])))
    print('finger closing axis vs cube faces: %.1f deg' %
          math.degrees(ryaw + theta - cyaw))
    a, b = off(mid)
    print('cube offset from tip midpoint: closing axis %.1f mm, along pads %.1f mm' % (a, b))
    a, b = off(tool)
    print('cube offset from tool0:        closing axis %.1f mm, along pads %.1f mm' % (a, b))
    rclpy.shutdown()


main()
