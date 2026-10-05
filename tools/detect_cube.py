import re
import subprocess

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
import tf2_geometry_msgs  # noqa: F401  registers PointStamped with tf2

FX = FY = 379.9173
CX, CY = 320.0, 240.0
OPTICAL = 'wrist_camera_optical_link'
TARGET = 'base_footprint'


def gz_pose(name):
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "%s"' % name)
    chunk = out[i:i + 800]
    g = lambda blk, ax: float(re.search(blk + r' \{[^}]*?\b' + ax + r': ([-\d.eE+]+)', chunk, re.S).group(1))
    return g('position', 'x'), g('position', 'y'), g('position', 'z')


def main():
    rclpy.init()
    n = Node('detect_cube')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    got = {}
    n.create_subscription(Image, '/wrist_camera/image', lambda m: got.setdefault('rgb', m), 1)
    n.create_subscription(Image, '/wrist_camera/depth_image', lambda m: got.setdefault('depth', m), 1)
    while len(got) < 2:
        rclpy.spin_once(n, timeout_sec=0.5)
    r, d = got['rgb'], got['depth']
    rgb = np.frombuffer(r.data, np.uint8).reshape(r.height, r.width, 3).astype(int)
    dep = np.frombuffer(d.data, np.float32).reshape(d.height, d.width)

    redness = rgb[:, :, 0] - np.maximum(rgb[:, :, 1], rgb[:, :, 2])
    mask = redness > 60
    if mask.sum() < 200:
        print('Cube not found: only %d red pixels' % mask.sum())
        return
    ys, xs = np.nonzero(mask)
    u, v = xs.mean(), ys.mean()
    z = float(np.median(dep[mask]))
    print('blob: %d px, centre u=%.1f v=%.1f, median depth %.3f m' % (mask.sum(), u, v, z))

    p = PointStamped()
    p.header.frame_id = OPTICAL
    p.point.x = (u - CX) / FX * z
    p.point.y = (v - CY) / FY * z
    p.point.z = z
    print('point in optical frame: x=%.3f y=%.3f z=%.3f' % (p.point.x, p.point.y, p.point.z))
    while not buf.can_transform(TARGET, OPTICAL, rclpy.time.Time()):
        rclpy.spin_once(n, timeout_sec=0.2)
    out = buf.transform(p, TARGET)
    q = out.point
    print('MEASURED cube top-face centre in %s: x=%.3f y=%.3f z=%.3f' % (TARGET, q.x, q.y, q.z))

    # truth from Gazebo, using Gazebo's own robot yaw (no AMCL involved)
    def gz_full(name):
        out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                             capture_output=True, text=True, timeout=30).stdout
        i = out.find('name: "%s"' % name)
        chunk = out[i:i + 800]
        g = lambda blk, ax: float(re.search(blk + r' \{[^}]*?\b' + ax + r': ([-\d.eE+]+)', chunk, re.S).group(1))
        qq = {a: g('orientation', a) for a in 'xyzw'}
        yw = np.arctan2(2 * (qq['w'] * qq['z'] + qq['x'] * qq['y']), 1 - 2 * (qq['y'] ** 2 + qq['z'] ** 2))
        return g('position', 'x'), g('position', 'y'), g('position', 'z'), yw
    rx, ry, rz, ryaw = gz_full('warehouse_amr')
    cx, cy, cz, _ = gz_full('pick_item')
    dx, dy = cx - rx, cy - ry
    tx = np.cos(ryaw) * dx + np.sin(ryaw) * dy
    ty = -np.sin(ryaw) * dx + np.cos(ryaw) * dy
    print('TRUE cube centre (Gazebo, real yaw %.1f deg): x=%.3f y=%.3f z=%.3f (cube z in world %.3f, robot z %.3f)'
          % (np.degrees(ryaw), tx, ty, cz - rz, cz, rz))
    print('ERROR measured - true: x %+.1f mm, y %+.1f mm, top-face z vs true centre+0.03: %+.1f mm'
          % (1000 * (q.x - tx), 1000 * (q.y - ty), 1000 * (q.z - (cz - rz + 0.03))))
    rclpy.shutdown()


main()
