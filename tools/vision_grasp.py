import math
import subprocess
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
import tf2_geometry_msgs  # noqa: F401

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import dock_grasp as dg   # noqa: E402  (its main() is guarded by the patch below)

FX = FY = 379.9173
CX, CY = 320.0, 240.0
OPTICAL = 'wrist_camera_optical_link'
CUBE_HALF = 0.03


def detect(node, buf):
    got = {}
    node.create_subscription(Image, '/wrist_camera/image', lambda m: got.setdefault('rgb', m), 1)
    node.create_subscription(Image, '/wrist_camera/depth_image', lambda m: got.setdefault('depth', m), 1)
    t0 = time.time()
    while len(got) < 2 and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len(got) < 2:
        raise RuntimeError('no camera images')
    r, d = got['rgb'], got['depth']
    rgb = np.frombuffer(r.data, np.uint8).reshape(r.height, r.width, 3).astype(int)
    dep = np.frombuffer(d.data, np.float32).reshape(d.height, d.width)
    mask = (rgb[:, :, 0] - np.maximum(rgb[:, :, 1], rgb[:, :, 2])) > 60
    if mask.sum() < 500:
        raise RuntimeError('cube not found (%d red pixels)' % mask.sum())
    ys, xs = np.nonzero(mask)
    # orientation of the blob: principal axes of the pixel cloud are useless for a square,
    # so use the angle of the convex hull's minimum-area rectangle via brute-force rotation
    pts = np.stack([xs, ys], 1).astype(float)
    pts -= pts.mean(0)
    best = None
    for deg in np.arange(0, 90, 1.0):
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        rot = pts @ np.array([[c, -s], [s, c]])
        area = (rot[:, 0].max() - rot[:, 0].min()) * (rot[:, 1].max() - rot[:, 1].min())
        if best is None or area < best[0]:
            best = (area, deg)
    img_angle = math.radians(best[1])
    u, v = xs.mean(), ys.mean()
    z = float(np.median(dep[mask]))
    p = PointStamped()
    p.header.frame_id = OPTICAL
    p.point.x, p.point.y, p.point.z = (u - CX) / FX * z, (v - CY) / FY * z, z
    while not buf.can_transform(dg.FRAME, OPTICAL, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    q = buf.transform(p, dg.FRAME).point
    # direction of the image x axis in the robot frame, to turn the image angle into a robot-frame yaw
    p2 = PointStamped()
    p2.header.frame_id = OPTICAL
    p2.point.x, p2.point.y, p2.point.z = p.point.x + 0.1, p.point.y, p.point.z
    q2 = buf.transform(p2, dg.FRAME).point
    axis = math.atan2(q2.y - q.y, q2.x - q.x)
    yaw = axis + img_angle
    yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return q.x, q.y, q.z - CUBE_HALF, yaw


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ''
    if mode not in ('measure', 'descend'):
        print('Usage: vision_grasp.py measure | descend   (then: dock_grasp.py grab)')
        return
    rclpy.init()
    n = dg.DockGrasp()
    cx, cy, cz, yaw = detect(n, n.buf)
    print('detected cube (robot frame): x=%.3f y=%.3f centre z=%.3f, yaw %.1f deg' %
          (cx, cy, cz, math.degrees(yaw)))
    if mode == 'measure':
        return
    tcp, origin = n.tcp()
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = cx, cy, cz + tcp + dg.APPROACH
    qx, qy, qz, qw = dg.down_quat(yaw)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
    print('pre-grasp tool0 target: x=%.3f y=%.3f z=%.3f' % (cx, cy, pose.position.z))
    joints = n.ik(pose)
    if joints is None:
        print('Pre-grasp unreachable or in collision.')
        return
    if not n.move(joints):
        return
    cur = n.tf(dg.FRAME, dg.TIP)
    p2 = Pose()
    p2.position.x, p2.position.y, p2.position.z = cur.translation.x, cur.translation.y, cz + tcp
    p2.orientation = cur.rotation
    n.line_to(p2)
    print('Stopped above the cube, gripper open. Run: dock_grasp.py grab')


main()
