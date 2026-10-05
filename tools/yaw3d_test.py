import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
import tf2_geometry_msgs  # noqa: F401

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg      # noqa: E402
import vision_grasp as vg    # noqa: E402
import vision_pick as vp     # noqa: E402


def detect3d(n):
    got = {}
    n.create_subscription(Image, '/wrist_camera/image', lambda m: got.setdefault('rgb', m), 1)
    n.create_subscription(Image, '/wrist_camera/depth_image', lambda m: got.setdefault('depth', m), 1)
    t0 = time.time()
    while len(got) < 2 and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.2)
    r, d = got['rgb'], got['depth']
    rgb = np.frombuffer(r.data, np.uint8).reshape(r.height, r.width, 3).astype(int)
    dep = np.frombuffer(d.data, np.float32).reshape(d.height, d.width)
    mask = (rgb[:, :, 0] - np.maximum(rgb[:, :, 1], rgb[:, :, 2])) > 60
    ys, xs = np.nonzero(mask)
    z = dep[ys, xs]
    # keep the top face: depth within 8 mm of the nearest red pixels
    keep = z < np.percentile(z, 20) + 0.008
    xs, ys, z = xs[keep], ys[keep], z[keep]
    while not n.buf.can_transform(dg.FRAME, vg.OPTICAL, rclpy.time.Time()):
        rclpy.spin_once(n, timeout_sec=0.2)
    pts = []
    for u, v, zz in zip(xs[::6], ys[::6], z[::6]):
        p = PointStamped()
        p.header.frame_id = vg.OPTICAL
        p.point.x, p.point.y, p.point.z = (u - vg.CX) / vg.FX * zz, (v - vg.CY) / vg.FY * zz, float(zz)
        q = n.buf.transform(p, dg.FRAME).point
        pts.append((q.x, q.y, q.z))
    pts = np.array(pts)
    xy = pts[:, :2]
    c = xy.mean(0)
    best = None
    for deg in np.arange(0, 90, 0.5):
        a = math.radians(deg)
        rot = (xy - c) @ np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        area = (rot[:, 0].max() - rot[:, 0].min()) * (rot[:, 1].max() - rot[:, 1].min())
        if best is None or area < best[0]:
            best = (area, deg, rot[:, 0].max() - rot[:, 0].min(), rot[:, 1].max() - rot[:, 1].min())
    yaw = math.radians(best[1])
    yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return c[0], c[1], pts[:, 2].mean() - 0.03, yaw, best[2], best[3]


def main():
    rclpy.init()
    n = dg.DockGrasp()
    tcp, _ = n.tcp()
    rob = n.robot_pose()
    x0, y0 = dg.to_robot(dg.CUBE_W, rob)
    res = []
    for label, h in (('look (+0.10)', 0.10), ('lower (+0.05)', 0.05), ('look again (+0.10)', 0.10)):
        j = n.ik(vp.pose_at(x0, y0, vp.NOMINAL_CENTRE_Z + tcp + h, vp.wrap90(-rob[2])))
        if j is None or not n.move(j):
            print('move failed at', label)
            return
        time.sleep(1.5)
        old = vg.detect(n, n.buf)
        cx, cy, cz, yaw, w, l = detect3d(n)
        print('%-19s OLD yaw %6.1f deg x=%.3f | NEW yaw %6.1f deg x=%.3f y=%.3f z=%.3f | face %.0f x %.0f mm'
              % (label, math.degrees(old[3]), old[0], math.degrees(yaw), cx, cy, cz, 1000 * w, 1000 * l))
        res.append(math.degrees(yaw))
    print('NEW yaw spread across poses: %.1f deg' % (max(res) - min(res)))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
