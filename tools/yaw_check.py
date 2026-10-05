import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

def blob(rgb):
    return (rgb[:, :, 0] - np.maximum(rgb[:, :, 1], rgb[:, :, 2])) > 60

def min_area_angle(pts):
    p = pts - pts.mean(0)
    best = None
    for deg in np.arange(0, 90, 0.5):
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        r = p @ np.array([[c, -s], [s, c]])
        area = (r[:, 0].max() - r[:, 0].min()) * (r[:, 1].max() - r[:, 1].min())
        if best is None or area < best[0]:
            best = (area, deg)
    return best[1]

def corner_angle(pts):
    # the four extreme points along the diagonals approximate the square's corners
    c = pts.mean(0)
    d = pts - c
    proj = [d @ np.array(v) for v in ([1, 1], [1, -1], [-1, -1], [-1, 1])]
    cs = [pts[np.argmax(p)] for p in proj]
    ang = []
    for i in range(4):
        e = cs[(i + 1) % 4] - cs[i]
        ang.append(math.degrees(math.atan2(e[1], e[0])) % 90)
    a = np.array(ang)
    # circular mean over 90-degree symmetry
    z = np.exp(1j * np.radians(a * 4))
    return math.degrees(np.angle(z.mean())) / 4 % 90

rclpy.init()
n = Node('yaw_check')
rows = []
last = {}
n.create_subscription(Image, '/wrist_camera/image', lambda m: last.update(rgb=m), 1)
for i in range(10):
    last.clear()
    while 'rgb' not in last:
        rclpy.spin_once(n, timeout_sec=0.5)
    m = last['rgb']
    rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).astype(int)
    ys, xs = np.nonzero(blob(rgb))
    pts = np.stack([xs, ys], 1).astype(float)
    rows.append((min_area_angle(pts), corner_angle(pts), len(pts)))
    print('shot %2d: min-area %5.1f deg | corner-edges %5.1f deg | %d px' % ((i + 1,) + rows[-1]))
for k, name in ((0, 'min-area'), (1, 'corner-edges')):
    v = np.array([r[k] for r in rows])
    z = np.exp(1j * np.radians(v * 4))
    mean = math.degrees(np.angle(z.mean())) / 4 % 90
    spread = np.std((v - mean + 45) % 90 - 45)
    print('%-13s mean %.1f deg, spread (std) %.2f deg' % (name, mean, spread))
rclpy.shutdown()
