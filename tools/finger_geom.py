import re
import struct
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node

LINKS = ['rq_robotiq_85_left_finger_tip_link', 'rq_robotiq_85_right_finger_tip_link',
         'rq_robotiq_85_left_finger_link', 'rq_robotiq_85_right_finger_link']
SHELF_TOP = 0.516      # robot frame: cube bottom / shelf top (world 0.55 minus robot z 0.034)
CLEARANCE = 0.005


def load_stl(path):
    data = open(path, 'rb').read()
    if data[:5] == b'solid' and b'facet' in data[:400]:
        v = re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data)
        return np.array(v, dtype=float)
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    arr = np.frombuffer(data, dtype=rec, count=n, offset=84)
    return arr['v'].reshape(-1, 3).astype(float)


def rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    root = ET.parse('/tmp/amr_cam.urdf').getroot()
    meshes = {}
    for l in root.iter('link'):
        if l.get('name') in LINKS:
            m = l.find('collision/geometry/mesh')
            if m is not None:
                meshes[l.get('name')] = m.get('filename').replace('file://', '')
    rclpy.init()
    n = Node('finger_geom')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    overall = None
    for name in LINKS:
        if name not in meshes:
            print(name, ': no collision mesh found in URDF')
            continue
        while not buf.can_transform('base_footprint', name, rclpy.time.Time()):
            rclpy.spin_once(n, timeout_sec=0.1)
        t = buf.lookup_transform('base_footprint', name, rclpy.time.Time()).transform
        pts = load_stl(meshes[name]) @ rot(t.rotation).T + np.array([t.translation.x, t.translation.y, t.translation.z])
        print('%-38s %5d verts | z %.3f..%.3f | y %.3f..%.3f | x %.3f..%.3f' %
              (name, len(pts), pts[:, 2].min(), pts[:, 2].max(), pts[:, 1].min(), pts[:, 1].max(),
               pts[:, 0].min(), pts[:, 0].max()))
        overall = pts[:, 2].min() if overall is None else min(overall, pts[:, 2].min())
    tool = buf.lookup_transform('base_footprint', 'ur_tool0', rclpy.time.Time()).transform.translation.z
    print('\nlowest finger point now: z = %.3f | tool0 z = %.3f' % (overall, tool))
    print('cube (robot frame): bottom/shelf top %.3f, centre 0.546, top 0.576' % SHELF_TOP)
    target = tool - (overall - (SHELF_TOP + CLEARANCE))
    print('tool0 z that puts the lowest finger point %.0f mm above the shelf top: %.3f' % (1000 * CLEARANCE, target))
    rclpy.shutdown()


main()
