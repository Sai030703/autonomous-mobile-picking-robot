#!/usr/bin/env python3
"""usage: snap_camera.py [count] [interval_seconds]   -> saves PNGs to ~/camera_shots/"""
import sys, time, os, zlib, struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = '/wrist_camera/image'

def write_png(path, w, h, rgb):
    raw = b''.join(b'\x00' + rgb[y*w*3:(y+1)*w*3] for y in range(h))
    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))
    open(path, 'wb').write(png)

def to_rgb(msg):
    w, h, enc = msg.width, msg.height, msg.encoding
    ch = 4 if enc in ('rgba8', 'bgra8') else 3
    out = bytearray()
    for y in range(h):
        row = bytes(msg.data[y*msg.step:y*msg.step + w*ch])
        if ch == 4:
            r = bytearray(w*3)
            if enc == 'rgba8':
                r[0::3], r[1::3], r[2::3] = row[0::4], row[1::4], row[2::4]
            else:
                r[0::3], r[1::3], r[2::3] = row[2::4], row[1::4], row[0::4]
        elif enc == 'bgr8':
            r = bytearray(w*3)
            r[0::3], r[1::3], r[2::3] = row[2::3], row[1::3], row[0::3]
        else:
            r = row
        out += r
    return w, h, bytes(out)

def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rclpy.init()
    n = Node('snap_camera')
    box = {}
    n.create_subscription(Image, TOPIC, lambda m: box.update(msg=m), qos_profile_sensor_data)
    outdir = os.path.expanduser('~/camera_shots')
    os.makedirs(outdir, exist_ok=True)
    saved, nxt = 0, 0.0
    while saved < count and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if 'msg' in box and time.time() >= nxt:
            w, h, rgb = to_rgb(box['msg'])
            path = os.path.join(outdir, time.strftime('cam_%H%M%S_') + str(saved) + '.png')
            write_png(path, w, h, rgb)
            print('saved', path, '(%dx%d, %s)' % (w, h, box['msg'].encoding))
            saved += 1
            nxt = time.time() + interval
    n.destroy_node(); rclpy.shutdown()

main()
