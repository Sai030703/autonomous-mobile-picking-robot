import math, re, subprocess, time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

def truth():
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "warehouse_amr"')
    c = out[i:i + 800]
    g = lambda blk, ax: float(re.search(blk + r' \{[^}]*?\b' + ax + r': ([-\d.eE+]+)', c, re.S).group(1))
    q = {a: g('orientation', a) for a in 'xyzw'}
    yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
    return g('position', 'x'), g('position', 'y'), math.degrees(yaw)

rclpy.init()
n = Node('pose_logger')
buf = Buffer()
TransformListener(buf, n)
log = open('/home/sai/pose_log.txt', 'a')
t0 = time.time()
while True:
    end = time.time() + 1.5
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    try:
        tx, ty, tyaw = truth()
        t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        ax, ay = t.translation.x, t.translation.y
        d = math.hypot(ax - tx, ay - ty)
        line = '%6.0fs truth (%.2f, %.2f, %4.0f) amcl (%.2f, %.2f) error %.2f m %s' % (
            time.time() - t0, tx, ty, tyaw, ax, ay, d, 'LOST' if d > 0.3 else '')
    except Exception as e:
        line = 'no data: %s' % e
    print(line, flush=True)
    log.write(line + '\n')
    log.flush()
