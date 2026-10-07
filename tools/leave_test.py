import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import dock_stats as ds   # noqa: E402

DOCK = (-0.8, -3.0, 0.0)
STATION = (-3.0, 0.0, 3.14159)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'direct'
    rclpy.init()
    n = ds.Stats()
    pub = n.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
    print('mode:', mode, '- going to the shelf dock')
    print('dock status', n.nav(*DOCK), '(4 = succeeded, 6 = aborted)')
    time.sleep(2.0)
    if mode == 'backup':
        t0 = time.time()
        while time.time() - t0 < 3.0:
            m = TwistStamped()
            m.header.frame_id = 'base_link'
            m.twist.linear.x = -0.2
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.05)
        time.sleep(1.5)
    p0 = n.amcl()
    print('start of the station trip: x=%.2f y=%.2f yaw=%.0f deg' % (p0[0], p0[1], math.degrees(p0[2])))
    t0 = time.time()
    s = n.nav(*STATION)
    p1 = n.amcl()
    print('STATION TRIP: status %s in %.1f s, ended at x=%.2f y=%.2f' % (s, time.time() - t0, p1[0], p1[1]))
    rclpy.shutdown()


main()
