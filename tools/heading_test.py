import math
import sys
import time

import rclpy

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import dock_stats as ds   # noqa: E402


def main():
    rclpy.init()
    n = ds.Stats()
    for label, goal in (('shelf dock, heading 0 deg', (-0.8, -3.0, 0.0)),
                        ('station, heading 180 deg', (-3.0, 0.0, 3.14159)),
                        ('station, heading 90 deg', (-3.0, 0.0, 1.5708)),
                        ('shelf dock, heading 0 deg', (-0.8, -3.0, 0.0)),
                        ('station, heading -90 deg', (-3.0, 0.0, -1.5708))):
        t0 = time.time()
        s = n.nav(*goal)
        time.sleep(1.5)
        p = n.amcl()
        print('%-28s status %s in %5.1f s, ended x=%.2f y=%.2f yaw=%4.0f deg' %
              (label, s, time.time() - t0, p[0], p[1], math.degrees(p[2])))
    rclpy.shutdown()


main()
