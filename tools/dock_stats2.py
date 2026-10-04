import math
import statistics
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import dock_stats as ds   # reuses Stats, gz_robot, to_robot, wrap (its main() is guarded below)

SITES = {'A': (-0.8, -3.0, 0.0), 'B': (-3.8, 0.25, 0.0)}
AHEAD = 0.72   # the "cube" is this far ahead of each dock pose, as at site A


def main():
    runs = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    rclpy.init()
    node = ds.Stats()
    res = {k: [] for k in SITES}
    for i in range(runs):
        name = 'AB'[i % 2]
        x, y, yaw = SITES[name]
        st = node.nav(x, y, yaw)
        time.sleep(3.0)
        a, g = node.amcl(), ds.gz_robot()
        cube = (x + AHEAD * math.cos(yaw), y + AHEAD * math.sin(yaw))
        ca, cg = ds.to_robot(cube, a), ds.to_robot(cube, g)
        fwd, side = 1000 * (ca[0] - cg[0]), 1000 * (ca[1] - cg[1])
        yerr = math.degrees(ds.wrap(a[2] - g[2]))
        derr = 1000 * math.hypot(g[0] - x, g[1] - y)
        ok = st == GoalStatus.STATUS_SUCCEEDED
        print('run %d site %s: nav %s | real dock error %.0f mm | AMCL cube error: along %+.1f mm, '
              'across %+.1f mm | yaw %+.1f deg | AMCL xy minus real: dx %+.0f dy %+.0f mm'
              % (i + 1, name, 'ok' if ok else 'FAIL', derr, fwd, side, yerr,
                 1000 * (a[0] - g[0]), 1000 * (a[1] - g[1])))
        if ok:
            res[name].append((fwd, side, yerr))
    for k, rows in res.items():
        if len(rows) >= 2:
            for j, label in enumerate(['along (mm)', 'across (mm)', 'yaw (deg)']):
                v = [r[j] for r in rows]
                print('site %s %-12s mean %+.1f std %.1f (n=%d)' % (k, label, statistics.mean(v), statistics.pstdev(v), len(v)))
    rclpy.shutdown()


main()
