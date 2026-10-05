import math
import os
import sys
import time

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg      # noqa: E402
import vision_grasp as vg    # noqa: E402
import vision_pick as vp     # noqa: E402


def main():
    rclpy.init()
    n = dg.DockGrasp()
    tcp, _ = n.tcp()
    rob = n.robot_pose()
    x0, y0 = dg.to_robot(dg.CUBE_W, rob)
    results = []
    for label, height in (('look pose (+0.10 m)', 0.10), ('lower pose (+0.05 m)', 0.05), ('look pose again (+0.10 m)', 0.10)):
        j = n.ik(vp.pose_at(x0, y0, vp.NOMINAL_CENTRE_Z + tcp + height, vp.wrap90(-rob[2])))
        if j is None or not n.move(j):
            print('move failed at', label)
            return
        time.sleep(1.5)
        cx, cy, cz, yaw = vg.detect(n, n.buf)
        tool = n.tf(dg.FRAME, dg.TIP)
        print('%-26s tool0 z=%.3f | cube x=%.3f y=%.3f z=%.3f | robot-frame yaw %.1f deg'
              % (label, tool.translation.z, cx, cy, cz, math.degrees(yaw)))
        results.append(math.degrees(yaw))
    print('yaw differs across poses by up to %.1f deg (the cube did not move)' % (max(results) - min(results)))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
