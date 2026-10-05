import math
import os
import sys
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg      # noqa: E402
import skills                # noqa: E402
import vision_pick as vp     # noqa: E402


def cube_state():
    rx, ry, rz, ryaw = vp.gz('warehouse_amr')
    cx, cy, cz, _ = vp.gz('pick_item')
    dx, dy = cx - rx, cy - ry
    return (math.cos(ryaw) * dx + math.sin(ryaw) * dy, -math.sin(ryaw) * dx + math.cos(ryaw) * dy, cz - rz)


def main():
    rclpy.init()
    n = dg.DockGrasp()
    ok, why, info = skills.pick(n)
    x, y, z = cube_state()
    print('PICK -> ok=%s reason=%s | Gazebo truth: cube at x=%.3f y=%.3f z=%.3f (lifted: %s)' %
          (ok, why, x, y, z, z > 0.5))
    if not ok:
        return
    ok, why, info = skills.place(n)
    x, y, z = cube_state()
    inside = 0.13 < x < 0.37 and 0.12 < y < 0.28 and abs(z - 0.380) < 0.012
    print('PLACE -> ok=%s reason=%s | Gazebo truth: cube at x=%.3f y=%.3f z=%.3f (in the tote: %s)' %
          (ok, why, x, y, z, inside))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
