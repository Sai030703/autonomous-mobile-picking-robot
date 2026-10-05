import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg      # noqa: E402
import vision_pick as vp     # noqa: E402

FINGER_DROP = 0.160     # tool0 height above the lowest finger point (from the STL meshes)
RELEASE_GAP = 0.02      # cube bottom this far above the tote floor when released
CUBE_BOTTOM_ABOVE_FINGERS = 0.005   # at grasp the lowest finger point was 5 mm above the cube bottom
TOTE_X, TOTE_Y = 0.25, 0.20
PLACE_YAW = 1.5708     # gripper turned 90 deg so the finger width lies along the tote's narrow side
CLEAR_Z = 0.85          # tool0 height for travelling with the cube (clear of the shelf top)


def main():
    rclpy.init()
    n = dg.DockGrasp()

    floor = n.tf(dg.FRAME, 'tote_floor').translation
    floor_top = floor.z + 0.01            # floor plate is 0.02 thick, link origin at its centre
    print('tote floor top (robot frame): z=%.3f at x=%.3f y=%.3f' % (floor_top, floor.x, floor.y))

    cur = n.tf(dg.FRAME, dg.TIP)
    print('tool0 now: x=%.3f y=%.3f z=%.3f' % (cur.translation.x, cur.translation.y, cur.translation.z))

    # 1. straight up to the clearing height
    p = Pose()
    p.position.x, p.position.y, p.position.z = cur.translation.x, cur.translation.y, CLEAR_Z
    p.orientation = cur.rotation
    if not n.line_to(p):
        print('lift failed'); return

    # 2. over the tote, same height, gripper still pointing down (yaw 0)
    j = n.ik(vp.pose_at(TOTE_X, TOTE_Y, CLEAR_Z, PLACE_YAW))
    if j is None:
        print('pose above the tote is unreachable'); return
    if not n.move(j):
        print('move above the tote failed'); return

    # 3. straight down until the cube bottom is RELEASE_GAP above the floor
    cur = n.tf(dg.FRAME, dg.TIP)
    release_z = floor_top + RELEASE_GAP + CUBE_BOTTOM_ABOVE_FINGERS + FINGER_DROP
    print('descending to tool0 z=%.3f' % release_z)
    p = Pose()
    p.position.x, p.position.y, p.position.z = cur.translation.x, cur.translation.y, release_z
    p.orientation = cur.rotation
    if not n.line_to(p):
        print('descent into the tote failed'); return

    # 4. release and retreat
    dg.grip(0.0)
    cur = n.tf(dg.FRAME, dg.TIP)
    p = Pose()
    p.position.x, p.position.y, p.position.z = cur.translation.x, cur.translation.y, CLEAR_Z
    p.orientation = cur.rotation
    n.line_to(p)
    print('PLACED. Check Gazebo: the cube should be inside the tote.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
