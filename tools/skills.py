"""Pick and place as callable skills. Each returns (ok, reason, data)."""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg      # noqa: E402
import vision_pick as vp     # noqa: E402
import yaw3d_test as y3      # noqa: E402

HELD_SHORTFALL = 0.010   # rad; chosen from one held and one empty reading, to be re-checked on logged data
CLEAR_Z = 0.85
TOTE_X, TOTE_Y = 0.25, 0.20
PLACE_YAW = 1.5708
RELEASE_GAP = 0.02
FINGER_DROP = 0.160
CUBE_BOTTOM_ABOVE_FINGERS = 0.005


def knuckle_shortfall(n, commanded=0.3):
    from sensor_msgs.msg import JointState
    got = {}
    sub = n.create_subscription(JointState, '/joint_states', lambda m: got.setdefault('m', m), 10)
    t0 = time.time()
    while 'm' not in got and time.time() - t0 < 3:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_subscription(sub)
    p = dict(zip(got['m'].name, got['m'].position))
    left = p['rq_robotiq_85_left_knuckle_joint']
    right = -p['rq_robotiq_85_right_knuckle_joint']
    return ((commanded - left) + (commanded - right)) / 2.0, left, right


def go_ready(n):
    return n.move(vp.READY_JOINTS)


def pick(n):
    """Detect, grasp, lift. Returns (ok, reason, info)."""
    tcp, _ = n.tcp()
    rob = n.robot_pose()
    x0, y0 = dg.to_robot(dg.CUBE_W, rob)
    dg.grip(0.0)
    if not go_ready(n):
        return False, 'ready pose move failed', {}
    time.sleep(1.0)
    j = n.ik(vp.pose_at(x0, y0, vp.NOMINAL_CENTRE_Z + tcp + vp.APPROACH, vp.wrap90(-rob[2])))
    if j is None or not n.move(j):
        return False, 'look pose failed', {}
    time.sleep(1.0)
    try:
        cx, cy, cz, yaw, _w, _l = y3.detect3d(n)
    except Exception as e:           # no red pixels, etc.
        return False, 'cube not found: %s' % e, {}
    n.add_shelf(cx + vp.SHELF_OFFSET * math.cos(yaw), cy + vp.SHELF_OFFSET * math.sin(yaw),
                (cz - vp.CUBE_HALF) - vp.SHELF_HALF_H - vp.SHELF_TOP_MARGIN / 2, yaw,
                height=0.55 - vp.SHELF_TOP_MARGIN)
    grasp_z = cz - vp.CUBE_HALF + vp.CLEAR + vp.FINGER_DROP
    j = n.ik(vp.pose_at(cx, cy, grasp_z + vp.APPROACH, yaw))
    if j is None or not n.move(j):
        return False, 'pre-grasp failed', {}
    time.sleep(1.0)
    cur = n.tf(dg.FRAME, dg.TIP)
    p2 = Pose()
    p2.position.x, p2.position.y, p2.position.z = cur.translation.x, cur.translation.y, grasp_z
    p2.orientation = cur.rotation
    if not n.line_to(p2):
        return False, 'descent blocked', {}
    reached = n.tf(dg.FRAME, dg.TIP).translation.z
    dg.grip(0.2)
    dg.grip(0.3)
    short, l, r = knuckle_shortfall(n)
    held = short > HELD_SHORTFALL and abs(l - r) < 0.05
    info = {'detected': (cx, cy, cz, math.degrees(yaw)), 'grasp_z': reached,
            'knuckle_shortfall': short, 'knuckles': (l, r)}
    print('PICK: reached z=%.3f (target %.3f), knuckle shortfall %.3f rad (L %.3f R %.3f) -> %s' %
          (reached, grasp_z, short, l, r, 'HELD' if held else 'EMPTY'))
    e = n.tf(dg.FRAME, dg.TIP)
    p3 = Pose()
    p3.position.x, p3.position.y, p3.position.z = e.translation.x, e.translation.y, e.translation.z + 0.15
    p3.orientation = e.rotation
    n.line_to(p3)
    short2, _, _ = knuckle_shortfall(n)
    info['shortfall_after_lift'] = short2
    if not held:
        return False, 'grasp missed (knuckles closed fully)', info
    return True, 'ok', info


def place(n):
    """Carry the held cube to the tote and release it. Returns (ok, reason, info)."""
    floor = n.tf(dg.FRAME, 'tote_floor').translation
    floor_top = floor.z + 0.01
    cur = n.tf(dg.FRAME, dg.TIP)
    p = Pose()
    p.position.x, p.position.y, p.position.z = cur.translation.x, cur.translation.y, CLEAR_Z
    p.orientation = cur.rotation
    if not n.line_to(p):
        return False, 'lift before carry failed', {}
    j = n.ik(vp.pose_at(TOTE_X, TOTE_Y, CLEAR_Z, PLACE_YAW))
    if j is None:
        return False, 'pose above the tote unreachable', {}
    if not n.move(j):
        return False, 'move above the tote failed', {}
    cur = n.tf(dg.FRAME, dg.TIP)
    release_z = floor_top + RELEASE_GAP + CUBE_BOTTOM_ABOVE_FINGERS + FINGER_DROP
    p = Pose()
    p.position.x, p.position.y, p.position.z = cur.translation.x, cur.translation.y, release_z
    p.orientation = cur.rotation
    if not n.line_to(p):
        return False, 'descent into the tote failed', {}
    dg.grip(0.0)
    cur = n.tf(dg.FRAME, dg.TIP)
    p = Pose()
    p.position.x, p.position.y, p.position.z = cur.translation.x, cur.translation.y, CLEAR_Z
    p.orientation = cur.rotation
    if not n.line_to(p):
        return False, 'retreat from the tote blocked', {}
    return True, 'ok', {}
