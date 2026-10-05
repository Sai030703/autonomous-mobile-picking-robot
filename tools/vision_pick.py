import csv
import math
import os
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Pose

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg      # noqa: E402
import vision_grasp as vg    # noqa: E402
import yaw3d_test as y3      # noqa: E402

FINGER_DROP = 0.160       # tool0 z minus lowest finger point (measured from the STL meshes)
CLEAR = 0.005
APPROACH = 0.10
CUBE_HALF = 0.03
NOMINAL_CENTRE_Z = 0.546  # design value in the robot frame, used only to choose the look pose
SHELF_OFFSET = 0.08       # shelf centre is this far beyond the cube, along the shelf's own x axis
SHELF_HALF_H = 0.275
SHELF_TOP_MARGIN = 0.03   # box top is this far below the real shelf top, so the planner lets the fingers reach the cube's base


def gz(name):
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "%s"' % name)
    if i < 0:
        raise RuntimeError('%s not in Gazebo pose list' % name)
    chunk = out[i:i + 800]
    g = lambda blk, ax: float(re.search(blk + r' \{[^}]*?\b' + ax + r': ([-\d.eE+]+)', chunk, re.S).group(1))
    q = {a: g('orientation', a) for a in 'xyzw'}
    yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
    return g('position', 'x'), g('position', 'y'), g('position', 'z'), yaw


def wrap90(a):
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


def pose_at(x, y, z, yaw):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = dg.down_quat(yaw)
    return p


READY_JOINTS = {'ur_shoulder_pan_joint': 0.0, 'ur_shoulder_lift_joint': -1.57, 'ur_elbow_joint': 1.57,
                'ur_wrist_1_joint': -1.57, 'ur_wrist_2_joint': -1.57, 'ur_wrist_3_joint': 0.0}


def main():
    rclpy.init()
    n = dg.DockGrasp()
    tcp, _ = n.tcp()

    # 1. coarse look pose from AMCL's nominal cube position (no ground truth)
    rob = n.robot_pose()
    x0, y0 = dg.to_robot(dg.CUBE_W, rob)
    dg.grip(0.0)
    n.move(READY_JOINTS)   # known-good intermediate pose, so the look move never starts from the upright pose
    time.sleep(1.0)
    j = n.ik(pose_at(x0, y0, NOMINAL_CENTRE_Z + tcp + APPROACH, wrap90(-rob[2])))
    if j is None or not n.move(j):
        print('look pose failed'); return
    time.sleep(1.0)

    # 2. detect; score against Gazebo truth (evaluation only)
    cx, cy, cz, yaw, _w, _l = y3.detect3d(n)
    print('DETECTED cube: x=%.3f y=%.3f centre z=%.3f yaw %.1f deg' % (cx, cy, cz, math.degrees(yaw)))
    rx, ry, rz, ryaw = gz('warehouse_amr')
    wx, wy, wz, wyaw = gz('pick_item')
    c, s = math.cos(ryaw), math.sin(ryaw)
    tx, ty = c * (wx - rx) + s * (wy - ry), -s * (wx - rx) + c * (wy - ry)
    err = (1000 * (cx - tx), 1000 * (cy - ty), 1000 * (cz - (wz - rz)),
           math.degrees(wrap90(yaw - wrap90(wyaw - ryaw))))

    # 3. planning-scene shelf placed relative to the DETECTED cube
    n.add_shelf(cx + SHELF_OFFSET * math.cos(yaw), cy + SHELF_OFFSET * math.sin(yaw),
                (cz - CUBE_HALF) - SHELF_HALF_H - SHELF_TOP_MARGIN / 2, yaw, height=0.55 - SHELF_TOP_MARGIN)

    # 4. pre-grasp, then a repeatability check
    grasp_z = cz - CUBE_HALF + CLEAR + FINGER_DROP
    j = n.ik(pose_at(cx, cy, grasp_z + APPROACH, yaw))
    if j is None or not n.move(j):
        print('pre-grasp failed'); return
    time.sleep(1.0)
    # DIAG: what the planner sees right before the descent
    try:
        from moveit_msgs.srv import GetPlanningScene
        from sensor_msgs.msg import JointState
        gc = n.create_client(GetPlanningScene, '/get_planning_scene')
        gc.wait_for_service(timeout_sec=5)
        rq = GetPlanningScene.Request()
        rq.components.components = rq.components.WORLD_OBJECT_GEOMETRY
        fu = gc.call_async(rq)
        rclpy.spin_until_future_complete(n, fu, timeout_sec=5)
        for o in fu.result().scene.world.collision_objects:
            for pr, ps in zip(o.primitives, o.primitive_poses):
                print('DIAG box %s frame=%s centre x=%.3f y=%.3f z=%.3f size=%s top z=%.3f' % (
                    o.id, o.header.frame_id, ps.position.x, ps.position.y, ps.position.z,
                    [round(d, 3) for d in pr.dimensions], ps.position.z + pr.dimensions[2] / 2))
        jj = {}
        n.create_subscription(JointState, '/joint_states', lambda m: jj.update(m=m), 10)
        t0 = time.time()
        while 'm' not in jj and time.time() - t0 < 3:
            rclpy.spin_once(n, timeout_sec=0.1)
        print('DIAG pre-grasp joints:', {k: round(v, 3) for k, v in zip(jj['m'].name, jj['m'].position) if k.startswith('ur_')})
    except Exception as e:
        print('DIAG failed:', e)
    # 5. straight-line descent over the current tool position
    cur = n.tf(dg.FRAME, dg.TIP)
    p2 = Pose()
    p2.position.x, p2.position.y, p2.position.z = cur.translation.x, cur.translation.y, grasp_z
    p2.orientation = cur.rotation
    if not n.line_to(p2):
        print('descent failed, stopping with the gripper open'); return
    e = n.tf(dg.FRAME, dg.TIP)
    print('AT GRASP: tool0 x=%.3f y=%.3f z=%.3f (target z %.3f)' %
          (e.translation.x, e.translation.y, e.translation.z, grasp_z))

    # 6. close and lift
    dg.grip(0.2)
    dg.grip(0.3)
    e = n.tf(dg.FRAME, dg.TIP)
    p3 = Pose()
    p3.position.x, p3.position.y, p3.position.z = e.translation.x, e.translation.y, e.translation.z + 0.15
    p3.orientation = e.rotation
    n.line_to(p3)
    time.sleep(2.0)
    lifted = gz('pick_item')[2] > 0.65
    print('RESULT: error x %+.1f y %+.1f z %+.1f mm, yaw %+.1f deg | cube world z after lift %.3f | LIFTED: %s' %
          (err + (gz('pick_item')[2], 'YES' if lifted else 'NO')))
    path = os.path.expanduser('~/vision_trials.csv')
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['err_x_mm', 'err_y_mm', 'err_z_mm', 'err_yaw_deg', 'lifted'])
        w.writerow([round(v, 1) for v in err] + [lifted])
    rclpy.shutdown()
    sys.exit(0 if lifted else 1)


if __name__ == '__main__':
    main()
