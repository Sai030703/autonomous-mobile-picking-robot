import itertools
import sys
import xml.etree.ElementTree as ET

urdf_path, out_path = sys.argv[1], sys.argv[2]
pairs = {}

def add(a, b, reason):
    pairs.setdefault(tuple(sorted((a, b))), reason)

# UR's own adjacent / never-colliding pairs, with our "ur_" prefix
for a, b in [
    ("base_link", "base_link_inertia"), ("base_link_inertia", "shoulder_link"),
    ("tool0", "wrist_1_link"), ("tool0", "wrist_2_link"), ("tool0", "wrist_3_link"),
    ("forearm_link", "upper_arm_link"), ("forearm_link", "wrist_1_link"),
    ("shoulder_link", "upper_arm_link"), ("wrist_1_link", "wrist_2_link"),
    ("wrist_1_link", "wrist_3_link"), ("wrist_2_link", "wrist_3_link"),
]:
    add("ur_" + a, "ur_" + b, "Adjacent")

# Mobile base: wheels and casters overlap the chassis box by design
base = ["base_footprint", "base_link", "left_wheel_link", "right_wheel_link",
        "front_caster_link", "rear_caster_link", "lidar_link", "imu_link"]
for a, b in itertools.combinations(base, 2):
    add(a, b, "User")
# Arm base sits on the deck
for l in ["ur_base_link", "ur_base_link_inertia", "ur_shoulder_link"]:
    add("base_link", l, "Adjacent")

# Gripper: the finger linkage overlaps itself by design
gripper = [
    "rq_ur_to_robotiq_link", "rq_gripper_mount_link", "rq_robotiq_85_base_link",
    "rq_robotiq_85_left_knuckle_link", "rq_robotiq_85_right_knuckle_link",
    "rq_robotiq_85_left_finger_link", "rq_robotiq_85_right_finger_link",
    "rq_robotiq_85_left_inner_knuckle_link", "rq_robotiq_85_right_inner_knuckle_link",
    "rq_robotiq_85_left_finger_tip_link", "rq_robotiq_85_right_finger_tip_link",
]
for a, b in itertools.combinations(gripper, 2):
    add(a, b, "User")
for g in gripper:
    for w in ["ur_tool0", "ur_wrist_3_link"]:
        add(g, w, "User")

# Every link named here must exist in the URDF
urdf_links = {l.get("name") for l in ET.parse(urdf_path).getroot().iter("link")}
missing = sorted({n for p in pairs for n in p} - urdf_links)
if missing:
    print("MISSING LINKS (not in URDF):", missing)
    sys.exit(1)

joints = ["ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
          "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"]
states = {"ready": [0, -1.57, 1.57, -1.57, -1.57, 0],
          "up": [0, -1.57, 0, -1.57, 0, 0]}

out = ['<?xml version="1.0" encoding="UTF-8"?>', '<robot name="warehouse_amr">',
       '  <group name="ur_manipulator">',
       '    <chain base_link="ur_base_link" tip_link="ur_tool0"/>',
       '  </group>']
for name, vals in states.items():
    out.append('  <group_state name="%s" group="ur_manipulator">' % name)
    for j, v in zip(joints, vals):
        out.append('    <joint name="%s" value="%s"/>' % (j, v))
    out.append('  </group_state>')
for (a, b), reason in sorted(pairs.items()):
    out.append('  <disable_collisions link1="%s" link2="%s" reason="%s"/>' % (a, b, reason))
out.append('</robot>')
open(out_path, "w").write("\n".join(out) + "\n")
print("wrote %s with %d disabled pairs, all link names verified" % (out_path, len(pairs)))
