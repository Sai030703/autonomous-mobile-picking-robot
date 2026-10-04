import sys
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# (joint, multiplier) copied from the URDF mimic table
JOINTS = [
    ('rq_robotiq_85_left_knuckle_joint', 1.0),
    ('rq_robotiq_85_right_knuckle_joint', -1.0),
    ('rq_robotiq_85_left_inner_knuckle_joint', 1.0),
    ('rq_robotiq_85_right_inner_knuckle_joint', -1.0),
    ('rq_robotiq_85_left_finger_tip_joint', -1.0),
    ('rq_robotiq_85_right_finger_tip_joint', 1.0),
]

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 gripper.py <0.0 open .. 0.7 closed>')
        return
    q = max(0.0, min(0.8, float(sys.argv[1])))
    rclpy.init()
    node = Node('gripper_cmd')
    pub = node.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
    msg = JointTrajectory()
    msg.joint_names = [j for j, _ in JOINTS]
    pt = JointTrajectoryPoint()
    pt.positions = [q * m for _, m in JOINTS]
    pt.time_from_start = Duration(sec=2)
    msg.points.append(pt)
    while pub.get_subscription_count() == 0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print(f'sent opening {q:.2f}')
    rclpy.shutdown()

main()
