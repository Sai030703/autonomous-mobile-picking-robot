import math, os, sys, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pose_logger_truth import truth
rclpy.init()
n = rclpy.create_node('reseed_amcl')
pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
x, y, yaw = truth()
m = PoseWithCovarianceStamped()
m.header.frame_id = 'map'
m.pose.pose.position.x, m.pose.pose.position.y = x, y
m.pose.pose.orientation.z = math.sin(math.radians(yaw) / 2)
m.pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
m.pose.covariance[0] = m.pose.covariance[7] = 0.25
m.pose.covariance[35] = 0.0685
for _ in range(5):
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.2)
    time.sleep(0.3)
print('reseeded AMCL to x=%.2f y=%.2f yaw=%.0f' % (x, y, yaw))
