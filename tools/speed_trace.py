import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    gx = float(sys.argv[1]) if len(sys.argv) > 1 else -3.0
    gy = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    gyaw = float(sys.argv[3]) if len(sys.argv) > 3 else 3.14159
    rclpy.init()
    n = Node('speed_trace')
    samples = []
    n.create_subscription(Odometry, '/diff_drive_controller/odom',
                          lambda m: samples.append((time.time(), m.twist.twist.linear.x, m.pose.pose.position.x,
                                                    m.pose.pose.position.y)), 20)
    cli = ActionClient(n, NavigateToPose, '/navigate_to_pose')
    cli.wait_for_server(timeout_sec=10.0)
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    t0 = time.time()
    fut = cli.send_goal_async(g)
    while not fut.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    h = fut.result()
    if not h.accepted:
        print('goal rejected')
        return
    rf = h.get_result_async()
    while not rf.done() and time.time() - t0 < 120:
        rclpy.spin_once(n, timeout_sec=0.05)
    res = rf.result()
    status = res.status if res else None
    print('trip: status %s (4 = succeeded, 6 = aborted) in %.1f s' % (status, time.time() - t0))
    if res:
        print('error: code %d %s' % (res.result.error_code, res.result.error_msg))
    v = [(t - t0, x, px, py) for t, x, px, py in samples if t >= t0]
    if not v:
        print('no odometry samples')
        return
    print('max |vx| %.2f m/s | odom distance from start to end %.2f m' %
          (max(abs(x) for _, x, _, _ in v), math.hypot(v[-1][2] - v[0][2], v[-1][3] - v[0][3])))
    k = 0
    while k < v[-1][0]:
        seg = [(x, px, py) for t, x, px, py in v if k <= t < k + 2]
        if seg:
            moved = math.hypot(seg[-1][1] - seg[0][1], seg[-1][2] - seg[0][2])
            print('  t=%4.0f-%4.0f s  mean |vx| %.2f m/s   moved %.2f m' %
                  (k, k + 2, sum(abs(s[0]) for s in seg) / len(seg), moved))
        k += 2
    rclpy.shutdown()


main()
