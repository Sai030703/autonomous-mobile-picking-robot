import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg      # noqa: E402
import skills                # noqa: E402

SHELF_DOCK = (-0.8, -3.0, 0.0)
STATION = (-3.0, 0.0, 3.14159)     # placeholder: free area near spawn
MAX_DOCK, MAX_PICK, MAX_PLACE = 3, 3, 2


class Orchestrator(dg.DockGrasp):
    def __init__(self):
        super().__init__()
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.pub = self.create_publisher(String, '/amr/status', 10)
        self.sub = self.create_subscription(String, '/warehouse/order', self.on_order, 10)
        self.order = None
        self.state = 'IDLE'
        self.errors = []
        self.actions = []
        self.create_timer(0.5, self.publish)

    def publish(self):
        try:
            t = self.buf.lookup_transform('map', dg.FRAME, rclpy.time.Time()).transform
            pose = (t.translation.x, t.translation.y, dg.yaw_of(t.rotation))
            pos = {'x': round(pose[0], 3), 'y': round(pose[1], 3), 'theta': round(pose[2], 3)}
        except Exception:
            pos = None
        msg = {'orderId': self.order['orderId'] if self.order else '', 'state': self.state,
               'agvPosition': pos, 'actionStates': self.actions[-6:], 'errors': self.errors}
        self.pub.publish(String(data=json.dumps(msg)))

    def on_order(self, m):
        if self.state in ('IDLE', 'DONE', 'FAILED'):
            self.order = json.loads(m.data)
            self.order_pending = True

    def set_state(self, s):
        self.state = s
        print('STATE ->', s)
        self.publish()

    def log(self, name, ok, reason=''):
        self.actions.append({'action': name, 'ok': ok, 'reason': reason})
        print('  %s: %s %s' % (name, 'ok' if ok else 'FAILED', reason))
        self.publish()

    def goto(self, pose):
        if not self.nav.wait_for_server(timeout_sec=10.0):
            return False, 'navigation server unavailable'
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = pose[0], pose[1]
        g.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        g.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        f = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=15.0)
        h = f.result()
        if h is None or not h.accepted:
            return False, 'goal rejected'
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=180.0)
        if r.result() is None:
            return False, 'navigation timed out'
        ok = r.result().status == GoalStatus.STATUS_SUCCEEDED
        res = r.result().result
        detail = 'code %d %s' % (res.error_code, res.error_msg)
        return ok, '' if ok else 'navigation status %d (%s)' % (r.result().status, detail)

    def goto_checked(self, pose):
        ok, why = self.goto(pose)
        if not ok:
            return ok, why
        ok2, why2 = self.arrived(pose)
        return ok2, ('arrived, ' if ok2 else 'navigation said success but ') + why2

    def arrived(self, pose, pos_tol=0.25, yaw_tol_deg=20.0):
        try:
            x, y, yaw = self.robot_pose()
        except Exception as e:
            return False, 'pose unavailable: %s' % e
        dpos = math.hypot(x - pose[0], y - pose[1])
        dyaw = abs(math.degrees((yaw - pose[2] + math.pi) % (2 * math.pi) - math.pi))
        ok = dpos <= pos_tol and dyaw <= yaw_tol_deg
        return ok, 'off by %.2f m and %.1f deg' % (dpos, dyaw)

    def redock_then_pick(self):
        x, y, _ = self.robot_pose()
        if math.hypot(x + 0.8, y + 3.0) > 0.12:
            self.goto(SHELF_DOCK)
        return skills.pick(self)[:2]

    def retry(self, name, fn, tries):
        for i in range(1, tries + 1):
            ok, why = fn()
            self.log('%s (attempt %d/%d)' % (name, i, tries), ok, why)
            if ok:
                return True
            skills.go_ready(self)
        self.errors.append({'errorType': name, 'errorDescription': 'failed after %d attempts: %s' % (tries, why)})
        return False

    def run_order(self):
        qty = int(self.order.get('quantity', 1))
        print('ORDER', self.order)
        self.actions, self.errors = [], []
        self.set_state('NAVIGATING_TO_SHELF')
        if not self.retry('dock at shelf', lambda: self.goto(SHELF_DOCK), MAX_DOCK):
            return self.set_state('FAILED')
        for k in range(qty):
            self.set_state('PICKING')
            if not self.retry('pick %d/%d' % (k + 1, qty), lambda: self.redock_then_pick(), MAX_PICK):
                return self.set_state('FAILED')
            self.set_state('PLACING')
            if not self.retry('place %d/%d' % (k + 1, qty), lambda: skills.place(self)[:2], MAX_PLACE):
                return self.set_state('FAILED')
        skills.go_ready(self)
        self.set_state('NAVIGATING_TO_STATION')
        if not self.retry('deliver to station', lambda: self.goto_checked(STATION), MAX_DOCK):
            return self.set_state('FAILED')
        self.set_state('DONE')


def main():
    rclpy.init()
    o = Orchestrator()
    o.order_pending = False
    print('Orchestrator ready. Publish an order on /warehouse/order.')
    while rclpy.ok():
        rclpy.spin_once(o, timeout_sec=0.2)
        if o.order_pending:
            o.order_pending = False
            o.run_order()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
