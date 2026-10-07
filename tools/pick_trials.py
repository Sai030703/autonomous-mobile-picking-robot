import csv, math, os, subprocess, sys, time
import rclpy
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg      # noqa: E402
import skills                # noqa: E402
import orchestrator as orch  # noqa: E402
import vision_pick as vp     # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 8
STAGES = [(-1.6, -2.7, 0.0), (-1.6, -3.3, math.pi), (-1.6, -3.0, math.pi / 2)]


def respawn():
    subprocess.run(['gz', 'service', '-s', '/world/default/remove', '--reqtype', 'gz.msgs.Entity',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req',
                    'name: "pick_item" type: MODEL'], capture_output=True)
    time.sleep(1)
    subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-file',
                    os.path.join(HERE, '..', 'worlds', 'pick_item.sdf'), '-name', 'pick_item',
                    '-x', '-0.08', '-y', '-3.0', '-z', '0.582'], capture_output=True)
    time.sleep(3)


def main():
    rclpy.init()
    n = orch.Orchestrator()
    path = os.path.expanduser('~/pick_trials.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['trial', 'dock_ok', 'real_x', 'real_y', 'real_yaw_deg', 'det_y', 'det_x',
                    'reached_z', 'ok', 'reason'])
        for i in range(N):
            skills.go_ready(n)
            n.goto(STAGES[i % 3])
            dock_ok, _ = n.goto(orch.SHELF_DOCK)
            time.sleep(2)
            respawn()
            skills.LAST.clear()
            ok, why, info = skills.pick(n)
            rx, ry, rz, ryaw = vp.gz('warehouse_amr')
            L = skills.LAST
            row = [i + 1, dock_ok, round(rx, 3), round(ry, 3), round(math.degrees(ryaw), 1),
                   round(L.get('cy', float('nan')), 3), round(L.get('cx', float('nan')), 3),
                   round(L.get('reached', float('nan')), 3), ok, why]
            w.writerow(row); f.flush()
            print('TRIAL', row)
            dg.grip(0.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
