import os, sys, math, re, subprocess
import rclpy
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dock_grasp as dg
import skills

rclpy.init()
n = dg.DockGrasp()
print('UNLOAD RESULT:', skills.unload(n))
out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                     capture_output=True, text=True, timeout=30).stdout
i = out.find('name: "pick_item"')
c = out[i:i + 800]
g = lambda ax: float(re.search(r'position \\{[^}]*?\\b' + ax + r': ([-\\d.eE+]+)', c, re.S).group(1))
x, y, z = g('x'), g('y'), g('z')
print('cube world: x=%.3f y=%.3f z=%.3f' % (x, y, z))
print('ON THE TABLE:', -4.0 < x < -3.6 and abs(y) < 0.4 and abs(z - 0.48) < 0.02)
