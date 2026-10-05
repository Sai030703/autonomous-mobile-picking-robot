import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

rclpy.init()
n = Node('grab_frame')
got = {}
n.create_subscription(Image, '/wrist_camera/image', lambda m: got.setdefault('rgb', m), 1)
n.create_subscription(Image, '/wrist_camera/depth_image', lambda m: got.setdefault('depth', m), 1)
while len(got) < 2:
    rclpy.spin_once(n, timeout_sec=0.5)
r, d = got['rgb'], got['depth']
print('rgb %dx%d %s | depth %dx%d %s' % (r.width, r.height, r.encoding, d.width, d.height, d.encoding))
rgb = np.frombuffer(r.data, np.uint8).reshape(r.height, r.width, 3).astype(int)
dep = np.frombuffer(d.data, np.float32).reshape(d.height, d.width)
open('/tmp/wrist.ppm', 'wb').write(b'P6 %d %d 255\n' % (r.width, r.height) + rgb.astype(np.uint8).tobytes())
ok = np.isfinite(dep)
if ok.any():
    print('valid depth: %d of %d, range %.2f - %.2f m' % (ok.sum(), dep.size, dep[ok].min(), dep[ok].max()))
redness = rgb[:, :, 0] - np.maximum(rgb[:, :, 1], rgb[:, :, 2])
mask = redness > 60
print('red pixels (redness > 60): %d' % mask.sum())
v, u = np.unravel_index(redness.argmax(), redness.shape)
print('reddest pixel u=%d v=%d rgb=%s depth=%.2f m redness=%d' % (u, v, rgb[v, u], dep[v, u], redness[v, u]))
if mask.any():
    ys, xs = np.nonzero(mask)
    print('red blob: centre u=%.0f v=%.0f | bbox u %d-%d, v %d-%d | median depth %.3f m'
          % (xs.mean(), ys.mean(), xs.min(), xs.max(), ys.min(), ys.max(), np.median(dep[mask])))
print('image centre pixel: rgb=%s depth=%.2f m' % (rgb[240, 320], dep[240, 320]))
print('saved /tmp/wrist.ppm')
rclpy.shutdown()
