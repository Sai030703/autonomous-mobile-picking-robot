import numpy as np

PGM = '/home/sai/ros2_ws/src/warehouse_amr/maps/warehouse_map.pgm'
RES, OX, OY = 0.05, -7.426, -34.015   # from warehouse_map.yaml

with open(PGM, 'rb') as f:
    tokens = []
    while len(tokens) < 4:
        line = f.readline()
        if line.startswith(b'#'):
            continue
        tokens += line.split()
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
img = img[::-1]                  # row 0 = lowest y, like the map frame
free = img >= 250                # trinary map: 254 free, 205 unknown, 0 occupied
occ = img <= 50

def cells(x0, x1, y0, y1):
    i0, i1 = int((x0 - OX) / RES), int((x1 - OX) / RES)
    j0, j1 = int((y0 - OY) / RES), int((y1 - OY) / RES)
    if i0 < 0 or j0 < 0 or i1 >= w or j1 >= h:
        return None
    return (slice(j0, j1 + 1), slice(i0, i1 + 1))

def all_free(x0, x1, y0, y1):
    s = cells(x0, x1, y0, y1)
    return s is not None and bool(free[s].all())

# 1) Does the map frame match the Gazebo world? Check the AWS ShelfD upright positions.
print('Alignment check (occupied cells near known shelf uprights):')
ok = 0
for cx, cy in [(4.73156, -1.242668), (4.73156, -3.038551), (4.73156, -6.750542)]:
    for dx in (-1.931, 0.014, 1.931):
        s = cells(cx + dx - 0.12, cx + dx + 0.12, cy - 0.35, cy + 0.35)
        n = int(occ[s].sum()) if s is not None else -1
        ok += n > 0
        print('  upright at (%.2f, %.2f): %d occupied cells' % (cx + dx, cy, n))
print('  -> %d of 9 uprights found. Most should be found if map == world.' % ok)

# 2) Shelf centres 3-6 m away, shelf front facing -x, dock pose 0.8 m in front of the centre.
hits = []
for sx in np.arange(-6.0, 6.0, 0.25):
    for sy in np.arange(-10.0, 9.0, 0.25):
        d = np.hypot(sx, sy)
        if not 3.0 <= d <= 6.0:
            continue
        shelf_and_lane = all_free(sx - 2.3, sx + 0.6, sy - 0.9, sy + 0.9)
        around_dock = all_free(sx - 0.8 - 0.7, sx - 0.8 + 0.7, sy - 0.7, sy + 0.7)
        if shelf_and_lane and around_dock:
            hits.append((d, sx, sy, np.degrees(np.arctan2(sy, sx))))

print('\n%d candidates 3-6 m from spawn. Closest per compass direction:' % len(hits))
best = {}
for d, sx, sy, b in sorted(hits):
    best.setdefault(int(((b + 360) % 360) // 45), (d, sx, sy, b))
for d, sx, sy, b in sorted(best.values(), key=lambda t: t[3]):
    print('  shelf centre (%.2f, %.2f)  dock pose (%.2f, %.2f) yaw 0   %.1f m away, bearing %.0f deg'
          % (sx, sy, sx - 0.8, sy, d, b))
