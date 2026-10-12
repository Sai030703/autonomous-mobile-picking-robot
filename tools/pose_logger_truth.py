import math, re, subprocess
def truth():
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "warehouse_amr"')
    c = out[i:i + 800]
    g = lambda blk, ax: float(re.search(blk + r' \{[^}]*?\b' + ax + r': ([-\d.eE+]+)', c, re.S).group(1))
    q = {a: g('orientation', a) for a in 'xyzw'}
    yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
    return g('position', 'x'), g('position', 'y'), math.degrees(yaw)
