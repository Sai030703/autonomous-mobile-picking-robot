import sys
import xml.etree.ElementTree as ET
from collections import Counter

root = ET.parse(sys.argv[1]).getroot()
c = root.tag.split('}')[0].strip('{')
q = lambda t: '{%s}%s' % (c, t)

unit_el = root.find('.//' + q('asset') + '/' + q('unit'))
up_el = root.find('.//' + q('asset') + '/' + q('up_axis'))
unit = float(unit_el.get('meter')) if unit_el is not None else 1.0
up = up_el.text.strip() if up_el is not None else 'Y_UP'
print('unit (m per file unit):', unit, '| up_axis:', up)

M = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
mats = list(root.iter(q('matrix')))
if mats:
    v = [float(x) for x in mats[0].text.split()]
    M = [v[0:4], v[4:8], v[8:12], v[12:16]]
if len(mats) > 1:
    print('WARNING: %d matrices found, only the first is applied' % len(mats))
print('node matrix (file units):')
for r in M:
    print('  ', r)

def tf(p):
    x, y, z = p
    return [(M[i][0] * x + M[i][1] * y + M[i][2] * z + M[i][3]) * unit for i in range(3)]

pts = []
for mesh in root.iter(q('mesh')):
    sources = {s.get('id'): s for s in mesh.findall(q('source'))}
    for vert in mesh.findall(q('vertices')):
        for inp in vert.findall(q('input')):
            if inp.get('semantic') == 'POSITION':
                arr = sources[inp.get('source').lstrip('#')].find(q('float_array'))
                f = [float(x) for x in arr.text.split()]
                pts += [tf(f[i:i + 3]) for i in range(0, len(f) - 2, 3)]

print('vertices:', len(pts))
for i, name in enumerate('XYZ'):
    lo = min(p[i] for p in pts)
    hi = max(p[i] for p in pts)
    print('%s after transform: %.3f to %.3f  (size %.3f m)' % (name, lo, hi, hi - lo))

axis = {'X_UP': 0, 'Y_UP': 1, 'Z_UP': 2}.get(up, 2)
bins = Counter(int(round(p[axis] / 0.01)) for p in pts)
print('vertical-axis heights with 4+ vertices (likely board surfaces):')
top = sorted(((n, k) for k, n in bins.items() if n >= 4), reverse=True)[:15]
for n, k in sorted(top, key=lambda t: t[1]):
    print('  %.2f m : %d vertices' % (k * 0.01, n))
