import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
c = root.tag.split('}')[0].strip('{')
q = lambda t: '{%s}%s' % (c, t)
unit = float(root.find('.//' + q('asset') + '/' + q('unit')).get('meter'))
m = [float(x) for x in next(root.iter(q('matrix'))).text.split()]
M = [m[0:4], m[4:8], m[8:12]]

pts = set()
for mesh in root.iter(q('mesh')):
    sources = {s.get('id'): s for s in mesh.findall(q('source'))}
    for vert in mesh.findall(q('vertices')):
        for inp in vert.findall(q('input')):
            if inp.get('semantic') == 'POSITION':
                f = [float(x) for x in sources[inp.get('source').lstrip('#')].find(q('float_array')).text.split()]
                for i in range(0, len(f) - 2, 3):
                    x, y, z = f[i:i + 3]
                    pts.add(tuple(round((M[r][0]*x + M[r][1]*y + M[r][2]*z + M[r][3]) * unit, 3) for r in range(3)))
for p in sorted(pts, key=lambda p: (p[2], p[0], p[1])):
    print('x=%7.3f  y=%7.3f  z=%6.3f' % p)
print(len(pts), 'distinct vertices')
