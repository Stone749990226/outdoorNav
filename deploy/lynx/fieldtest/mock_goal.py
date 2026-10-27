"""Submit an on-road goal ~distance metres ahead of the current mock pose and follow it."""
import json, math, sys, time, urllib.request
from pathlib import Path

BASE = 'http://127.0.0.1:8766'
DISTANCE = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0

def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.load(r)

def post(path, payload):
    req = urllib.request.Request(BASE + path, data=json.dumps(payload).encode(),
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=8) as r:
        return json.load(r)

status = get('/api/status')
pose = status['pose']
mapinfo = get('/api/map')
bundle = json.load(open(str(Path(__file__).resolve().parents[3] / 'scenes/north_campus/generated/bundle.json')))
roads = bundle['network']['roads']
best = None
for road in roads:
    pts = road['points']
    for a, b in zip(pts, pts[1:]):
        ax, ay, bx, by = a['x'], a['y'], b['x'], b['y']
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy) or 1e-9
        t = max(0.0, min(1.0, ((pose['x'] - ax) * dx + (pose['y'] - ay) * dy) / (length ** 2)))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(px - pose['x'], py - pose['y'])
        if best is None or d < best[0]:
            best = (d, road, pts.index(a), t, length)
_, road, index, t, length = best
remaining = DISTANCE
pts = road['points']
seg = length * (1 - t)
point = None
i = index
while i + 1 < len(pts):
    seg_len = math.hypot(pts[i + 1]['x'] - pts[i]['x'], pts[i + 1]['y'] - pts[i]['y'])
    seg_len = seg_len * (1 - t) if i == index else seg_len
    if seg_len >= remaining:
        frac = (remaining / seg_len) if seg_len else 0
        ax, ay, bx, by = pts[i]['x'], pts[i]['y'], pts[i + 1]['x'], pts[i + 1]['y']
        point = {'x': ax + (bx - ax) * frac, 'y': ay + (by - ay) * frac}
        break
    remaining -= seg_len
    i += 1
if point is None:
    point = {'x': pts[-1]['x'], 'y': pts[-1]['y']}
print('goal', point, 'road', road['id'], 'distance', DISTANCE)
plan = post('/api/plans', {'frame_id': 'map', 'scene_version': mapinfo['scene_version'],
                            'route_version': mapinfo['route_version'], 'goal': point})
print('plan poses', len(plan.get('reference_path', plan.get('path', []))))
task = post('/api/tasks', {'frame_id': 'map', 'scene_version': mapinfo['scene_version'],
                            'route_version': mapinfo['route_version'], 'goal': point})
task_id = task['task_id']
print('task', task_id, task['state'])
deadline = time.time() + 120
while time.time() < deadline:
    time.sleep(4)
    t = get('/api/tasks/' + task_id)
    s = get('/api/status')
    print('%.0fs state=%s pose=(%.2f,%.2f) yaw=%.2f' % (
        time.time() - (deadline - 120), t['state'], s['pose']['x'], s['pose']['y'], s['pose']['yaw']))
    if t['state'] in ('succeeded', 'failed', 'canceled'):
        print('final', json.dumps(t, ensure_ascii=False)[:300])
        break
