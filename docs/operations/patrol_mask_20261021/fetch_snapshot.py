"""Read-only snapshot of the robot's patrol policy and map; never sends motion."""
import base64
import json
from pathlib import Path
import subprocess

DEST = Path(__file__).resolve().parent / 'before'
SCRIPT = r'''
import base64, hashlib, json, pathlib
root = pathlib.Path('/home/jetson/Documents/deploy_jetson2')
files = list((root/'deploy/ros').glob('g2*patrol*.py'))
files += [root/'integration'/name for name in
          ('g2_patrol_policy.py', 'g2_geometry.py', 'g2_navigation_goals.py')]
files += list((root/'maps/demo_box_20261018_patrol').glob('*'))
files += [pathlib.Path('/tmp/mission_patrol.json')]
out = {}
for p in files:
    if not p.is_file(): continue
    data = p.read_bytes()
    key = str(p.relative_to(root)) if str(p).startswith(str(root)) else 'mission_patrol.json'
    out[key] = {'sha256': hashlib.sha256(data).hexdigest(), 'data': base64.b64encode(data).decode()}
print(json.dumps(out))
'''

if __name__ == '__main__':
    result = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                             'jetson@10.53.253.252', 'python3 -'], input=SCRIPT,
                            text=True, capture_output=True, timeout=60, check=True)
    records = json.loads(result.stdout)
    DEST.mkdir(parents=True, exist_ok=True)
    for name, record in records.items():
        path = DEST / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(base64.b64decode(record['data']))
    (DEST/'hashes.json').write_text(json.dumps({k:v['sha256'] for k,v in records.items()}, indent=2))
    print(json.dumps({'files': list(records), 'destination': str(DEST)}, indent=2))
