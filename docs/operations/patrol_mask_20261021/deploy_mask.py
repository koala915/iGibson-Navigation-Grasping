"""Back up and install only the tested mask/policy while the robot is stopped.

No service restart, motion, localization reset, arm or motor commands.
"""
import base64
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
FILES = ['integration/g2_patrol_policy.py',
         'maps/demo_box_20261018_patrol/navigation_center_mask.pgm',
         'maps/demo_box_20261018_patrol/scene_layout.json']
REMOTE = r'''
import base64,datetime,hashlib,importlib.util,json,math,os,pathlib,shutil,socket,sys
payload = json.loads(sys.stdin.read())
root = pathlib.Path('/home/jetson/Documents/deploy_jetson2')
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def health():
    sock=socket.socket(socket.AF_UNIX);sock.settimeout(3)
    try:
        sock.connect('/tmp/grasp_health.sock');sock.sendall(b'health\n')
        return json.loads(sock.makefile().readline())
    finally: sock.close()
def idle():
    h=health();ch=h['chassis']
    if (h.get('holding') is not False or ch.get('moving') is not False or
        ch.get('arm_busy') is not False or ch.get('arm_pose')!='travel' or
        ch.get('maintenance') is not True or ch.get('client') is not None or
        ch.get('motors')!=[0,0,0,0]):
        raise RuntimeError('robot must be idle, empty, in travel and maintenance before config update')
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try: args=(p/'cmdline').read_bytes().split(b'\0')
        except (OSError,IOError): continue
        if any(a.rsplit(b'/',1)[-1]==b'g2_map_patrol.py' for a in args):
            raise RuntimeError('patrol process still exists; refusing update')
    return h
for name,expected in payload['expected'].items():
    if digest(root/name)!=expected:
        raise RuntimeError('file changed since snapshot; refusing overwrite: '+name)
initial=idle()
stamp=datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%S%fZ')
backup=root/'backups'/('patrol_mask_'+stamp)
backup.mkdir(parents=True)
staged={}
for name,data in payload['files'].items():
    source=root/name; saved=backup/name; saved.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(str(source),str(saved))
    stage=source.with_name(source.name+'.mask-update-'+stamp)
    stage.write_bytes(base64.b64decode(data));shutil.copymode(str(source),str(stage))
    if name.endswith('.py'): compile(stage.read_text(),str(source),'exec')
    staged[name]=stage
replaced=[]
try:
    if idle()['service_pid']!=initial['service_pid']: raise RuntimeError('owner changed')
    for name,stage in staged.items():
        if digest(root/name)!=payload['expected'][name]: raise RuntimeError('concurrent edit: '+name)
    for name,stage in staged.items():
        os.replace(str(stage),str(root/name));replaced.append(name)
    spec=importlib.util.spec_from_file_location('checked_policy',str(root/'integration/g2_patrol_policy.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    policy=module.Policy(str(root/'maps/demo_box_20261018_patrol'))
    policy.require_motion_ready(.10)
    pose=payload['run19_pose']
    if not all(policy.allowed(pose[0]+.06*math.cos(i*math.pi/360),
                              pose[1]+.06*math.sin(i*math.pi/360)) for i in range(720)):
        raise RuntimeError('run19 turn-reserve replay failed')
    final=idle()
    if final['service_pid']!=initial['service_pid']: raise RuntimeError('owner changed')
except Exception:
    for name in reversed(replaced): shutil.copy2(str(backup/name),str(root/name))
    raise
finally:
    for stage in staged.values():
        if stage.exists(): stage.unlink()
print(json.dumps({'ok':True,'backup':str(backup),'hashes':{name:digest(root/name) for name in staged},
                  'service_pid':final['service_pid'],'moving':final['chassis']['moving'],
                  'motors':final['chassis']['motors'],'maintenance':final['chassis']['maintenance'],
                  'run19_turn_reserve_passed':True,'motion_commands_sent':0},indent=2))
'''

if __name__ == '__main__':
    summary = json.loads((HERE/'verification/summary.json').read_text())
    if len(summary) != 11 or any(test['exit_code'] for test in summary):
        raise SystemExit('all regression suites must pass first')
    expected = json.loads((HERE/'before/hashes.json').read_text())
    dependencies = FILES + ['maps/demo_box_20261018_patrol/navigation_center_mask.yaml',
                            'maps/demo_box_20261018_patrol/demo_box.pgm',
                            'maps/demo_box_20261018_patrol/demo_box.yaml',
                            'deploy/ros/g2_map_patrol.py']
    payload = dict(expected={name:expected[name] for name in dependencies},
                   files={name:base64.b64encode((ROOT/name).read_bytes()).decode() for name in FILES},
                   run19_pose=json.loads((HERE/'before/mission_patrol.json').read_text())['initial']['pose'])
    # The shell command contains only static Python source encoded as base64;
    # data and all paths are passed as stdin JSON, never interpolated in a shell.
    encoded = base64.b64encode(REMOTE.encode()).decode()
    command = "python3 -c \"import base64;exec(base64.b64decode('{}'))\"".format(encoded)
    result = subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',
                             'jetson@10.53.253.252',command], input=json.dumps(payload),
                            text=True,capture_output=True,timeout=60)
    (HERE/'deployment.txt').write_text(result.stdout+'\n'+result.stderr)
    print(result.stdout)
    if result.returncode:
        raise SystemExit(result.stderr)
    reply = json.loads(result.stdout)
    for name in FILES:
        if reply['hashes'][name] != hashlib.sha256((ROOT/name).read_bytes()).hexdigest():
            raise SystemExit('remote/local hash mismatch: '+name)
    (HERE/'deployment.json').write_text(json.dumps(reply,indent=2)+'\n')
