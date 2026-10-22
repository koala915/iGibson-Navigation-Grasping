"""Read-only health, map/policy hashes and active robot controller inventory."""
import json
from pathlib import Path
import subprocess

SCRIPT = r'''
import hashlib,json,pathlib,socket,time
root=pathlib.Path('/home/jetson/Documents/deploy_jetson2')
def rpc(command,path='/tmp/grasp_health.sock'):
    sock=socket.socket(socket.AF_UNIX);sock.settimeout(3)
    try:
        sock.connect(path);sock.sendall((command+'\n').encode())
        return json.loads(sock.makefile().readline())
    finally:sock.close()
out={'health':rpc('health'),'status':rpc('status','/tmp/grasp_service.sock'),'stamp':time.time()}
loc=pathlib.Path('/dev/shm/g2_localization_health.json')
out['localization']=json.loads(loc.read_text()) if loc.exists() else None
if out['localization']:out['localization']['age']=time.time()-out['localization']['stamp']
names=['integration/g2_patrol_policy.py','maps/demo_box_20261018_patrol/scene_layout.json',
       'maps/demo_box_20261018_patrol/navigation_center_mask.pgm','deploy/ros/g2_map_patrol.py']
out['hashes']={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names}
out['controllers']=[]
for p in pathlib.Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:args=(p/'cmdline').read_bytes().split(b'\0')
    except OSError:continue
    if any(a.rsplit(b'/',1)[-1] in (b'g2_map_patrol.py',b'mission_loop.py',b'sugarbox_approach_grasp.py') for a in args):
        out['controllers'].append({'pid':int(p.name),'command':b' '.join(args).decode(errors='replace')})
print(json.dumps(out))
'''

if __name__ == '__main__':
    result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',
                           'jetson@10.53.253.252','python3 -'],input=SCRIPT,text=True,
                          capture_output=True,timeout=30,check=True)
    record=json.loads(result.stdout)
    path=Path(__file__).resolve().parent/'start_state.json'
    path.write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))
