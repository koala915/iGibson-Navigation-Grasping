"""Supervised test from the operator-confirmed, unchanged run19 post-place stop.

Restore only measured prior retreat budget after matching owner and odometry.
Use normal Room recovery/physical guards, then the unmodified mission entry.
No relocalization, generic startup reverse or fake rear-clear confirmation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from integration import mission_loop as mission, placement_dock as dock
from read_start_state import SCRIPT


def context(state, previous, transcript):
    health=state['health'];ch=health['chassis']
    if (state['controllers'] or health['service_pid']!=previous['final_health']['service_pid']
            or health.get('holding') is not False or ch.get('moving') is not False
            or ch.get('arm_busy') is not False or ch.get('arm_pose')!='travel'
            or ch.get('motors')!=[0,0,0,0] or ch.get('client') is not None):
        raise RuntimeError('previous post-placement state is not safely resumable')
    old,new=previous['initial']['odom'],health['odom']['pose']
    if (math.hypot(old[0]-new[0],old[1]-new[1])>.002
            or abs(dock.wrap(old[2]-new[2]))>.003):
        raise RuntimeError('robot odometry changed since the previous post-placement stop')
    pose=dock.localization_pose(state['localization'])
    prior=previous['initial']['pose']
    if math.hypot(pose[0]-prior[0],pose[1]-prior[1])>.10 or abs(dock.wrap(pose[2]-prior[2]))>math.radians(20):
        raise RuntimeError('localized pose no longer agrees with the recorded stop')
    if '[MISSION] 1 delivered' not in transcript or previous.get('pulses')!=0 or previous.get('cleanup_error'):
        raise RuntimeError('no verified delivered-and-stopped context')
    tail=transcript.rsplit('[MISSION] 1 delivered',1)[1]
    steps=[float(x) for x in re.findall(r'\[ADJUST\] odometry moved -(\d+\.\d+) m',tail)]
    if not steps or not 0<sum(steps)<mission.ROOM_EXIT_MAX_M:
        raise RuntimeError('no valid measured previous back-out budget')
    return sum(steps)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real',action='store_true')
    parser.add_argument('--i-am-beside-the-robot',action='store_true')
    parser.add_argument('--operator-confirmed-rear-clear',action='store_true')
    args=parser.parse_args()
    if not args.real or not args.i_am_beside_the_robot or not args.operator_confirmed_rear_clear:
        raise SystemExit('real test requires the operator beside robot and explicit rear-clear confirmation')
    host='10.53.253.252'
    if mission.main(['--host',host]) != 0:
        raise SystemExit('preflight failed; post-placement test not started')
    folder=ROOT/'logs/mission_20261021_patrol21'
    folder.mkdir(exist_ok=True)
    r=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','jetson@'+host,'python3 -'],
                     input=SCRIPT,text=True,capture_output=True,timeout=30,check=True)
    state=json.loads(r.stdout)
    previous=json.loads((Path(__file__).parent/'before/mission_patrol.json').read_text())
    transcript=(ROOT/'logs/mission_20261021_patrol19/mission.out').read_text(encoding='utf-8')
    backed=context(state,previous,transcript)
    layout=dock.fetch_layout(host);box=dock.box_from_layout(layout);obstacle=dock.bin_obstacle(box)
    mask=dock.fetch_nav_mask(host)
    # The tested map/policy must still be exactly the deployed version.
    for name in ('integration/g2_patrol_policy.py','maps/demo_box_20261018_patrol/navigation_center_mask.pgm',
                 'maps/demo_box_20261018_patrol/scene_layout.json'):
        if state['hashes'][name]!=hashlib.sha256((ROOT/name).read_bytes()).hexdigest():
            raise RuntimeError('robot/tested configuration differs: '+name)
    room=mission.Room(host,layout['room_polygon'],obstacles=[obstacle],mask=mask,
                      locate=lambda h:dock.localization_pose(dock.read_localization(h)))
    room.backed=backed  # measured 0.232+0.065m, proven unchanged since run19, not a fabricated stage
    report={'previous_backed_m':backed,'same_owner_and_odom':True,
            'operator_confirmed_rear_clear':True,'ok':False}
    try:
        pose=room._pose()
        if not mission.patrol_exit_allowed(mask,pose):
            print('[RESUME_EXIT] previous delivery stopped after {:.3f}m; completing verified exit'.format(backed),flush=True)
            if not room.make():raise RuntimeError('no safe completion within the remaining back-out budget')
        pose=room._pose()
        if not mission.patrol_exit_allowed(mask,pose):raise RuntimeError('exit margin not confirmed')
        report.update(ok=True,actual_total_backed_m=room.backed,pose=pose)
    finally:
        mission.runner.stop(host)
        mission.runner.service_command(host,'maintenance-lock')
        (folder/'resume_exit.json').write_text(json.dumps(report,indent=2)+'\n')
    print('[RESUME_EXIT] actual exit verified; starting patrol without relocalization',flush=True)
    raise SystemExit(mission.main(['--host',host,'--real','--i-am-beside-the-robot',
                                  '--log-dir',str(folder)]))
