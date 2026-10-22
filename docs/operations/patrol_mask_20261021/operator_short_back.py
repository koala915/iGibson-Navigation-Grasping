"""One operator-supervised 10cm back-out, only with explicit rear-clear confirmation.

This is a manual positioning action, not automatic startup recovery. Never run
while a mission is active. All motion uses the existing resident TCP owner.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from integration import mission_loop as mission, placement_dock as dock
from read_start_state import SCRIPT as STATE_SCRIPT


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-confirmed-rear-clear',action='store_true')
    parser.add_argument('--i-am-beside-the-robot',action='store_true')
    args=parser.parse_args()
    if not args.operator_confirmed_rear_clear or not args.i_am_beside_the_robot:
        raise SystemExit('requires the operator beside the robot and explicit rear-clear confirmation')
    host='10.53.253.252'
    report={'host':host,'requested_back_m':.10,'speed_mps':.04,
            'operator_confirmed_rear_clear':True}
    check=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',
                          'jetson@'+host,'python3 -'],input=STATE_SCRIPT,text=True,
                         capture_output=True,timeout=30,check=True)
    state=json.loads(check.stdout)
    if state['controllers']:
        raise SystemExit('another robot controller is running; refusing manual positioning')
    # Obtain a fresh map, box envelope and pose. Nothing moves during these checks.
    layout=dock.fetch_layout(host)
    box=dock.box_from_layout(layout)
    obstacle=dock.bin_obstacle(box)
    mask=dock.fetch_nav_mask(host)
    pose=dock.localization_pose(dock.read_localization(host))
    walls=layout['room_polygon']
    problem=dock.back_path_problem(pose,.10,walls,dock.BACK_WALL_MARGIN_M,[obstacle])
    endpoint=(pose[0]-.10*math.cos(pose[2]),pose[1]-.10*math.sin(pose[2]))
    if problem or not mission.patrol_start_allowed(mask,endpoint):
        raise SystemExit('short back-out does not have a verified safe exit: '+str(problem))
    report['start_pose']=pose
    try:
        mission.prepare_back_out(host)
        report['actual_back_m']=mission.runner.back_straight(host,.10,walls,speed=.04,
                                                            obstacles=[obstacle])
        after=dock.localization_pose(dock.read_localization(host))
        report['end_pose']=after
        report['patrol_turn_allowed']=mission.patrol_start_allowed(mask,after[:2])
        if not report['patrol_turn_allowed']:
            raise RuntimeError('actual endpoint cannot start patrol; stopped, no further backing')
        report['ok']=True
    except Exception as exc:
        report.update(ok=False,error=str(exc))
        raise
    finally:
        try:
            mission.runner.stop(host)
            report['maintenance_lock']=mission.runner.service_command(host,'maintenance-lock')
        finally:
            path=ROOT/'logs/mission_20261021_patrol20/operator_back_result.json'
            path.write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2))
