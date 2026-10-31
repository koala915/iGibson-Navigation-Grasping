"""Hardware-free regressions for the patrol mask change."""
import json
from pathlib import Path
import subprocess
import sys
import os
import argparse

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent/'verification'
TESTS = [
    ['tests/test_patrol_mask_reduction.py'],
    ['tests/test_mission_loop.py'],
    ['tests/test_placement_dock.py'],
    ['tests/test_flow_contracts.py'],
    ['tests/test_patrol_vision_permit.py'],
    ['tests/test_safety_guards.py'],
    ['grasp/v21/test_deploy_controller.py'],
    ['grasp/v21/test_servo_read.py'],
    ['grasp/v21/test_deploy_floor_guard.py'],
    ['integration/mission_fsm.py', '--selftest'],
    ['ui/test_server.py'],
]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--only', nargs='*')
    parser.add_argument('--output', type=Path, default=OUT)
    parser.add_argument('--extra-tests', nargs='*', default=[])
    args = parser.parse_args()
    OUT = args.output.resolve()
    OUT.mkdir(parents=True, exist_ok=True)
    temporary = OUT/'tmp'
    temporary.mkdir(exist_ok=True)
    env = dict(os.environ, TEMP=str(temporary), TMP=str(temporary), TMPDIR=str(temporary),
               PYTHONIOENCODING='utf-8')
    summary_path = OUT/'summary.json'
    summary = json.loads(summary_path.read_text()) if args.only and summary_path.exists() else []
    for command in TESTS + [[test] for test in args.extra_tests]:
        if args.only and command[0] not in args.only:
            continue
        result = subprocess.run([sys.executable]+command, cwd=ROOT, env=env,
                                capture_output=True, text=True, encoding='utf-8', timeout=180)
        (OUT/(Path(command[0]).stem+'.txt')).write_text(result.stdout+'\n'+result.stderr, encoding='utf-8')
        summary = [item for item in summary if item['test'] != command]
        summary.append(dict(test=command, exit_code=result.returncode))
        print(command[0], result.returncode, flush=True)
    summary_path.write_text(json.dumps(summary, indent=2))
    sys.exit(any(item['exit_code'] for item in summary))
