"""Run repository offline suites in isolated processes; never invokes real mode."""
import concurrent.futures
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
REPORT = Path(__file__).resolve().parent
ROUND = sys.argv[1]
TARGETED = [
    'tests/test_mission_runtime.py', 'tests/test_flow_contracts.py', 'tests/test_mission_loop.py', 'tests/test_sugarbox_approach_grasp.py',
    'tests/test_sugarbox_seed_target.py', 'tests/test_sugarbox_blind_finish.py',
    'tests/test_placement_dock.py', 'tests/test_patrol_vision_permit.py',
]
if ROUND == 'targeted':
    SUITES = [[p] for p in TARGETED]
elif ROUND == 'network':
    SUITES = [['grasp/v23/test_chassis_server.py'], ['ui/test_server.py']]
else:
    workflow = (ROOT / '.github/workflows/tests.yml').read_text(encoding='utf-8')
    SUITES = [line.split() for line in re.findall(r'^\s+run: python (.+)$', workflow, re.M)
              if line.split()[0] not in TARGETED + ['grasp/v23/test_chassis_server.py', 'ui/test_server.py']]
    SUITES += [['grasp/v23/test_deploy_controller.py'],
               ['grasp/v23/test_servo_read.py'],
               ['grasp/v23/test_deploy_floor_guard.py']]

OUT = REPORT / ROUND
OUT.mkdir(parents=True, exist_ok=True)
TEMP = REPORT / 'tmp'
TEMP.mkdir(exist_ok=True)
ENV = dict(os.environ, PYTHONUTF8='1', PYTHONIOENCODING='utf-8',
           TEMP=str(TEMP), TMP=str(TEMP), TMPDIR=str(TEMP))
tracked = subprocess.check_output(['git', 'diff', '--name-only'], cwd=ROOT,
                                 stderr=subprocess.DEVNULL, text=True).splitlines()
tracked += ['integration/patrol_vision_permit_jetson.py', 'tests/test_patrol_vision_permit.py', 'tests/test_flow_contracts.py', 'integration/mission_runtime.py', 'integration/robot_rpc.py', 'tests/test_mission_runtime.py']

def hashes():
    return {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
            for p in tracked if (ROOT / p).is_file()}

def run(args):
    start = time.monotonic()
    result = subprocess.run([sys.executable] + args, cwd=ROOT, env=ENV,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=240)
    output = result.stdout.decode('utf-8', errors='replace')
    logfile = OUT / (args[0].replace('/', '__') + '.txt')
    logfile.write_text(output, encoding='utf-8')
    return {'suite': ' '.join(args), 'exit_code': result.returncode,
            'seconds': round(time.monotonic() - start, 2), 'log': str(logfile),
            'counts': re.findall(r'Ran \d+ tests?[^\n]*|\d+ passed[^\n]*|ALL[^\n]*PASS[^\n]*', output),
            'tail': output[-2400:] if result.returncode else output[-350:]}

if __name__ == '__main__':
    before = hashes()
    started = datetime.datetime.now().astimezone().isoformat()
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        for future in concurrent.futures.as_completed([pool.submit(run, args) for args in SUITES]):
            item = future.result()
            results.append(item)
            print(json.dumps(item, ensure_ascii=False), flush=True)
    after = hashes()
    summary = {'started_at': started, 'python': sys.executable,
               'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
               'source_hashes': before,
               'changed_during_tests': [p for p in before if before[p] != after.get(p)],
               'results': results}
    (OUT / 'results.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
    print('REPORT', OUT / 'results.json', flush=True)
    sys.exit(1 if any(r['exit_code'] for r in results) else 0)
