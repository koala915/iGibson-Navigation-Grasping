"""Read-only live preflight plus isolated offline tests of the current checkout."""
import concurrent.futures
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent / 'verify_2305'
OUT.mkdir(exist_ok=True)
TEMP = OUT / 'tmp'
TEMP.mkdir(exist_ok=True)
ENV = dict(os.environ, PYTHONUTF8='1', TEMP=str(TEMP), TMP=str(TEMP), TMPDIR=str(TEMP))
SUITES = ['tests/test_mission_loop.py', 'tests/test_flow_contracts.py',
          'tests/test_mission_runtime.py', 'tests/test_sugarbox_seed_target.py',
          'tests/test_sugarbox_approach_grasp.py', 'tests/test_sugarbox_blind_finish.py',
          'tests/test_placement_dock.py', 'tests/test_patrol_vision_permit.py']


def hashes():
    files = SUITES + ['integration/mission_loop.py', 'integration/robot_rpc.py',
                     'integration/mission_runtime.py', 'integration/sugarbox_approach_grasp.py',
                     'integration/sugarbox_rl_approach_final2.py']
    return {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in files}


def run(path):
    started = time.monotonic()
    result = subprocess.run([sys.executable, path], cwd=ROOT, env=ENV,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=180)
    output = result.stdout.decode('utf-8', errors='replace')
    log = OUT/(Path(path).stem+'.txt')
    log.write_text(output, encoding='utf-8')
    return {'suite':path, 'exit_code':result.returncode, 'seconds':round(time.monotonic()-started, 2),
            'log':str(log), 'tail':output[-4000:] if result.returncode else output[-350:]}


if __name__ == '__main__':
    before = hashes()
    started = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        results = list(pool.map(run, SUITES))
    after = hashes()
    summary = {'started_at':started, 'source_hashes':before,
               'changed_during_tests':[p for p in before if before[p]!=after[p]], 'results':results}
    (OUT/'offline_results.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n',encoding='utf-8')
    for item in results:
        print(json.dumps(item, ensure_ascii=False), flush=True)
