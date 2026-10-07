#!/usr/bin/env python3
"""Single v23 finalist entry point; no torch/model/serial initialization."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'integration'))

if __name__ == '__main__':
    import subprocess
    commands = {
        'check': [str(ROOT/'integration/g2_safety_audit.py'),
                  '--vision-health-file','/tmp/x3plus_vision_health.json'],
        'probe': [str(ROOT/'integration/g2_nav_client.py'),'--probe'],
        'map': [str(ROOT/'deploy/ros/g2_demo_mapping.py')],
        'supervise': [str(ROOT/'integration/g2_demo_supervisor.py')],
        'arm': [str(ROOT/'grasp/v23/graspctl.py')],
        'monitor': [str(ROOT/'integration/v23_recovery.py')],
    }
    if len(sys.argv)>1 and sys.argv[1] in commands:
        raise SystemExit(subprocess.call([sys.executable]+commands[sys.argv[1]]+sys.argv[2:]))
    from v23_demo import main
    raise SystemExit(main())
