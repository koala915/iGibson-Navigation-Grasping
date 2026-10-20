"""Actual model warm-up/inference and refused-go smoke test, with no robot I/O."""
import json
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from integration.mission_runtime import MissionRuntime
from integration.sugarbox_ground_calibration import resolve_asset_paths


class NoCamera:
    def close(self):
        pass


def main():
    import numpy as np
    folder = Path(__file__).resolve().parent / 'model_smoke'
    assets = resolve_asset_paths(str(ROOT))
    models = [ROOT / 'detection/models/best.pt', ROOT / 'detection/models/placement_box_best.pt']
    # Use the exact placement asset named by the real runner.
    from integration.sugarbox_approach_grasp import PLACEMENT_MODEL, find_gstreamer
    models[1] = PLACEMENT_MODEL
    runtime = MissionRuntime('127.0.0.1', 'NO_CAMERA', folder)
    runtime.receiver = NoCamera()  # never launch GStreamer or open UDP
    environments = []
    try:
        for target, model, size in zip(('sugarbox', 'placement'), models, (1280, 640)):
            environments.append(dict(os.environ, SUGARBOX_RUN_MODE='VISION',
                SUGARBOX_TARGET=target, SUGARBOX_YOLO_MODEL=str(model),
                SUGARBOX_SAM_MODEL=str(assets['sam']), SUGARBOX_HOMOGRAPHY=str(assets['homography']),
                SUGARBOX_YOLO_IMGSZ=str(size), SUGARBOX_WAIT_GO='1', ROUTE_B_IP='127.0.0.1',
                SUGARBOX_TRAINING_CONTRACT='1', SUGARBOX_CONTRACT_ALLOW_DRIVE='0'))
            environments[-1]['GSTREAMER_LAUNCH_EXE'] = str(find_gstreamer())
        started = time.monotonic()
        runtime.prepare(environments)
        initial = time.monotonic()-started
        # All real code paths here use VISION, no ROS bridge or MotorClient is constructed.
        pid = runtime.process.pid
        records = []
        for i, env in enumerate(environments * 2):
            started = time.monotonic()
            boxes = runtime.detect(env['SUGARBOX_YOLO_MODEL'], np.zeros((480, 640, 3), np.uint8),
                                   int(env['SUGARBOX_YOLO_IMGSZ']), 0.7)
            runtime.stage_log = folder / ('refused_go_{}.log'.format(i))
            engine = runtime.start_engine(env)
            engine.write('{"go":false}\n')
            code = engine.wait(timeout=30)
            assert code == 5, code
            assert runtime.process.pid == pid
            records.append({'target':env['SUGARBOX_TARGET'], 'boxes':len(boxes),
                            'exit_code':code, 'seconds':round(time.monotonic()-started, 3)})
        (folder/'results.json').write_text(json.dumps({'prepare_seconds':initial,
            'worker_pid':pid, 'runs':records, 'hardware_connected':False},indent=2)+'\n',encoding='utf-8')
        print(json.dumps({'prepare_seconds':round(initial,3),'runs':records}))
    finally:
        runtime.close()


if __name__ == '__main__':
    mp.freeze_support()
    main()
