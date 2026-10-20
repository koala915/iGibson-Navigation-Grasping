"""Mission-scoped model process and one rear-camera receiver. No hardware at import.

Only immutable assets and decoded frames survive a stage. A fresh engine module
and all of its controllers are built for every run. A failed/overrunning worker
is terminated, never replayed; the caller must stop the mission.
"""
from contextlib import contextmanager, redirect_stdout, redirect_stderr
import importlib.util
import json
import multiprocessing as mp
import os
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
ENGINE = ROOT / 'integration/sugarbox_rl_approach_final2.py'
HEADER = struct.Struct('<Qdd')  # sequence, wall receipt, monotonic receipt
WIDTH, HEIGHT = 640, 480


class SharedFrames:
    def __init__(self, context=None):
        context = context or mp.get_context('spawn')
        self.buffer = context.RawArray('B', HEADER.size + WIDTH * HEIGHT * 3)
        self.lock = context.RLock()

    @contextmanager
    def locked(self):
        # A terminated worker may have died while copying a frame. Never leave
        # the receiver or the next reader blocked forever on an orphaned lock.
        if not self.lock.acquire(timeout=1.0):
            raise RuntimeError('rear frame lock unavailable; mission must stop')
        try:
            yield
        finally:
            self.lock.release()

    def publish(self, frame, wall=None, monotonic=None):
        if tuple(frame.shape) != (HEIGHT, WIDTH, 3) or str(frame.dtype) != 'uint8':
            raise ValueError('rear frame must be uint8 640x480 BGR')
        wall = time.time() if wall is None else wall
        monotonic = time.monotonic() if monotonic is None else monotonic
        with self.locked():
            view = memoryview(self.buffer).cast('B')
            sequence = HEADER.unpack_from(view)[0] + 1
            view[HEADER.size:] = frame.tobytes()
            HEADER.pack_into(view, 0, sequence, wall, monotonic)

    def snapshot(self, since=0.0, clock='monotonic'):
        import numpy as np
        with self.locked():
            view = memoryview(self.buffer).cast('B')
            sequence, wall, mono = HEADER.unpack_from(view)
            if not sequence or mono <= since:
                return None, 0.0
            raw = bytes(view[HEADER.size:])
        return np.frombuffer(raw, np.uint8).reshape(HEIGHT, WIDTH, 3), (
            wall if clock == 'wall' else mono)

    def lease(self):
        return FrameLease(self, time.monotonic())


class FrameLease:
    """A new stage cannot consume a frame from before it acquired its lease."""
    def __init__(self, frames, since):
        self.frames, self.since, self.closed = frames, since, False

    def latest(self):
        return (None, 0.0) if self.closed else self.frames.snapshot(self.since)

    def read(self):
        frame, stamp = (None, 0.0) if self.closed else self.frames.snapshot(self.since, 'wall')
        return frame is not None, frame, stamp

    def close(self):
        self.closed = True

    release = close


class Resources:
    def __init__(self, frames):
        self.frames, self.assets, self.warmed = frames, {}, set()
        self.identities, self.poisoned = {}, False

    def _key(self, kind, paths):
        paths = tuple(str(Path(p).resolve()) for p in paths)
        identities = tuple((Path(p).stat().st_size, Path(p).stat().st_mtime_ns) for p in paths)
        key = (kind,) + paths
        if key in self.identities and self.identities[key] != identities:
            raise RuntimeError('asset changed during mission; restart after preflight: ' + str(paths))
        self.identities[key] = identities
        return key

    def get(self, kind, paths, load):
        if self.poisoned:
            raise RuntimeError('model worker did not finish its previous stage safely')
        key = self._key(kind, paths)
        if key not in self.assets:
            self.assets[key] = load()
        return self.assets[key]

    def models(self, module):
        yolo = self.get('yolo', [module.YOLO_MODEL_PATH], lambda: module.YOLO(module.YOLO_MODEL_PATH))
        sam = self.get('sam', [module.SAM2_MODEL_PATH], lambda: module.SAM(module.SAM2_MODEL_PATH))
        key = (module.YOLO_MODEL_PATH, module.SAM2_MODEL_PATH, module.YOLO_IMGSZ)
        if key not in self.warmed:
            module.warm_up_models(yolo, sam)
            self.warmed.add(key)
        return yolo, sam

    def policy(self, module):
        pair = self.get('ppo', [module.RL_MODEL_PATH, module.VECNORMALIZE_PATH],
                        module.load_policy_and_vecnormalize)
        if pair[1].training or pair[1].norm_reward:
            raise RuntimeError('cached VecNormalize must remain inference-only')
        return pair

    def reader(self):
        return self.frames.lease()


def load_stage(env):
    """Re-evaluate configuration in a new namespace; heavy imports stay cached."""
    previous = dict(os.environ)
    try:
        os.environ.clear()
        os.environ.update(env)
        name = 'integration._mission_approach_stage'
        spec = importlib.util.spec_from_file_location(name, ENGINE)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module  # dataclasses resolve their defining module
        spec.loader.exec_module(module)
        return module
    finally:
        os.environ.clear()
        os.environ.update(previous)


class GoInput:
    def __init__(self, connection, generation):
        self.connection, self.generation = connection, generation

    def readline(self):
        message = self.connection.recv()
        if message.get('op') != 'go' or message.get('generation') != self.generation:
            return '{}\n'
        return message['line']


def worker(connection, frames, stage_loader=load_stage):
    resources = Resources(frames)
    try:
        while True:
            message = connection.recv()
            operation, generation = message['op'], message['generation']
            if operation == 'close':
                return
            if operation not in ('prepare', 'detect', 'run'):
                connection.send({'generation': generation, 'ok': False,
                                 'error': 'unexpected worker command; mission stopped'})
                return
            with open(message['log'], 'a', encoding='utf-8', buffering=1) as log, \
                    redirect_stdout(log), redirect_stderr(log):
                try:
                    if operation == 'prepare':
                        for env in message['environments']:
                            module = stage_loader(env)
                            resources.models(module)
                            resources.policy(module)
                        result = None
                    elif operation == 'detect':
                        from ultralytics import YOLO
                        model = resources.get('yolo', [message['model']],
                                              lambda: YOLO(message['model']))
                        prediction = model.predict(message['frame'], imgsz=message['imgsz'],
                                                   conf=message['conf'], verbose=False)[0]
                        result = [box.xyxy[0].tolist() for box in prediction.boxes]
                    elif operation == 'run':
                        module = stage_loader(message['env'])
                        result = module.main(resources=resources,
                                             go_stream=GoInput(connection, generation), headless=True)
                        if not isinstance(result, int) or isinstance(result, bool):
                            raise RuntimeError('engine ended without an explicit result')
                        if resources.poisoned:
                            raise RuntimeError('vision thread did not exit; refusing cached reuse')
                    else:
                        raise ValueError('unsupported worker operation')
                    connection.send({'generation': generation, 'ok': True, 'result': result})
                except BaseException:
                    traceback.print_exc()
                    connection.send({'generation': generation, 'ok': False,
                                     'error': 'model worker failed; see ' + message['log']})
                    return
    except (EOFError, BrokenPipeError):
        return
    finally:
        connection.close()


class EngineLease:
    def __init__(self, runtime, generation):
        self.runtime, self.generation = runtime, generation
        self.stdin, self.returncode = self, None
        self.started = time.monotonic()
        self.log = runtime.stage_log
        self.offset = 0

    def drain_log(self):
        if self.log.exists():
            with self.log.open('r', encoding='utf-8', errors='replace') as output:
                output.seek(self.offset)
                text = output.read()
                self.offset = output.tell()
            if text:
                print(text, end='', flush=True)

    def write(self, line):
        if self.returncode is not None:
            raise RuntimeError('engine lease already ended')
        self.runtime.connection.send({'op': 'go', 'generation': self.generation, 'line': line})

    def close(self):
        pass  # runner closes stdin after its single go message, not the worker

    def poll(self):
        self.drain_log()
        if self.returncode is not None:
            return self.returncode
        if self.runtime.connection.poll():
            try:
                reply = self.runtime.receive(self.generation)
            except Exception:
                self.returncode = 2
                raise
            self.returncode = int(reply)
            self.runtime.active = None
            self.drain_log()
        elif not self.runtime.process.is_alive():
            self.returncode = 2
            self.runtime.active = None
        return self.returncode

    def wait(self, timeout=None):
        deadline = None if timeout is None else time.monotonic() + timeout
        while self.poll() is None:
            if deadline is not None and time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired('resident approach', timeout)
            time.sleep(0.02)
        self.runtime.metric('approach', self.started, exit_code=self.returncode)
        return self.returncode

    def abort(self):
        self.runtime.abort()
        self.returncode = 2


class Tee:
    def __init__(self, terminal, log):
        self.terminal, self.log = terminal, log

    def write(self, text):
        self.terminal.write(text)
        self.log.write(text)
        return len(text)

    def flush(self):
        self.terminal.flush()
        self.log.flush()


class MissionRuntime:
    def __init__(self, host, gst, log_dir, process_target=worker):
        self.host, self.gst, self.log_dir = host, gst, Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.context = mp.get_context('spawn')
        self.frames = SharedFrames(self.context)
        self.connection, child = self.context.Pipe()
        self.process = self.context.Process(target=process_target, args=(child, self.frames), daemon=True)
        self.process.start()
        child.close()
        self.active, self.receiver, self.generation = None, None, 0
        self.closed = False
        self.owner_thread = threading.get_ident()
        self.stage_log = self.log_dir / 'runtime.log'
        self.metrics = self.log_dir / 'timings.jsonl'

    def usable(self):
        """The worker can still run a stage (an abort ends it for the mission)."""
        return not self.closed and self.process.is_alive()

    def metric(self, event, started, **fields):
        with self.metrics.open('a', encoding='utf-8') as output:
            output.write(json.dumps(dict(event=event, seconds=round(time.monotonic()-started, 4),
                                         **fields), ensure_ascii=False) + '\n')

    def send(self, operation, **fields):
        if threading.get_ident() != self.owner_thread or self.active is not None:
            raise RuntimeError('a mission stage already owns the worker')
        if not self.process.is_alive():
            raise RuntimeError('model worker unavailable; do not replay motion')
        self.generation += 1
        self.connection.send(dict(op=operation, generation=self.generation,
                                  log=str(self.stage_log), **fields))
        return self.generation

    def receive(self, generation, timeout=120):
        if not self.connection.poll(timeout):
            self.abort()
            raise RuntimeError('model worker timed out; mission stopped')
        try:
            reply = self.connection.recv()
        except (EOFError, OSError):
            self.abort()
            raise RuntimeError('model worker connection lost; mission stopped')
        if reply.get('generation') != generation or reply.get('ok') is not True:
            self.abort()
            raise RuntimeError(reply.get('error', 'wrong worker generation; mission stopped'))
        return reply['result']

    def prepare(self, environments):
        started = time.monotonic()
        self.receive(self.send('prepare', environments=environments), timeout=180)
        self.metric('models_ready', started)

    def reader(self):
        if self.receiver is None:
            from integration.rear_stream import StreamReader
            self.receiver = StreamReader(5600, self.gst, publish=self.frames.publish)
        return self.frames.lease()

    def detect(self, model, frame, imgsz, conf):
        return self.receive(self.send('detect', model=str(model), frame=frame, imgsz=imgsz, conf=conf),
                            timeout=15)

    def start_engine(self, env):
        if self.active is not None:
            raise RuntimeError('a mission stage already owns the worker')
        if self.receiver is None:
            self.reader().close()
        self.stage_log.write_text('', encoding='utf-8')
        generation = self.send('run', env=dict(env, SUGARBOX_WAIT_GO='1'))
        self.active = EngineLease(self, generation)
        return self.active

    def run_stage(self, runner, argv, log_path):
        if self.active is not None:
            raise RuntimeError('cannot start two stages')
        previous_log = self.stage_log
        self.stage_log = Path(log_path).with_suffix('.engine.log')
        started = time.monotonic()
        code = None
        try:
            with open(log_path, 'w', encoding='utf-8', buffering=1) as output, \
                    redirect_stdout(Tee(sys.stdout, output)):
                code = runner.main(['--host', self.host] + list(argv))
                return code
        finally:
            if self.active is not None:
                self.abort()
            self.metric('stage', started, target=argv[argv.index('--target')+1],
                        log=str(log_path), exit_code=code)
            self.stage_log = previous_log

    def abort(self):
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=5)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=5)
        self.active = None

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.process.is_alive() and self.active is None:
                self.send('close')
                self.process.join(timeout=3)
        finally:
            self.abort()
            self.connection.close()
            if self.receiver is not None:
                self.receiver.close()


@contextmanager
def resident_session(runner, runtime):
    if getattr(runner, '_MISSION_RUNTIME', None) is not None:
        raise RuntimeError('mission runtime already installed')
    runner._MISSION_RUNTIME = runtime
    try:
        yield runtime
    finally:
        runner._MISSION_RUNTIME = None
        runtime.close()
