"""Bounded repairs and checkpoints. No serial owner, model or motion replay."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time


class TransientFault(RuntimeError):
    def __init__(self, reason, unit=None):
        super().__init__(reason)
        self.unit = unit


class Recovery:
    def __init__(self, repair, clock=time.monotonic, sleep=time.sleep,
                 grace_s=30, stable_samples=3, max_repairs=2, cooldown_s=10):
        self.repair, self.clock, self.sleep = repair, clock, sleep
        self.grace_s, self.stable_samples = grace_s, stable_samples
        self.max_repairs, self.cooldown_s = max_repairs, cooldown_s
        self.attempts = {}  # bounded: three allowlisted services only
        self.last_repair = -math.inf

    def wait(self, check, report=None):
        deadline = self.clock()+self.grace_s
        good = 0
        last = None
        while True:
            try:
                result = check()
                if last is None:
                    return result
                good += 1
                if good >= self.stable_samples:
                    return result
            except TransientFault as exc:
                last, good = exc, 0
                if report:
                    report('waiting',str(exc))
                count = self.attempts.get(exc.unit,0)
                if exc.unit in ('grasp-vision','x3plus-g2-lidar','x3plus-g2-rosbridge') \
                        and count < self.max_repairs and self.clock()-self.last_repair >= self.cooldown_s:
                    self.attempts[exc.unit] = count+1
                    self.last_repair = self.clock()
                    if report:
                        report('repairing',exc.unit)
                    try:
                        self.repair(exc.unit)
                    except (OSError,RuntimeError):
                        pass  # wait for evidence; never assume the repair succeeded
            if self.clock() >= deadline:
                raise TransientFault('recovery_exhausted: %s' % last)
            self.sleep(0.5)


class Checkpoint:
    def __init__(self, path, plan, boot_id, service_pid):
        self.path = Path(path)
        self.identity = dict(plan_sha256=hashlib.sha256(
            json.dumps(plan,sort_keys=True,allow_nan=False).encode()).hexdigest(),
            boot_id=boot_id,service_pid=service_pid)

    def write(self, next_step, pose, holding, inflight=False):
        value = dict(self.identity,next_step=next_step,pose=pose,holding=holding,
                     inflight=inflight)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(value)+'\n')
        temporary.replace(self.path)

    def resume(self, steps):
        value = json.loads(self.path.read_text())
        if any(value.get(k) != v for k,v in self.identity.items()) or value.get('inflight') is not False:
            raise RuntimeError('checkpoint uncertain or service/boot/plan changed; no replay')
        index = value.get('next_step')
        pose, holding = 'travel', False
        if type(index) is not int or not 0 <= index <= len(steps):
            raise RuntimeError('invalid checkpoint step')
        for step in steps[:index]:
            action = step['action']
            if action == 'home':pose = 'e1'
            elif action == 'grasp':holding = True
            elif action == 'release':pose,holding = 'e1',False
            elif action == 'stow':pose = 'travel'
        if (value.get('pose'),value.get('holding')) != (pose,holding):
            raise RuntimeError('checkpoint state differs from completed steps')
        return index,pose,holding


def atomic_status(path, value):
    target = Path(path)
    temporary = target.with_suffix('.tmp')
    temporary.write_text(json.dumps(value)+'\n')
    temporary.replace(target)


def owner_restart_allowed(status, owner_ok, available):
    """Cached idle/empty state permits diagnostic restart, never a motion replay."""
    ch = status.get('chassis') or {}
    serial = status.get('serial') or {}
    if status.get('stack_version') != 'v23' or type(status.get('service_pid')) is not int \
            or status['service_pid'] <= 0 or not serial or serial.get('healthy') is not False:
        raise RuntimeError('new service API and an identified Serial fault are required')
    if not owner_ok or not math.isfinite(available) or available < 1024:
        raise RuntimeError('restart owner/RAM gate')
    if status.get('holding') is not False or ch.get('arm_pose') not in ('travel','restart_pending') \
            or ch.get('arm_busy') is not False or ch.get('moving') is not False \
            or ch.get('motors') != [0]*4 or ch.get('client') is not None:
        raise RuntimeError('restart requires idle empty travel; unknown action is not restarted')


class RestartEscalation:
    def __init__(self, boot_id, budget='/tmp/v23_restart_budget.json',
                 marker='/tmp/v23_restart_no_motion.json',clock=time.monotonic,delay_s=15):
        self.boot_id,self.budget,self.marker = boot_id,Path(budget),Path(marker)
        self.clock,self.delay_s = clock,delay_s
        self.first_fault = None

    def observe(self, status, restart):
        if (status.get('serial') or {}).get('healthy') is not False:
            self.first_fault = None
            return 'waiting'
        if self.first_fault is None:self.first_fault=self.clock()
        if self.clock()-self.first_fault < self.delay_s:return 'waiting'
        # Missing/corrupt persisted budget fails closed; never reset on monitor restart.
        if self.budget.exists():
            value=json.loads(self.budget.read_text())
            if not isinstance(value,dict) or 'boot_id' not in value or type(value.get('attempts')) is not int \
                    or value['attempts'] < 0:
                raise RuntimeError('restart budget corrupt')
            if value['boot_id']==self.boot_id and value['attempts']>=1:return 'restart_exhausted'
        def authorize():
            # Durable intent before issuing the restart; a timeout consumes the attempt.
            atomic_status(self.budget,dict(boot_id=self.boot_id,attempts=1))
            atomic_status(self.marker,dict(boot_id=self.boot_id,no_motion=True))
        restart(authorize)
        return 'restarting'


def unresolved_action(status, path='/tmp/v23_checkpoint.json'):
    checkpoint=Path(path)
    if not checkpoint.exists():return False
    value=json.loads(checkpoint.read_text())
    # Completed plans are harmless; unfinished work belonging to this owner is not.
    return value.get('service_pid') == status.get('service_pid') and value.get('inflight') is not False


def main():
    """Persistent passive monitor; bounded repairs only while the robot is idle."""
    import g2_safety_audit as audit
    from v23_demo import ResidentBackend, check_health
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='/tmp/v23_monitor.json')
    parser.add_argument('--restart-owner',action='store_true',
                        help='one diagnostic-only Serial owner restart per boot after 15 s persistent fault')
    args = parser.parse_args()
    backend = ResidentBackend()
    sensors = audit.Sensors()
    recovery = Recovery(backend.repair)
    escalation = RestartEscalation(Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    stable = 0
    # One monitor, no accumulating processes/threads/history.
    import fcntl
    with open('/tmp/v23_monitor.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            while True:
                sample = dict(state='diagnostics',monotonic=time.monotonic(),pid=os.getpid())
                try:
                    health = backend.health()
                    sample['serial'] = health.get('serial')
                    ch = health.get('chassis') or {}
                    if not health.get('ok'):
                        if args.restart_owner:
                            with open('/tmp/v23_demo.lock','a') as demo_lock:
                                fcntl.flock(demo_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                                sample['state']=escalation.observe(health,backend.restart_owner)
                            atomic_status(args.output,sample)
                            stable=0
                            time.sleep(1)
                            continue
                        raise RuntimeError('control transport unhealthy; diagnostic mode')
                    if ch.get('moving') or ch.get('arm_busy') or ch.get('client'):
                        # Moving tasks own their stop policy; no service repair race.
                        sample['state'] = 'task_active'
                    else:
                        check_health(health,ch.get('arm_pose'),health.get('holding'))
                        if audit.available_mb() < 1024:
                            sample['state'] = 'reduced_RAM'
                        else:
                            latest,_ = sensors.snapshot()
                            faults = audit.sensor_faults(latest,time.time(),time.monotonic())
                            unit = None
                            if any('/scan' in f for f in faults):unit='x3plus-g2-lidar'
                            elif faults:unit='x3plus-g2-rosbridge'
                            try:
                                backend.guard(ch.get('arm_pose'),health.get('holding'))
                            except TransientFault as exc:
                                unit = exc.unit or unit
                                faults.append(str(exc))
                            if faults:
                                sample.update(state='waiting',faults=faults)
                                count = recovery.attempts.get(unit,0)
                                if unit and count < recovery.max_repairs and \
                                        time.monotonic()-recovery.last_repair >= recovery.cooldown_s:
                                    with open('/tmp/v23_demo.lock','a') as demo_lock:
                                        # A running plan repairs itself. The passive monitor
                                        # must not race its next action or maintenance lock.
                                        fcntl.flock(demo_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                                        recovery.attempts[unit]=count+1
                                        recovery.last_repair=time.monotonic()
                                        backend.repair(unit)
                                    sample['state']='repairing'
                                elif unit and count >= recovery.max_repairs:
                                    sample['state']='degraded'
                            else:
                                stable += 1
                                sample['state']='ready' if stable >= 3 else 'waiting'
                except Exception as exc:
                    sample.update(state='diagnostics',reason=str(exc))
                if sample['state'] not in ('ready','waiting') or sample.get('faults'):
                    stable = 0
                atomic_status(args.output,sample)
                time.sleep(1)
        finally:
            sensors.close()


if __name__ == '__main__':
    main()
