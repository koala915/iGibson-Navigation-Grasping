"""Bounded passive RAM recorder; optional ROS sampling for isolated diagnostics.

No actuator, repair or Serial access. Default RAM-only mode avoids changing
rosbridge delivery behavior while navigation clients are running.
"""
import argparse
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import signal
import subprocess
import threading
import time


class TopicStats:
    def __init__(self):
        self.count=0
        self.stamp=None
        self.received=None
        self.max_gap_s=0.

    def accept(self, stamp, now):
        if stamp==self.stamp:
            return
        if self.received is not None:
            self.max_gap_s=max(self.max_gap_s,now-self.received)
        self.count+=1
        self.stamp,self.received=stamp,now

    def snapshot(self, now):
        age=None if self.received is None else now-self.received
        return dict(count=self.count,stamp=self.stamp,arrival_age_s=age,
                    max_gap_s=max(self.max_gap_s,age or 0))


def stamp(msg):
    value=msg['header']['stamp']
    return value['secs']+value['nsecs']*1e-9


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=1800)
    parser.add_argument('--output',required=True)
    parser.add_argument('--ros-topics',action='store_true',
                        help='isolated diagnostics only; do not combine with navigation clients')
    args=parser.parse_args()
    if not 10<=args.seconds<=1800:
        parser.error('seconds must be 10..1800')
    output=Path(args.output)
    output.parent.mkdir(parents=True,exist_ok=True)
    logger=logging.getLogger('baseline')
    logger.setLevel(logging.INFO)
    handler=RotatingFileHandler(str(output),maxBytes=1024*1024,backupCount=1)
    handler.setFormatter(logging.Formatter('%(message)s'))
    logger.addHandler(handler)
    lock=threading.Lock()
    stats={name:TopicStats() for name in ('/scan_raw','/scan','/odom_setmotor','odom_tf')} if args.ros_topics else {}
    stopped=threading.Event()
    for sig in (signal.SIGTERM,signal.SIGINT):
        signal.signal(sig,lambda *_:stopped.set())
    ros=None
    if args.ros_topics:
        import roslibpy
        ros=roslibpy.Ros(host='127.0.0.1',port=9090)
        ros.run(timeout=5)
    topics=[]
    def receive(name,msg):
        if name=='/tf':
            matches=[t for t in msg.get('transforms',[]) if t.get('header',{}).get('frame_id')=='odom' and t.get('child_frame_id')=='base_footprint']
            if not matches:
                return
            name,msg='odom_tf',matches[-1]
        try:
            value=stamp(msg)
        except (KeyError,TypeError):
            return
        with lock:
            stats[name].accept(value,time.monotonic())
    for name,kind in ([('/scan_raw','sensor_msgs/LaserScan'),('/scan','sensor_msgs/LaserScan'),('/odom_setmotor','nav_msgs/Odometry'),('/tf','tf2_msgs/TFMessage')] if args.ros_topics else []):
        # Sample diagnostics rather than serializing every full scan through
        # rosbridge. Reported counts/gaps are monitor samples, not sensor Hz.
        # /tf multiplexes static and odom transforms; throttling the whole
        # topic can repeatedly select static messages and hide live odom TF.
        interval_ms=0 if name=='/tf' else (500 if name.startswith('/scan') else 200)
        topic=roslibpy.Topic(ros,name,kind,queue_length=1,throttle_rate=interval_ms)
        topics.append(topic)
        topic.subscribe(lambda msg,n=name:receive(n,msg))
    begun=time.monotonic()
    samples,min_ram,first_rss,last_rss=0,float('inf'),{},{}
    try:
        while not stopped.is_set() and time.monotonic()-begun<args.seconds:
            available=next(int(l.split()[1])/1024 for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:'))
            rss={}
            for unit in ('grasp-service','grasp-vision'):
                pid=subprocess.check_output(['systemctl','show',unit,'-p','MainPID','--value'],timeout=3).decode().strip()
                try:
                    rss[unit]=next(int(l.split()[1])/1024 for l in Path('/proc/'+pid+'/status').read_text().splitlines() if l.startswith('VmRSS:'))
                except (OSError,StopIteration):
                    rss[unit]=None
            now=time.monotonic()
            with lock:
                snapshot={n:s.snapshot(now) for n,s in stats.items()}
            row=dict(elapsed_s=now-begun,time=time.time(),available_mb=available,rss_mb=rss,topics=snapshot,
                     ros_sampling_enabled=args.ros_topics,
                     monitor_sampling_ms={'scan':500,'odom':200,'tf':0} if args.ros_topics else {})
            logger.info(json.dumps(row))
            samples+=1
            min_ram=min(min_ram,available)
            if not first_rss:
                first_rss=dict(rss)
            last_rss=rss
            stopped.wait(2)
    finally:
        with lock:
            summary=dict(elapsed_s=time.monotonic()-begun,samples=samples,
                         min_available_mb=min_ram if samples else None,
                         first_rss_mb=first_rss,last_rss_mb=last_rss,
                         topics={n:s.snapshot(time.monotonic()) for n,s in stats.items()},
                         complete=time.monotonic()-begun>=args.seconds)
        output.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        for topic in topics:
            topic.unsubscribe()
        if ros is not None:
            ros.terminate()
        handler.close()


if __name__=='__main__':
    main()
