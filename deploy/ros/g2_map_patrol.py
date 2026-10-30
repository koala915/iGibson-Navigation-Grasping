#!/usr/bin/env python
"""Supervised fixed-scene AMCL patrol; one resident TCP owner, no Serial access."""
from __future__ import division
import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time

ROOT=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0,os.path.join(ROOT,'integration'))
from g2_patrol_policy import Policy,wrap,finite,guided_heading,stop_reserve
from g2_geometry import LIDAR_FORWARD_M
from g2_navigation_goals import resolve_destination,nearest_route_entry,patrol_goals
from g2_boundary_recovery import RecoveryPolicy
from g2_patrol_entry import EntryGeometry,MAX_POSITION_SIGMA,MAX_TURN_TRANSLATION_M,uncertainty_allowed
import g2_task_recovery as task_recovery


class Transient(RuntimeError):
    pass


class BoundaryRequired(RuntimeError):
    pass


class LocalizationWorkerStalled(RuntimeError):
    pass


def monotonic_seconds():
    if hasattr(time,'monotonic'):return time.monotonic()
    with open('/proc/uptime') as f:return float(f.read().split()[0])


def rpc(command,path='/tmp/grasp_health.sock'):
    s=socket.socket(socket.AF_UNIX);s.settimeout(2)
    try:
        s.connect(path);s.sendall((command+'\n').encode('ascii'))
        return json.loads(s.makefile('r').readline())
    finally:
        s.close()


def available_mb():
    with open('/proc/meminfo') as f:
        return next(float(x.split()[1])/1024 for x in f if x.startswith('MemAvailable:'))


def checkpoint(report):
    path=report.get('output_path')
    if path:
        with open(path+'.tmp','w') as f:f.write(json.dumps(report))
        os.rename(path+'.tmp',path)


def front_distance(points):
    hits=[]
    for angle,d in points:
        a=angle+math.pi;x=LIDAR_FORWARD_M+d*math.cos(a);y=d*math.sin(a)
        if x>.20 and abs(y)<=.18:
            if x<.32:
                return x
            hits.append((math.atan2(y,x),x,y))
    hits.sort();best=float('inf');cluster=[]
    for p in hits:
        if cluster and (p[0]-cluster[-1][0]>math.radians(1) or
                        math.hypot(p[1]-cluster[-1][1],p[2]-cluster[-1][2])>.04):
            if len(cluster)>=3:best=min(best,min(q[1] for q in cluster))
            cluster=[]
        cluster.append(p)
    if len(cluster)>=3:best=min(best,min(q[1] for q in cluster))
    return best


class Sensors:
    def __init__(self):
        import rospy,tf
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_srvs.srv import Empty
        self.rospy=rospy;self.tf=tf;self.listener=tf.TransformListener()
        self.lock=threading.Lock();self.scan=None;self.odom=None;self.amcl=None
        def scan(m):
            points=[(m.angle_min+i*m.angle_increment,float(v)) for i,v in enumerate(m.ranges)
                    if not math.isnan(v) and not math.isinf(v) and m.range_min<=v<=m.range_max]
            with self.lock:self.scan=(time.time(),m.header.stamp.to_sec(),m.header.frame_id,points)
        def odom(m):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            with self.lock:self.odom=(time.time(),m.header.stamp.to_sec(),m.header.frame_id,m.child_frame_id,
                [p.x,p.y,tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]],m.twist.twist.linear.x,m.twist.twist.angular.z)
        def amcl(m):
            with self.lock:self.amcl=(time.time(),m.header.stamp.to_sec(),m.header.frame_id,list(m.pose.covariance))
        self.subs=[rospy.Subscriber('/scan',LaserScan,scan,queue_size=1,tcp_nodelay=True),
                   rospy.Subscriber('/odom_setmotor',Odometry,odom,queue_size=1,tcp_nodelay=True),
                   rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,amcl,queue_size=1,tcp_nodelay=True)]
        self.nomotion=rospy.ServiceProxy('/request_nomotion_update',Empty)
        self.nomotion_started=None
        def update(_):
            self.nomotion_started=monotonic_seconds()
            try:self.nomotion()
            except rospy.ServiceException:pass
            finally:self.nomotion_started=None
        self.timer=rospy.Timer(rospy.Duration(.5),update)

    def state(self):
        requested=getattr(self,'nomotion_started',None)
        if requested is not None and monotonic_seconds()-requested>2:
            raise LocalizationWorkerStalled('AMCL no-motion update worker stuck; supervisor restart required')
        with self.lock:a,o,p=self.scan,self.odom,self.amcl
        now=time.time()
        if a is None or o is None or p is None:raise Transient('missing ROS data')
        if a[2]!='laser' or o[2:4]!=('odom','base_footprint') or p[2]!='map':raise RuntimeError('wrong frames')
        if now-a[0]>.35 or now-o[0]>.20 or not 0<=now-a[1]<=.4 or not 0<=now-o[1]<=.4:raise Transient('stale scan/odom')
        if now-p[0]>2 or not 0<=now-p[1]<=2:raise Transient('stale AMCL')
        if not a[3]:raise Transient('empty scan')
        cov=p[3]
        if not all(finite(v) for v in (cov[0],cov[7],cov[35])):
            raise RuntimeError('invalid covariance')
        if min(cov[0],cov[7],cov[35])<0:raise Transient('negative covariance sample: '+repr([cov[0],cov[7],cov[35]]))
        if max(cov[0],cov[7])>.15**2 or cov[35]>math.radians(10)**2:raise Transient('AMCL uncertainty')
        try:
            stamp=self.listener.getLatestCommonTime('map','odom').to_sec()
            t,q=self.listener.lookupTransform('map','odom',self.rospy.Time(0))
        except self.tf.Exception as exc:raise Transient('map TF: '+str(exc))
        if not -.6<=now-stamp<=1:raise Transient('stale map TF')
        yaw=self.tf.transformations.euler_from_quaternion(q)[2];c,s=math.cos(yaw),math.sin(yaw)
        pose=[t[0]+c*o[4][0]-s*o[4][1],t[1]+s*o[4][0]+c*o[4][1],wrap(yaw+o[4][2])]
        if not all(finite(v) for v in pose+o[4]+[o[5],o[6]]):raise RuntimeError('nonfinite pose/twist')
        return {'pose':pose,'odom':o[4],'odom_stamp':o[1],'vx':o[5],'wz':o[6],
                'front':front_distance(a[3]),'scan_age':now-a[1],
                'cov':[cov[0],cov[7],cov[35]],'tf_age':now-stamp}


class Runner:
    def __init__(self,sensors,policy,report,seconds):
        self.sensors=sensors;self.policy=policy;self.report=report
        self.end=time.time()+seconds;self.sock=None;self.last_health=0;self.pid=rpc('health')['service_pid']

    def stop(self):
        if self.sock is not None:
            for _ in range(3):
                try:self.sock.sendall(b'{"action":"stop"}\n')
                except socket.error:break

    def mask_rotation_allowed(self,pose):
        return all(self.policy.allowed(pose[0]+.06*math.cos(i*math.pi/8),
                                       pose[1]+.06*math.sin(i*math.pi/8)) for i in range(16))

    def rotation_allowed(self,pose):
        if getattr(self.policy,'compact',False):return self.policy.compact_turn_allowed(pose)
        if self.mask_rotation_allowed(pose):return True
        if not self.policy.allowed(*pose[:2]):return False
        try:return EntryGeometry(self.policy).box_turn_allowed(pose)
        except (ValueError,KeyError):return False

    def enter_turn_reserve(self):
        """Join the real route; reverse is authorized only inside this startup scope."""
        self._entry_reverse_used=0.
        self._entry_active=True
        try:
            for _ in range(12):
                state=self.ready();pose=state['pose']
                if self.rotation_allowed(pose):return
                entry=nearest_route_entry(self.policy,pose[:2])
                path=entry['path']
                target=next((p for p in path[1:] if math.hypot(p[0]-pose[0],p[1]-pose[1])>.01),entry['point'])
                self.report['startup_route_entry']=entry
                toward=(target[0]-pose[0])*math.cos(pose[2])+(target[1]-pose[1])*math.sin(pose[2])
                geometry=EntryGeometry(self.policy)
                if (toward>0 and self.policy.stopping_path_allowed(pose,.088)
                        and not geometry.translation_problem(pose,.088)):
                    self.pulse(.08,0,.08)
                elif self._entry_reverse_used+.088<=.10:
                    problem=EntryGeometry(self.policy).reverse_problem(pose,.088)
                    if problem:raise RuntimeError(problem)
                    self.pulse(-.08,0,.08)
                else:raise RuntimeError('cannot enter turn reserve toward route entry')
            raise RuntimeError('entry turn reserve not reached')
        finally:
            self.stop();self._entry_active=False

    def state(self,check_vision=True):
        vision_path = getattr(self, 'vision_permit', None)
        if check_vision and vision_path:
            try:
                with open(vision_path) as f: permit = json.load(f)
                if (permit.get('owner_pid') != self.pid or
                        permit.get('boot_id') != self.vision_boot):
                    raise RuntimeError('vision observer context changed')
                age = time.time() - permit['source_stamp']
                if not 0 <= age <= 1.2:
                    raise Transient('vision observer source stale')
                required=getattr(self,'_task_observation_generation',0)
                if getattr(self,'_task_observation_pending',False):
                    if permit.get('recovery_ack')!=required:raise Transient('mission recovery pause')
                    self._task_observation_pending=False
                self.report['last_frame']=permit.get('frame')
                if permit.get('allow') is not True:
                    if permit.get('pause') is True:
                        raise Transient('mission recovery pause')
                    raise RuntimeError('vision candidate requires stopped confirmation')
            except (IOError, ValueError, KeyError, TypeError):
                raise Transient('vision observer unavailable')
        if time.time()>=self.end:raise RuntimeError('patrol duration limit')
        try:
            with open('/dev/shm/g2_localization_health.json') as f:loc=json.load(f)
        except (IOError,ValueError) as exc:raise Transient('localization supervisor unavailable: '+str(exc))
        if hasattr(self,'report'):
            self.report['last_localization']={key:loc.get(key) for key in
                ('ready','status','reason_code','next_action','wait_elapsed_s','retry_after_s')}
        if getattr(self,'task_recovery_enabled',False):
            context=(loc.get('boot_id'),loc.get('owner_pid'),loc.get('map_hash'))
            if not all(value is not None for value in context):raise RuntimeError('localization context missing')
            if getattr(self,'_task_context',context)!=context:raise RuntimeError('boot/owner/map changed')
            self._task_context=context
        if loc.get('owner_pid')!=self.pid or not loc.get('ready') or not 0<=time.time()-loc['stamp']<=.4:
            detail='localization supervisor: '+str(loc.get('reason',loc.get('status')))
            if loc.get('degraded') and 0<=time.time()-loc['stamp']<=.4:
                raise RuntimeError(detail+' ['+str(loc.get('reason_code'))+'; '+str(loc.get('next_action'))+']')
            raise Transient(detail)
        memory=available_mb();self.report['min_available_mb']=min(memory,self.report.get('min_available_mb',memory))
        if memory<768:raise RuntimeError('RAM pressure')
        if time.time()-self.last_health>1:
            h=rpc('health');ch=h['chassis'];self.last_health=time.time()
            if h['service_pid']!=self.pid or ch['arm_pose']!='travel' or ch['arm_busy'] or ch['board_rx_age_s']>.2:
                raise RuntimeError('resident feedback/arm/owner fault')
        s=self.sensors.state()
        # The tight budgets belong to the physical clearance near a wall or the box.
        # 2026-10-29: applied everywhere, sigma 3-5 cm just after a relocalization
        # kept the patrol from starting in open floor. Inside the mask's 6 cm
        # turn ring the supervisor's and Sensors' own limits stand, as before.
        if not self.mask_rotation_allowed(s['pose']):
            if getattr(self.policy,'compact',False) and not uncertainty_allowed(s):
                raise Transient('compact patrol localization uncertainty exceeds budget')
            if getattr(self,'boundary_recovery',False) and max(s['cov'][:2])>MAX_POSITION_SIGMA**2:
                raise Transient('map geometry localization uncertainty exceeds budget')
        if not self.policy.allowed(*s['pose'][:2]) and not getattr(self,'_boundary_recovery_active',False):
            raise BoundaryRequired('current centre outside navigation mask')
        if getattr(self,'task_recovery_enabled',False):
            self._task_last_good=dict(pose=s['pose'],odom=s['odom'],stamp=s['odom_stamp'],context=self._task_context)
        return s

    def pause_task(self,episode,reason,elapsed=0.):
        episode.pause('LOCALIZATION_NOT_READY',elapsed)
        # Before the first pulse nothing has moved, so nothing went unobserved: ROS
        # data still arriving at startup is not an outage.
        if (getattr(self,'vision_permit',None) and not getattr(self,'_task_observation_pending',False)
                and getattr(self,'_task_moved',False)):
            self._task_observation_generation=getattr(self,'_task_observation_generation',0)+1
            self._task_observation_pending=True
        record=self.report.setdefault('task_checkpoint',{})
        record.update(status='WAIT_LOCALIZATION',reason=reason,
            recovery_generation=getattr(self,'_task_observation_generation',0),
            requires_observation=getattr(self,'_task_observation_pending',False),
            last_frame=self.report.get('last_frame'),stopped=getattr(self,'_task_last_good',None),
            wait_used_s=episode.elapsed())
        checkpoint(self.report)

    def boundary_read(self,check_vision=True):
        """Read fresh state outside the SOFT mask, retaining every other gate."""
        before=getattr(self,'_boundary_recovery_active',False)
        self._boundary_recovery_active=True
        try:return self.state(check_vision=check_vision)
        finally:self._boundary_recovery_active=before

    def recover_boundary(self):
        """Bounded map-checked escape, then return to the normal patrol gates."""
        self.stop()
        if not getattr(self,'boundary_recovery',False):
            raise BoundaryRequired('boundary recovery disabled for this scene')
        recovery=RecoveryPolicy(self.policy)
        before=getattr(self,'_boundary_recovery_active',False);self._boundary_recovery_active=True
        try:initial=self.settle()
        finally:self._boundary_recovery_active=before
        target=recovery.target(initial['pose'])
        record={'start':initial['pose'],'target':target,'trace':[],'ok':False}
        self.report.setdefault('boundary_escapes',[]).append(record)
        started=time.time();travelled=0.;last=initial['odom'][:2];stalls=0
        before=getattr(self,'_boundary_recovery_active',False);self._boundary_recovery_active=True
        try:
            for _ in range(100):
                state=self.state();pose=state['pose']
                # A ready flag alone allows much larger uncertainty in ordinary
                # patrol. Escape specifically reserves at most 4cm (3 sigma).
                if max(state['cov'][:2])>MAX_POSITION_SIGMA**2:
                    raise RuntimeError('boundary escape localization uncertainty exceeds budget')
                if recovery.recovered(pose):
                    self.settle();final=self.state()
                    if recovery.recovered(final['pose']):
                        record.update(ok=True,final=final['pose']);return
                    continue
                if time.time()-started>45 or travelled>.65:
                    raise RuntimeError('boundary escape distance/time budget exhausted')
                vx,wz=recovery.command(pose,target,state['vx'])
                pulse_start=time.time();origin=state['odom'][:2]
                try:
                    while time.time()-pulse_start<.08:
                        fresh=self.state()
                        drift=math.hypot(fresh['odom'][0]-origin[0],fresh['odom'][1]-origin[1])
                        if drift>(.03 if wz else .06):
                            raise RuntimeError('boundary escape pulse drift exceeded')
                        command=recovery.command(fresh['pose'],target,fresh['vx'] if wz else 0.)
                        if command!=(vx,wz):break
                        if abs(wrap(fresh['odom'][2]-state['odom'][2]))>.20:
                            raise RuntimeError('boundary escape pulse angular limit')
                        self.sock.sendall((json.dumps({'action':'velocity','vx':vx,'wz':wz})+'\n').encode('ascii'))
                        time.sleep(.02)
                finally:self.stop()
                stopped=self.settle();position=stopped['odom'][:2]
                distance=math.hypot(position[0]-last[0],position[1]-last[1]);travelled+=distance;last=position
                if not recovery.hard_allowed(stopped['pose']):
                    raise RuntimeError('boundary escape physical clearance lost')
                stalls=stalls+1 if vx and distance<.002 else 0
                if stalls>=5:raise RuntimeError('boundary escape made no progress')
                record['trace'].append({'pose':stopped['pose'],'vx':vx,'wz':wz,'travelled_m':travelled})
                checkpoint(self.report)
            raise RuntimeError('boundary escape iteration budget exhausted')
        finally:
            self.stop();self._boundary_recovery_active=before;checkpoint(self.report)

    def ready(self,seconds=30,allow_boundary=False,check_vision=True):
        end=time.time()+seconds;stable=0;last=None;wait_started=None;first=None
        episode=getattr(self,'_task_episode',None)
        if episode is None:
            episode=task_recovery.Episode();self._task_episode=episode
        if episode.started is not None:last=-1
        anchor=getattr(self,'_task_last_good',None)
        waiting_before=episode.elapsed()
        while True:
            try:
                if check_vision:state=self.boundary_read() if allow_boundary else self.state()
                else:state=self.boundary_read(check_vision=False) if allow_boundary else self.state(check_vision=False)
                if stable==0 and last is None:return state
                episode.check()
                if getattr(self,'task_recovery_enabled',False):
                    task_recovery.continuity(anchor,dict(pose=state['pose'],odom=state['odom'],
                        stamp=state['odom_stamp'],context=self._task_context))
                if getattr(self,'task_recovery_enabled',False):
                    if state['odom_stamp'] <= (anchor or {}).get('stamp',float('-inf')):
                        time.sleep(.04);continue
                    if last is not None and state['odom_stamp']<last and last!=-1:
                        raise RuntimeError('odometry clock reversed during recovery')
                    if abs(state['vx'])>=.01 or abs(state['wz'])>=.02:
                        stable=0;first=None;time.sleep(.04);continue
                    if first is not None and (math.hypot(state['pose'][0]-first[0],state['pose'][1]-first[1])>.03
                            or abs(wrap(state['pose'][2]-first[2]))>math.radians(3)):
                        stable=0;first=None
                    first=first or state['pose']
                if state['odom_stamp']!=last:
                    last=state['odom_stamp'];stable+=1
                if stable>=5:
                    local_wait=max(0.,task_recovery.monotonic()-episode.started) if episode.started is not None else 0.
                    episode.resume()
                    self._task_paused_total=getattr(self,'_task_paused_total',0.)+local_wait
                    self.end+=max(0.,episode.elapsed()-waiting_before)
                    self.recovery_generation=getattr(self,'recovery_generation',0)+1
                    if hasattr(self,'report'):
                        self.report['last_sensor_wait_elapsed_s']=self.report.get('sensor_wait_elapsed_s',0.)
                        self.report['sensor_wait_elapsed_s']=0.
                    return state
            except BoundaryRequired:
                if not getattr(self,'boundary_recovery',False) or self.sock is None:raise
                self.recover_boundary();stable=0;last=-1;end=time.time()+seconds
            except Transient as exc:
                stable=0;last=-1
                if wait_started is None:wait_started=time.time()
                if hasattr(self,'report'):
                    self.report['sensor_wait_elapsed_s']=max(0.,time.time()-wait_started)
                    self.report['sensor_wait_reason']=str(exc)
                self.stop()
                elapsed=self.report.get('last_localization',{}).get('wait_elapsed_s',0.) or 0.
                self.pause_task(episode,str(exc),elapsed)
                if time.time()>=end:raise Transient('sensor recovery timeout: '+str(exc))
            if time.time()>=end:raise Transient('sensor recovery stability timeout')
            time.sleep(.04)

    def settle(self):
        self.stop();start=time.time();last=None;stable=0;fault=None
        while time.time()-start<3:
            # No velocity is authorized here. A camera heartbeat outage must
            # not hide fresh zero-velocity feedback or spend the stop deadline
            # waiting for permission to move. The next ready()/pulse() still
            # requires the normal vision gate before any nonzero output.
            try:
                s=(self.boundary_read(check_vision=False) if getattr(self,'boundary_recovery',False)
                   else self.state(check_vision=False))
            except Transient as exc:
                fault=str(exc);stable=0
                episode=getattr(self,'_task_episode',None)
                if episode is None:
                    episode=task_recovery.Episode();self._task_episode=episode
                elapsed=self.report.get('last_localization',{}).get('wait_elapsed_s',0.) or 0.
                self.pause_task(episode,str(exc),elapsed)
                waited=task_recovery.monotonic()
                self.ready(allow_boundary=getattr(self,'boundary_recovery',False),check_vision=False)
                start+=task_recovery.monotonic()-waited
                continue
            if s['odom_stamp']!=last:
                last=s['odom_stamp'];stable=stable+1 if abs(s['vx'])<.01 and abs(s['wz'])<.02 else 0
            if stable>=3 and time.time()-start>=.45:return s
            time.sleep(.03)
        raise Transient('stop feedback timeout'+(': '+fault if fault else ''))

    def pulse(self,vx,wz,duration):
        if vx<0 and (not getattr(self,'_entry_active',False) or vx<-.08 or wz or duration>.08):
            raise RuntimeError('reverse is limited to startup micro-pulses')
        initial=self.ready();start=time.time();self.report['pulses']+=1
        self._task_moved=True
        reverse_origin=initial['odom'][:2] if vx<0 else None
        stopped=False
        try:
            if vx and duration>getattr(self,'max_forward_duration',.6):raise RuntimeError('forward pulse duration limit')
            while time.time()-start<duration:
                s=self.state();pose=s['pose']
                if math.hypot(s['odom'][0]-initial['odom'][0],s['odom'][1]-initial['odom'][1])>.35*duration+.12:
                    raise RuntimeError('pulse translation limit')
                if vx<0:
                    travelled=math.hypot(s['odom'][0]-reverse_origin[0],s['odom'][1]-reverse_origin[1])
                    distance=.35*max(0,duration-(time.time()-start))+stop_reserve(duration)
                    if getattr(self,'_entry_reverse_used',0.)+travelled+distance>.10:
                        raise RuntimeError('startup reverse total distance limit')
                    if not uncertainty_allowed(s,directional=True):raise Transient('entry geometry localization uncertainty')
                    problem=EntryGeometry(self.policy).reverse_problem(pose,distance)
                    if problem:raise RuntimeError(problem)
                    # Retain the ordinary front/person brake even while reversing.
                    if s['front']<getattr(self,'front_brake_m',.5):raise Transient('front brake')
                elif vx:
                    brake=getattr(self,'front_brake_m',.5)
                    if getattr(self,'boundary_micro',False) and duration<=.08:brake=min(brake,.35)
                    # The laser brake stays in every patrol pulse: the operator
                    # stands in the room during a mission (koala review,
                    # 2026-10-22), and only the laser sees a person. The map
                    # geometry below is an added check, never a replacement.
                    if s['front']<brake:
                        raise Transient('front brake')
                    distance=.35*max(0,duration-(time.time()-start))+stop_reserve(duration)
                    if not self.policy.stopping_path_allowed(pose,distance):raise Transient('predicted stop crosses mask')
                    if getattr(self,'_entry_active',False) or getattr(self.policy,'compact',False):
                        geometry=EntryGeometry(self.policy)
                        end=(pose[0]+distance*math.cos(pose[2]),pose[1]+distance*math.sin(pose[2]))
                        circle_safe=geometry.segment_allowed(pose,end)
                        if not circle_safe:
                            if not uncertainty_allowed(s,directional=True):raise Transient('entry geometry localization uncertainty')
                            problem=geometry.translation_problem(pose,distance)
                            if problem:raise RuntimeError(problem)
                    if getattr(self,'boundary_recovery',False):
                        geometry=RecoveryPolicy(self.policy)
                        end=(pose[0]+distance*math.cos(pose[2]),pose[1]+distance*math.sin(pose[2]))
                        if not geometry.segment(pose,end):
                            if not uncertainty_allowed(s) or not EntryGeometry(self.policy).segment_allowed(pose,end):
                                raise Transient('predicted stop violates physical wall/bin geometry')
                else:
                    if not self.rotation_allowed(pose):
                        if getattr(self,'boundary_recovery',False):
                            self.stop();self.recover_boundary()
                            raise Transient('replan required after boundary escape')
                        raise RuntimeError('turn drift reserve outside mask')
                    if not self.mask_rotation_allowed(pose):
                        if not uncertainty_allowed(s):raise Transient('entry geometry localization uncertainty')
                        if math.hypot(s['odom'][0]-initial['odom'][0],s['odom'][1]-initial['odom'][1])>MAX_TURN_TRANSLATION_M:
                            raise RuntimeError('physical turn translation reserve exceeded')
                    if abs(wrap(s['odom'][2]-initial['odom'][2]))>math.radians(20):raise RuntimeError('pulse angular limit')
                self.sock.sendall((json.dumps({'action':'velocity','vx':vx,'wz':wz})+'\n').encode('ascii'))
                time.sleep(.04)
        except Transient as exc:
            self.stop()
            stopped=True
            episode=getattr(self,'_task_episode',None)
            if episode is None:
                episode=task_recovery.Episode();self._task_episode=episode
            self.pause_task(episode,str(exc),self.report.get('last_localization',{}).get('wait_elapsed_s',0.) or 0.)
            raise
        except BoundaryRequired:
            self.stop()
            if not getattr(self,'boundary_recovery',False):raise
            self.recover_boundary()
            raise Transient('replan required after boundary escape')
        finally:
            if not stopped:self.stop()
        final=self.settle();self.report['trace'].append(final);checkpoint(self.report)
        if not vx and not self.mask_rotation_allowed(initial['pose']):
            if (math.hypot(final['odom'][0]-initial['odom'][0],final['odom'][1]-initial['odom'][1])>MAX_TURN_TRANSLATION_M
                    or not self.rotation_allowed(final['pose'])):
                raise RuntimeError('physical turn stopped outside reserve')
        if vx<0:
            self._entry_reverse_used+=math.hypot(final['odom'][0]-reverse_origin[0],final['odom'][1]-reverse_origin[1])
            if self._entry_reverse_used>.10:raise RuntimeError('startup reverse total distance limit')
        if getattr(self,'boundary_recovery',False) and not self.policy.allowed(*final['pose'][:2]):
            self.recover_boundary()
            raise Transient('replan required after boundary escape at stop')

    def navigate(self,goal,label):
        self.report['active_goal']={'label':label,'point':goal};checkpoint(self.report)
        current=self.ready()['pose'][:2]
        origin=self.report.get('last_planned_goal',current)
        if math.hypot(origin[0]-current[0],origin[1]-current[1])>.13:
            origin=current
        begun=time.time();last_distance=None;stalled=0;generation=getattr(self,'recovery_generation',0)
        paused_before=getattr(self,'_task_paused_total',0.)
        for _ in range(160):
            s=self.ready();p=s['pose'];distance=math.hypot(goal[0]-p[0],goal[1]-p[1])
            if generation!=getattr(self,'recovery_generation',0):
                generation=self.recovery_generation;origin=p[:2];last_distance=None;stalled=0
                if not self.policy.segment_allowed(origin,goal):raise Transient('replan required after localization correction')
            if distance<=.045:
                s=self.settle();error=math.hypot(goal[0]-s['pose'][0],goal[1]-s['pose'][1])
                if error<=.065:
                    self.report['last_planned_goal']=goal
                    self.report['arrivals'].append({'label':label,'goal':goal,'pose':s['pose'],'error_m':error,'cov':s['cov']});checkpoint(self.report);return
            if time.time()-begun-(getattr(self,'_task_paused_total',0.)-paused_before)>150:raise RuntimeError('waypoint timeout')
            if not self.policy.segment_allowed(p[:2],goal):raise RuntimeError('waypoint segment crosses mask')
            err=wrap(guided_heading(p,origin,goal)-p[2])
            try:
                if abs(err)>math.radians(3):
                    self.pulse(0,1 if err>0 else -1,min(.12,max(.04,(abs(err)-.02)/1.5)))
                else:
                    choices=[d for d in (.6,.35,.15,.08,.04) if d<=getattr(self,'max_forward_duration',.6) and .35*d+.025<distance and self.policy.stopping_path_allowed(p,.35*d+stop_reserve(d))]
                    if getattr(self,'boundary_micro',False) and s['front']<.5:
                        choices=[d for d in choices if d<=.08]
                    if not choices:
                        if distance<.10:choices=[.04] if self.policy.stopping_path_allowed(p,.074) else []
                        if not choices:raise RuntimeError('cannot approach goal within stop reserve')
                    self.pulse(.15,0,choices[0])
                stalled=stalled+1 if last_distance is not None and abs(distance-last_distance)<.002 and abs(err)<math.radians(3) else 0
                if stalled>=5:raise RuntimeError('no translational progress')
                last_distance=distance
            except Transient as exc:
                self.stop();self.report['recoveries'].append(str(exc))
                if len(self.report['recoveries'])>12:raise RuntimeError('recovery budget exhausted')
                self.settle();fresh=self.ready();origin=fresh['pose'][:2]
                # Localization may correct map->odom during recovery. Re-anchor
                # guidance and recheck the entire route before another output.
                if not self.policy.segment_allowed(origin,goal):
                    raise Transient('replan required after localization correction')
                last_distance=None;stalled=0
        raise RuntimeError('waypoint iteration limit')

    def goto(self,goal,label):
        current=self.ready()['pose'][:2];path=self.policy.plan(current,goal)
        self.report['task_checkpoint']=dict(stage='PATROL_OR_TRANSIT',target=list(goal),
            label=label,navigation_complete=False,observation_complete=False,
            context=getattr(self,'_task_context',None),start_odom=getattr(self,'_task_last_good',{}).get('odom'),
            remaining_path=path[1:],holding=self.report.get('holding'),arm_outcome='not_started')
        checkpoint(self.report)
        self.report.setdefault('planned_paths',[]).append({'label':label,'points':path})
        remaining=list(path[1:]);attempt=0
        while remaining:
            try:
                point=remaining[0]
                self.navigate(point,label if len(remaining)==1 else label+'_via')
                remaining.pop(0)
                self.report['task_checkpoint']['remaining_path']=list(remaining)
                checkpoint(self.report)
            except Transient as exc:
                self.stop()
                if 'replan required' not in str(exc) or attempt>=3:raise
                attempt+=1
                # Repair toward the interrupted sub-leg endpoint, NOT straight
                # to the final goal. Unobserved intermediate legs remain queued.
                current=self.ready()['pose'][:2]
                repaired=self.policy.plan(current,remaining[0])
                remaining=list(repaired[1:])+remaining[1:]
                self.report['task_checkpoint']['remaining_path']=list(remaining)
                self.report['planned_paths'].append({'label':label,'points':[current]+remaining,'reason':str(exc)})
        self.report['task_checkpoint'].update(navigation_complete=True,observation_complete=True)
        episode=getattr(self,'_task_episode',None)
        if episode is not None:episode.complete(True,True)
        checkpoint(self.report)

    def verify_parked_goal(self,goal):
        final=self.settle();self.report['final']=final
        error=math.hypot(goal[0]-final['pose'][0],goal[1]-final['pose'][1])
        self.report['final_goal_error_m']=error
        self.report['parked_goal_valid']=error<=.065
        if error>.065:raise RuntimeError('final turn displaced centre beyond goal tolerance')

    def face_interior(self):
        centre=[sum(p[i] for p in self.policy.layout['room_polygon'])/4 for i in (0,1)]
        self.face_point(centre,'room_centre')

    def face_point(self,point,label='requested_point'):
        for _ in range(50):
            s=self.ready();pose=s['pose'];error=wrap(math.atan2(point[1]-pose[1],point[0]-pose[0])-pose[2])
            if abs(error)<=math.radians(6):
                self.report['park_facing']=label;self.report['park_face_point']=point;self.report['park_yaw_error_deg']=math.degrees(error);return
            try:self.pulse(0,1 if error>0 else -1,min(.15,max(.05,(abs(error)-.03)/1.5)))
            except Transient as exc:
                self.report['recoveries'].append(str(exc));self.settle()
                if len(self.report['recoveries'])>12:raise RuntimeError('recovery budget exhausted')
        raise RuntimeError('final orientation limit')

    def face_yaw(self,yaw):
        for _ in range(50):
            error=wrap(yaw-self.ready()['pose'][2])
            if abs(error)<=math.radians(6):
                self.report.update(park_facing='requested_yaw',park_face_yaw=yaw,
                                   park_yaw_error_deg=math.degrees(error));return
            try:self.pulse(0,1 if error>0 else -1,min(.15,max(.05,(abs(error)-.03)/1.5)))
            except Transient as exc:
                self.report['recoveries'].append(str(exc));self.settle()
                if len(self.report['recoveries'])>12:raise RuntimeError('recovery budget exhausted')
        raise RuntimeError('final orientation limit')


def resolve_goals(requested,policy,points):
    if not isinstance(requested,list) or not 1<=len(requested)<=12:
        raise ValueError('goals must be a list of 1..12 entries')
    result=[]
    for i,item in enumerate(requested):
        if item in ('home','start'):goal=points[0];name='home'
        elif item=='dropoff':
            raise ValueError('dropoff approach/exit not yet validated')
        elif isinstance(item,dict):goal=item['point'];name=str(item.get('name','goal%d'%i))
        else:raise ValueError('unknown goal')
        if not isinstance(goal,(list,tuple)) or len(goal)!=2:raise ValueError('illegal goal')
        goal=[float(v) for v in goal]
        if not all(finite(v) for v in goal) or not policy.allowed(*goal):raise ValueError('illegal goal')
        result.append((name,goal))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',required=True);p.add_argument('--output',required=True)
    p.add_argument('--loops',type=int,default=1);p.add_argument('--seconds',type=int,default=600);p.add_argument('--real',action='store_true');p.add_argument('--goals',help='JSON list of home, dropoff or {name,point:[x,y]}')
    p.add_argument('--inset',type=float,default=.60,help='Legacy patrol/check inset, 0.60..0.90 m; patrol-cycle follows the scene route_file')
    p.add_argument('--face-point',type=float,nargs=2,metavar=('X','Y'),help='Face this map point after arrival; does not authorize approaching it')
    p.add_argument('--face-yaw',type=float,help='Absolute map yaw after arrival, -pi..pi; turn only in the normal turn-clear region')
    p.add_argument('--keep-heading',action='store_true',help='Keep the final travel heading when chaining navigation stages')
    p.add_argument('--boundary-micro',action='store_true',help='Use <=0.08s pulses and a 0.35m front brake below normal 0.50m clearance')
    p.add_argument('--vision-permit', help='Optional boot/owner-bound camera observer heartbeat JSON')
    p.add_argument('--forward-pulse-max', type=float, default=.6)
    p.add_argument('--destination',choices=('nearest-patrol','patrol-cycle','dropoff-preparation'))
    p.add_argument('--point',type=float,nargs=2,metavar=('X','Y'),help='Navigate to any reachable map point')
    args=p.parse_args()
    if not finite(args.forward_pulse_max) or not .04<=args.forward_pulse_max<=.6:p.error('forward-pulse-max must be .04..0.6 s')
    if not 1<=args.loops<=2 or not 30<=args.seconds<=600:p.error('bounded loops 1..2, seconds 30..600')
    if not finite(args.inset) or not .60<=args.inset<=.90:p.error('patrol inset must be 0.60..0.90 m')
    if args.face_point and not all(finite(v) for v in args.face_point):p.error('face point must be finite')
    if args.face_yaw is not None and (not finite(args.face_yaw) or not -math.pi<=args.face_yaw<=math.pi):p.error('face-yaw must be -pi..pi')
    if sum((bool(args.keep_heading),bool(args.face_point),args.face_yaw is not None))>1:p.error('choose keep-heading, face-point or face-yaw')
    if sum(bool(x) for x in (args.goals,args.destination,args.point))>1:p.error('choose goals, destination or point')
    policy=Policy(args.scene)
    # Explicit destinations/goals follow their own checked paths. A legacy
    # inset square must not reject a valid route around the moved box.
    points=([point for _,point in patrol_goals(policy)] if args.destination or args.point or args.goals
            else policy.patrol_points(args.inset))
    report={'ok':False,'real':args.real,'output_path':args.output,'waypoints':points,'patrol_inset_m':args.inset,'primary_patrol_point':'opposite paper-box corner','arrivals':[],'recoveries':[],'trace':[],'pulses':0}
    runner=None;lease=None;unlocked=False
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    try:
        if args.real:policy.require_motion_ready(LIDAR_FORWARD_M)
        goals=None
        if args.goals:
            with open(args.goals) as f:goals=resolve_goals(json.load(f),policy,points)
            report['requested_goals']=[{'name':name,'point':goal} for name,goal in goals]
        if available_mb()<1024:raise RuntimeError('RAM below 1GB at start')
        h=rpc('health');ch=h['chassis'];owner=str(h['service_pid'])
        report['holding']=h.get('holding')
        held=subprocess.check_output(['fuser','/dev/myserial']).decode().split()
        if held!=[owner] or ch['motors']!=[0,0,0,0] or ch['moving'] or ch['client'] is not None:raise RuntimeError('sole idle owner preflight failed')
        import rospy
        rospy.init_node('g2_map_patrol',disable_signals=True);sensors=Sensors();runner=Runner(sensors,policy,report,args.seconds)
        runner.boundary_micro=args.boundary_micro
        runner.task_recovery_enabled=True
        runner.max_forward_duration=args.forward_pulse_max
        runner.vision_permit=args.vision_permit
        runner.boundary_recovery=policy.layout.get('boundary_recovery',{}).get('enabled') is True
        if args.vision_permit:
            with open('/proc/sys/kernel/random/boot_id') as f:runner.vision_boot=f.read().strip()
        report['boundary_micro']=args.boundary_micro
        report['initial']=runner.ready(8,allow_boundary=runner.boundary_recovery)
        planning_start=report['initial']['pose'][:2]
        if runner.boundary_recovery and not policy.allowed(*planning_start):
            planning_start=list(RecoveryPolicy(policy).target(report['initial']['pose']))
            report['escape_preflight_target']=planning_start
        if args.destination:
            goals=resolve_destination(policy,planning_start,args.destination)
            report['destination']=args.destination
        elif args.point:
            policy.plan(planning_start,args.point)
            goals=[('requested_point',args.point)]
        if goals:
            report['requested_goals']=[{'name':name,'point':goal} for name,goal in goals]
            previous=planning_start;report['preflight_paths']=[]
            for name,goal in goals:
                report['preflight_paths'].append({'label':name,'points':policy.plan(previous,goal)})
                previous=goal
        if args.real:
            import fcntl
            lease=open('/tmp/g2_motion_client.lock','a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
            result=rpc('maintenance-unlock','/tmp/grasp_service.sock')
            if not result.get('ok'):raise RuntimeError('unlock refused')
            unlocked=True
            runner.sock=socket.create_connection(('127.0.0.1',7000),timeout=1);runner.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            if runner.boundary_recovery and not policy.allowed(*runner.boundary_read()['pose'][:2]):
                runner.recover_boundary()
            runner.enter_turn_reserve()
            if goals:
                for leg_index,(name,goal) in enumerate(goals):
                    report['remaining_route']=[{'name':n,'point':g} for n,g in goals[leg_index:]]
                    report['leg_index']=leg_index
                    runner.goto(goal,name)
                    report['remaining_route']=[{'name':n,'point':g} for n,g in goals[leg_index+1:]]
                    report['completed_leg_index']=leg_index
                    checkpoint(report)
            else:
                runner.goto(points[0],'diagonal_start')
                for loop in range(args.loops):
                    for i in (1,2,3,0):runner.goto(points[i],'loop%d_point%d'%(loop+1,i))
            if args.keep_heading or ((args.destination or args.point) and not args.face_point and args.face_yaw is None):report['park_facing']='last_leg'
            elif args.face_yaw is not None:runner.face_yaw(args.face_yaw)
            elif args.face_point:runner.face_point(args.face_point)
            else:runner.face_interior()
            runner.verify_parked_goal(goals[-1][1] if goals else points[0])
        report['ok']=True
        if args.destination=='dropoff-preparation':
            report['next_stage']='visual_dock_required'
            report['release_ready']=False
    except (Exception,KeyboardInterrupt) as exc:report['error']=str(exc) or 'interrupted'
    finally:
        if runner is not None:
            runner.stop()
            if runner.sock is not None:runner.sock.close()
        if unlocked:
            try:
                for _ in range(30):
                    h=rpc('health')
                    if h['chassis']['motors']==[0,0,0,0] and not h['chassis']['moving']:
                        result=rpc('maintenance-lock','/tmp/grasp_service.sock')
                        if result.get('ok'):break
                    time.sleep(.1)
                else:raise RuntimeError('maintenance lock timeout')
                report['final_health']=rpc('health')
            except Exception as exc:report['ok']=False;report['cleanup_error']=str(exc)
        if lease:lease.close()
        with open(args.output,'w') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:report.get(k) for k in ('ok','error','pulses','recoveries','arrivals')}));return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
