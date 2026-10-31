"""Read-only/fake-only reproductions. No serial, camera, ROS, or motor connection."""
import sys,io,contextlib,threading,time,ast,json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'integration'));sys.path.insert(0,str(ROOT/'ui'))
import mission_pipeline as mp, feedback_odom as fo, ros_io as rio, nav_rl as nr
import map_goal_provider as mgp, mission_fsm as mf, vision_grasp_pipeline as vgp
import server as ui, vision_grasp_bridge as bridge, nav_rl_grasp_pipeline as ng
results={}
class Cached:
 def get_motion_data(self): return (0.,0.,0.)
r=fo.FeedbackOdomReader(Cached())
for t in [100,100.1,100.2,100.3,105]:
 s=r.poll(now=t)
s=r.poll(now=105.1);s=r.poll(now=105.2);s=r.poll(now=105.3)
results['cached_feedback_accepted']={'fresh':s.fresh,'valid':s.valid,'stationary':s.stationary,'stamp':s.stamp}
p=mp.OdomPublisher(fo.FeedbackOdomReader(Cached()),rio.NullRosIO())
p.start();p.stop()
try:p.join(timeout=1);results['odom_join']='unexpected success'
except Exception as e:results['odom_join']=repr(e)
runner=mp.MissionRunner.__new__(mp.MissionRunner);runner.started=False
with contextlib.redirect_stdout(io.StringIO()),patch('builtins.input',side_effect=EOFError):runner._await_operator()
results['eof_starts_patrol']=runner.started
nav=vgp.Navigator.__new__(vgp.Navigator);nav.open_cameras=lambda:None;nav._detect_arm=lambda:(False,-1.,0.,0.,None)
with patch.object(vgp.time,'sleep',lambda _:None),contextlib.redirect_stdout(io.StringIO()):results['five_unavailable_frames_verify_success']=nav.verify_grasp([.24,0,.02])
console=ui.Console(SimpleNamespace(allow_real=True,simulate=False,status_port=8099))
parsers={'A':mp.parse_args,'B':bridge.parse_args,'C':ng.parse_args}
for mode in 'ABC':
 for real in [False,True]:
  cmd=console.build_argv({'mode':mode,'real':real,'unlock':True})['argv'];stderr=io.StringIO()
  with patch.object(sys,'argv',cmd[1:]),contextlib.redirect_stderr(stderr):
   try:parsers[mode]();code=0
   except SystemExit as e:code=e.code
  results[f'ui_{mode}_{"real" if real else "dry"}']={'exit':code,'error':stderr.getvalue().splitlines()[-1:]}
route=mgp._toy_route();route.bin_center=(.2,0.);route.bin_approach=(.5,0.,3.141592653589793);g=mgp.MapGoalProvider(route);g.set_bin_target();g.set_pose(mgp.MapPose(.5,0.,0.,100.))
results['opposite_bin_heading_counts_arrived']=g.arrived(now=100.)
f=mf.MissionFSM();f.state=mf.State.PLACE_ALIGN;f.entered_at=100
tr=f.step(mf.Sense(now=100.1,stationary=True))
results['place_align_without_heading']=tr.action.value
msg={'header':{'frame_id':'odom','stamp':{'secs':1,'nsecs':0}},'pose':{'pose':{'position':{'x':1,'y':2},'orientation':{'x':0,'y':0,'z':0,'w':0}},'covariance':[-1]*36}}
fake=rio.RosBridgeIO(roslibpy_module=rio._FakeRoslibpy());fake._on_amcl(msg)
results['invalid_amcl_frame_quaternion_negative_covariance']=fake.pose_quality_ok();fake.close()
msg={'header':{'frame_id':'laser','stamp':{'secs':1,'nsecs':0}},'angle_min':-3.14,'angle_increment':.01,'range_min':.05,'range_max':12.,'ranges':[float('nan')]*629}
pts=nr.laser_scan_to_points(msg);cfg=nr.NavRLConfig()
results['all_nan_scan']={'points':len(pts),'front':str(nr.front_min_raw(pts,cfg)),'ray_min':float(nr.scan_to_rays(pts,cfg).min())}
results['fine_alignment_speed']={'requested_near_mps':vgp.ARM_NEAR_VX_MPS,'actual_mps':vgp.action_to_vxyz('forward',vgp.choose_arm_forward_speed('ARM_NEAR'))[0],'requested_yaw_cap':vgp.ARM_MAX_WZ,'minimum_turn_radps':vgp.action_to_vxyz('turn_left',vgp.ARM_MIN_TURN_SPEED)[2]}
results['c3_reachable_target_negative_camera_distance']={'base_x_m':.24,'camera_x_m':.2762,'camera_distance_m':.24-.2762,'distance_state':vgp.get_arm_distance_state(.24-.2762)}
frozen=mp.OdomPublisher(fo.FeedbackOdomReader(Cached()),rio.NullRosIO());frozen._state=s
with patch.object(mp.time,'time',return_value=10000.):
 results['frozen_snapshot_still_fresh']={'fresh':frozen.state().fresh,'stamp':frozen.state().stamp,'now':10000.}
print(json.dumps(results,ensure_ascii=False,indent=2))
(ROOT/'docs/planning/codebase_audit_2026-09-06/evidence/reproductions.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
