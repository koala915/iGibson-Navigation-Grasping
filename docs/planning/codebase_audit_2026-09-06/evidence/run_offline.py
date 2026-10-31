import subprocess,sys,json,time,os
from pathlib import Path
out=Path('docs/planning/codebase_audit_2026-09-06/evidence')
cases=[[p] for p in ['grasp/v21/test_deploy_controller.py','grasp/v21/test_servo_read.py','grasp/v21/test_deploy_floor_guard.py','grasp/v21/test_one_command_launcher.py','tests/test_safety_guards.py','tests/test_mission_end_to_end.py','tests/test_model_package.py','tests/test_grasp_home_homography.py','tests/test_stream_cam_capture.py','tests/test_vision_grasp_bridge_pose.py','tests/test_trash_target.py','ui/test_server.py','grasp/v23/test_deploy_controller.py','grasp/v23/test_servo_read.py','grasp/v23/test_deploy_floor_guard.py','grasp/v23/test_one_command_launcher.py','grasp/v23/test_three_pose_scan.py']]
cases += [[p,'--selftest'] for p in ['integration/mission_fsm.py','integration/mission_pipeline.py','integration/vision_grasp_pipeline.py','integration/nav_rl.py','integration/nav_rl_grasp_pipeline.py','integration/feedback_odom.py','integration/map_goal_provider.py','integration/ros_io.py','integration/mission_status.py','integration/grasp_home_homography.py','integration/arm_cam_geometry.py','integration/solve_cam_to_base.py','integration/solve_arm_cam_extrinsics.py','grasp/v23/pose_explorer.py','ui/launcher.py']]
results=[]
env=dict(os.environ,PYTHONIOENCODING='utf-8',PYTHONDONTWRITEBYTECODE='1')
for i,args in enumerate(cases):
 start=time.monotonic()
 try:
  r=subprocess.run([sys.executable,*args],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=180,env=env)
  code=r.returncode; output=r.stdout.decode('utf-8',errors='replace')
 except subprocess.TimeoutExpired as e:
  code='TIMEOUT';output=(e.stdout or b'').decode('utf-8',errors='replace')
 log=f'{i+1:02d}_{Path(args[0]).parent.name}_{Path(args[0]).stem}.txt';(out/log).write_text(output,encoding='utf-8')
 row=dict(command=[sys.executable,*args],exit_code=code,seconds=round(time.monotonic()-start,2),log=log)
 results.append(row);(out/'offline_test_results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
 print(json.dumps(row),flush=True)
