from pathlib import Path
import sys,json,contextlib,io
root=Path.cwd();sys.path.insert(0,str(root/'integration'))
import mission_pipeline as mp, vision_grasp_pipeline as vgp
from unittest.mock import patch
out={}
g=vgp._load_grasp_module();m,v=mp.resolve_grasp_model()
cfg=g.DeployConfig(model_path=m,vecnorm_path=v)
try:
 c=g.GraspController(cfg,real_servo=False,use_socket=False,obj_provider=lambda:([.24,0,.0325],.065))
 out['mission_constructor']='success; missing optional height is NOT a constructor bug';c.close()
except Exception as e:out['mission_constructor']=repr(e)
class Part:
 def __init__(self):self.closed=False
 def close(self):self.closed=True
 def _close_device(self):self.closed=True
c=g.GraspController.__new__(g.GraspController);c.detection=None;c.fk=Part();c.servo=Part();c.close()
out['controller_close']={'fk_closed':c.fk.closed,'servo_closed':c.servo.closed}
print(json.dumps(out,indent=2))
(root/'docs/planning/codebase_audit_2026-09-06/evidence/controller_reproductions.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
