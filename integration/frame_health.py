"""Atomic per-inference heartbeat for the deployed vision compatibility patch."""
import json
import math
import os
from pathlib import Path
import time


def write_frame_health(path,shape,inference_s):
    if not path:
        return
    if len(shape)<2 or shape[0]<=0 or shape[1]<=0 or not math.isfinite(inference_s) or inference_s<0:
        raise ValueError('invalid frame/inference heartbeat')
    target=Path(path)
    temporary=target.with_name(target.name+'.%d.tmp'%os.getpid())
    temporary.write_text(json.dumps(dict(pid=os.getpid(),monotonic=time.monotonic(),
                                         height=int(shape[0]),width=int(shape[1]),
                                         inference_s=float(inference_s)))+'\n')
    temporary.replace(target)
