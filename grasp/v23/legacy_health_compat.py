"""Cached health metadata for older deployed v23 controllers.

No Serial access, actuator calls or inferred transport diagnostics. This
adapter deliberately advertises that automatic Serial-owner restart is absent.
"""
import os
import time


def health_snapshot(service, resident_rss_mb):
    controller=service.controller
    device=getattr(controller.servo,'device',None)
    transport=getattr(device,'__dict__',{}).get('_v23_transport')
    serial=transport.status() if transport is not None else None
    return dict(ok=serial is None or bool(serial['healthy']),serial=serial,
                stack_version='v23',service_pid=os.getpid(),
                model_path=str(controller.cfg.model_path),
                vecnorm_path=str(controller.cfg.vecnorm_path),
                holding=getattr(controller,'_grip_hold_rad',None) is not None,
                chassis=service.chassis.status() if service.chassis else None,
                odom=service.odom_bridge.status() if service.odom_bridge else None,
                startup_pose=service.startup_pose,chassis_tcp_ready=service.chassis_tcp_ready,
                rss_mb=round(resident_rss_mb,1),
                uptime_s=round(time.monotonic()-service.started_monotonic,1),
                capabilities=dict(independent_health=True,owner_restart=False,
                                  serial_transport_diagnostics=serial is not None))
