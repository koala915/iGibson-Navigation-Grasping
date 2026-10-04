#!/usr/bin/env python3
"""Rear-camera encode-only diagnostic. No motors, serial, network or image files."""
import json
import os
import resource
import subprocess
import time

REAR = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
ARM = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0'


def main():
    if not os.path.exists(REAR) or os.path.realpath(REAR) == os.path.realpath(ARM):
        raise RuntimeError('rear camera identity unavailable')
    owner = subprocess.run(['fuser', REAR], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if owner.returncode == 0:
        raise RuntimeError('rear camera is already owned')
    cmd = ['gst-launch-1.0', '-q', 'v4l2src', 'device=' + REAR, 'num-buffers=60',
           'do-timestamp=true', '!', 'video/x-raw,format=YUY2,width=640,height=480,framerate=30/1',
           '!', 'videoconvert', '!', 'video/x-raw,format=I420', '!', 'nvvidconv',
           '!', 'video/x-raw(memory:NVMM),format=NV12', '!', 'nvv4l2h264enc',
           'bitrate=2000000', 'insert-sps-pps=true', 'iframeinterval=30',
           '!', 'h264parse', '!', 'rtph264pay', 'config-interval=1', 'pt=96',
           '!', 'fakesink', 'sync=false']
    start = time.monotonic()
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
    elapsed = time.monotonic() - start
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    print(json.dumps({'exit': result.returncode, 'frames': 60, 'elapsed_s': elapsed,
        'max_rss_mb': usage.ru_maxrss / 1024.0,
        'cpu_core_percent': 100 * (usage.ru_utime + usage.ru_stime - before.ru_utime - before.ru_stime) / elapsed,
        'stderr': result.stderr.decode('utf-8', errors='replace')}))
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
