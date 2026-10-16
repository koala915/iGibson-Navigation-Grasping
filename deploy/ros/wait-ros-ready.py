#!/usr/bin/python3
"""Bounded startup readiness checks; standard library, no robot commands."""
import argparse
import socket
import time
import xmlrpc.client

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['master', 'bridge'])
parser.add_argument('--timeout', type=float, default=30)
args = parser.parse_args()
socket.setdefaulttimeout(1)
deadline = time.monotonic() + args.timeout
last_error = ''
while time.monotonic() < deadline:
    try:
        result = xmlrpc.client.ServerProxy('http://127.0.0.1:11311').getPid('/startup_readiness')
        if result[0] != 1:
            raise RuntimeError(str(result))
        if args.mode == 'bridge':
            with socket.create_connection(('127.0.0.1', 9090), timeout=1):
                pass
        print(args.mode + ' ready', flush=True)
        raise SystemExit(0)
    except (OSError, xmlrpc.client.Error, RuntimeError) as exc:
        last_error = str(exc)
    time.sleep(0.25)
raise SystemExit(args.mode + ' not ready: ' + last_error)
