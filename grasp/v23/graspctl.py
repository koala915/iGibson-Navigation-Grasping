#!/usr/bin/env python3
"""Tiny client for grasp_service.py. No dependencies, runs on system python3.

    python3 graspctl.py status
    python3 graspctl.py grasp
    python3 graspctl.py quit

Exit code is 0 only when the service reports ok (and, for `grasp`, a confirmed
hold), so a shell loop can drive repeats without parsing the JSON.
"""
# No `from __future__ import annotations`: this client is meant to run on the
# Jetson's system python3 (3.6.9), which predates it. Keep it dependency-free.
import json
import socket
import sys

DEFAULT_SOCKET = "/tmp/grasp_service.sock"
# A grasp episode is ~10 s today; the cap is generous so a slow retry still
# reports its own outcome rather than the client inventing a timeout.
TIMEOUTS = {"grasp": 180.0, "release": 120.0, "home": 60.0,
            "status": 15.0, "quit": 15.0}


def main(argv) -> int:
    cmd = argv[1] if len(argv) > 1 else "status"
    sock_path = argv[2] if len(argv) > 2 else DEFAULT_SOCKET
    if cmd not in TIMEOUTS:
        print("usage: graspctl.py [grasp|status|quit] [socket]", file=sys.stderr)
        return 2

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(TIMEOUTS[cmd])
    try:
        sock.connect(sock_path)
    except OSError as exc:
        print("cannot reach the grasp service at %s: %s" % (sock_path, exc),
              file=sys.stderr)
        return 1
    try:
        sock.sendall((cmd + "\n").encode())
        reply = sock.makefile("r").readline().strip()
    except socket.timeout:
        print("no reply within %.0fs — the service may still be mid-episode"
              % TIMEOUTS[cmd], file=sys.stderr)
        return 1
    finally:
        sock.close()

    if not reply:
        print("empty reply", file=sys.stderr)
        return 1
    try:
        data = json.loads(reply)
    except ValueError:
        print(reply)
        return 1

    print(json.dumps(data, indent=2, ensure_ascii=False))
    if not data.get("ok"):
        return 1
    if cmd == "grasp" and not data.get("confirmed"):
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
