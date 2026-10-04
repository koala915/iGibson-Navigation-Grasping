#!/usr/bin/env python3
"""Manual four-wheel motor-server smoke test.

The default is a dry-run. Pass ``--real`` explicitly to send motion commands.
Importing this module is always side-effect free.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import time


def send(sock: socket.socket, msg: dict) -> None:
    sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description="X3Plus set_motor smoke test")
    parser.add_argument(
        "--host",
        default=os.getenv("X3PLUS_JETSON_HOST", "yahboom.local"),
    )
    parser.add_argument("--port", type=int, default=7000)
    parser.add_argument("--speed", type=int, default=30)
    parser.add_argument("--real", action="store_true", help="actually move the robot")
    args = parser.parse_args()

    speed = max(0, min(abs(args.speed), 100))
    sequence = [
        ({"action": "set_motor", "fl": speed, "fr": speed,
          "rl": speed, "rr": speed}, 1.0),
        ({"action": "set_motor", "fl": 0, "fr": 0, "rl": 0, "rr": 0}, 0.5),
        ({"action": "set_motor", "fl": -speed, "fr": speed,
          "rl": speed, "rr": -speed}, 1.0),
    ]
    stop = {"action": "set_motor", "fl": 0, "fr": 0, "rl": 0, "rr": 0}

    if not args.real:
        print("[DRY] no commands sent; add --real for supervised motion")
        for command, duration in sequence:
            print(f"[DRY] {command} for {duration:.1f}s")
        print(f"[DRY] final {stop}")
        return 0

    sock = socket.create_connection((args.host, args.port), timeout=2.0)
    try:
        for command, duration in sequence:
            send(sock, command)
            time.sleep(duration)
    finally:
        try:
            send(sock, stop)
        finally:
            sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
