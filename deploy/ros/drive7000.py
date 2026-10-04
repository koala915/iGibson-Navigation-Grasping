"""Send the TCP 7000 velocity protocol, as the approach controller does (G2 tests).

usage:
  python drive7000.py HOST drive VX WZ SECONDS     # 10 Hz commands, then stop
  python drive7000.py HOST silent VX WZ            # one command, then hold the socket quiet 2 s (watchdog)
  python drive7000.py HOST drop VX WZ SECONDS      # 10 Hz commands, then disconnect without stop
"""
import json
import socket
import sys
import time

host, mode = sys.argv[1], sys.argv[2]
vx, wz = float(sys.argv[3]), float(sys.argv[4])
secs = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0

s = socket.create_connection((host, 7000), timeout=3)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)


def send(msg):
    s.sendall((json.dumps(msg) + "\n").encode())


vel = {"action": "velocity", "vx": vx, "wz": wz}
t0 = time.time()
if mode == "silent":
    send(vel)
    print("%.2f sent one command, now silent for 2 s" % (time.time() - t0), flush=True)
    time.sleep(2.0)
else:
    n = 0
    while time.time() - t0 < secs:
        send(vel)
        n += 1
        time.sleep(0.1)
    print("%.2f sent %d commands" % (time.time() - t0, n), flush=True)
    if mode == "drive":
        for _ in range(3):
            send({"action": "stop"})
            time.sleep(0.03)
        print("%.2f sent stop" % (time.time() - t0), flush=True)
s.close()
print("%.2f closed" % (time.time() - t0), flush=True)
