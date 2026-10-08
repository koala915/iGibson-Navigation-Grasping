"""Bounded, diagnostic-only socket independent of the arm command loop."""
import json
import os
import socket
import threading


class HealthServer:
    def __init__(self, snapshot, path='/tmp/grasp_health.sock'):
        self.snapshot, self.path = snapshot, path
        self.stop = threading.Event()
        self.thread = None

    def start(self):
        if os.path.exists(self.path):
            os.unlink(self.path)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(self.path)
        os.chmod(self.path, 0o600)
        self.sock.listen(4)
        self.sock.settimeout(0.2)
        self.thread = threading.Thread(target=self.run, name='v23-health', daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                conn, _ = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                conn.settimeout(0.2)
                try:
                    command = conn.recv(64).strip()
                    try:
                        reply = self.snapshot() if command == b'health' else {'ok':False}
                    except Exception as exc:
                        reply = {'ok':False,'reason':'health_snapshot_failed','detail':str(exc)}
                    conn.sendall((json.dumps(reply)+'\n').encode())
                except (OSError, ValueError):
                    pass

    def close(self):
        self.stop.set()
        self.sock.close()
        self.thread.join(timeout=1)
        if os.path.exists(self.path):
            os.unlink(self.path)
