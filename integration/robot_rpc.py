"""One SSH connection for bounded resident commands. Never retries a command.

The server uses only Jetson Python 3.6's standard library and existing sockets;
it neither imports a controller nor owns a serial port.
"""
import json
import queue
import shlex
import subprocess
import threading

SERVER = r'''
import json, socket, sys
allowed = {'status':15, 'health':2, 'home':60, 'stow':60, 'grasp':180,
           'release':120, 'release-preview':120, 'maintenance-lock':10,
           'maintenance-unlock':10}
for line in sys.stdin:
    request = json.loads(line)
    command = request['command']
    if command not in allowed:
        raise ValueError('unsupported command')
    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    connection.settimeout(allowed[command])
    try:
        connection.connect('/tmp/grasp_health.sock' if command == 'health'
                           else '/tmp/grasp_service.sock')
        connection.sendall((command+'\n').encode())
        reply = json.loads(connection.makefile('r').readline())
        result = {'id':request['id'], 'reply':reply}
    except Exception as exc:
        result = {'id':request['id'], 'error':str(exc)}
    finally:
        connection.close()
    sys.stdout.write(json.dumps(result)+'\n')
    sys.stdout.flush()
'''


class RobotSession:
    def __init__(self, host, popen=subprocess.Popen):
        self.host, self.process, self.sequence = host, None, 0
        self.popen, self.lock, self.replies = popen, threading.Lock(), queue.Queue()
        self.closed = False

    def _start(self):
        self.process = self.popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                                  'jetson@' + self.host, 'python3 -u -c ' + shlex.quote(SERVER)],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        try:
            for line in self.process.stdout:
                self.replies.put(json.loads(line))
        except (OSError, ValueError) as exc:
            self.replies.put(exc)
        finally:
            self.replies.put(EOFError('resident command channel ended'))

    def request(self, host, command, timeout):
        if host != self.host:
            raise ValueError('resident command channel belongs to another robot')
        with self.lock:
            if self.closed:
                raise RuntimeError('resident command channel closed; command was not replayed')
            # No reply, a late or garbled one, or a socket error on the robot leave
            # the command's outcome unknown: the channel closes and nothing is resent.
            try:
                if self.process is None:
                    self._start()
                self.sequence += 1
                self.process.stdin.write(json.dumps({'id': self.sequence, 'command': command})+'\n')
                self.process.stdin.flush()
                reply = self.replies.get(timeout=timeout)
                if (not isinstance(reply, dict) or reply.get('id') != self.sequence
                        or 'error' in reply or not isinstance(reply.get('reply'), dict)):
                    raise RuntimeError('resident command reply invalid: ' + str(reply))
            except (Exception, KeyboardInterrupt):
                self.close()
                raise
        # The service answered this very request: a refusal or an unconfirmed grasp
        # is an outcome, and the channel stays usable for the next command (the
        # loop has to read the arm's state after a missed grasp).
        if reply['reply'].get('ok') is not True:
            raise RuntimeError('{} refused: {}'.format(command, reply['reply']))
        if command == 'grasp' and reply['reply'].get('confirmed') is not True:
            raise RuntimeError('grasp was not confirmed')
        return reply['reply']

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.process is not None:
            try:
                self.process.stdin.close()
                self.process.wait(timeout=1)
            except (OSError, subprocess.TimeoutExpired):
                self.process.kill()
                self.process.wait(timeout=3)
