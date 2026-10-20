"""One SSH connection for bounded resident commands.

A command that moves the arm is never retried: with its reply lost, its outcome
is unknown and the session closes for good. A read-only one (status, health)
changes nothing on the robot, so a lost reply ends only that ssh channel; it is
asked once more on a new one, which later commands use too (2026-10-21: a WiFi
drop at the venue should not end the mission).

The server uses only Jetson Python 3.6's standard library and existing sockets;
it neither imports a controller nor owns a serial port.
"""
import json
import queue
import shlex
import subprocess
import threading

READ_ONLY = ('status', 'health')

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
        self.host, self.process, self.replies, self.sequence = host, None, None, 0
        self.popen, self.lock = popen, threading.Lock()
        self.closed = False

    def _start(self):
        # Each channel has its own queue: a late reply, or the end of a dropped
        # channel, must never be read as an answer on the next one.
        self.replies = queue.Queue()
        self.process = self.popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                                  'jetson@' + self.host, 'python3 -u -c ' + shlex.quote(SERVER)],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, args=(self.process, self.replies),
                         daemon=True).start()

    @staticmethod
    def _read(process, replies):
        try:
            for line in process.stdout:
                replies.put(json.loads(line))
        except (OSError, ValueError) as exc:
            replies.put(exc)
        finally:
            replies.put(EOFError('resident command channel ended'))

    def _exchange(self, command, timeout):
        if self.process is not None and not self.replies.empty():
            # Something arrived unasked: the channel ended while idle. Nothing of
            # this command has gone out yet, so any command may take a new one.
            self._drop()
        if self.process is None:
            self._start()
        self.sequence += 1
        self.process.stdin.write(json.dumps({'id': self.sequence, 'command': command})+'\n')
        self.process.stdin.flush()
        reply = self.replies.get(timeout=timeout)
        if (not isinstance(reply, dict) or reply.get('id') != self.sequence
                or 'error' in reply or not isinstance(reply.get('reply'), dict)):
            raise RuntimeError('resident command reply invalid: ' + str(reply))
        return reply

    def request(self, host, command, timeout):
        if host != self.host:
            raise ValueError('resident command channel belongs to another robot')
        with self.lock:
            if self.closed:
                raise RuntimeError('resident command channel closed; command was not replayed')
            # No reply, a late or garbled one, or a socket error on the robot leave
            # an arm command's outcome unknown: the session closes, nothing is resent.
            # A read-only one is asked once more on a new channel.
            tries = 2 if command in READ_ONLY else 1
            for attempt in range(tries):
                try:
                    reply = self._exchange(command, timeout)
                    break
                except KeyboardInterrupt:
                    self.close()
                    raise
                except Exception as exc:
                    if command not in READ_ONLY:
                        self.close()
                        raise
                    self._drop()
                    if attempt + 1 == tries:
                        raise
                    print('[RPC] {} reply lost ({}): asking again on a new channel'.format(
                        command, str(exc) or type(exc).__name__), flush=True)
        # The service answered this very request: a refusal or an unconfirmed grasp
        # is an outcome, and the channel stays usable for the next command (the
        # loop has to read the arm's state after a missed grasp).
        if reply['reply'].get('ok') is not True:
            raise RuntimeError('{} refused: {}'.format(command, reply['reply']))
        if command == 'grasp' and reply['reply'].get('confirmed') is not True:
            raise RuntimeError('grasp was not confirmed')
        return reply['reply']

    def _drop(self):
        """End the current ssh channel only; the next command opens another."""
        process, self.process = self.process, None
        if process is None:
            return
        try:
            process.stdin.close()
            process.wait(timeout=1)
        except (OSError, ValueError, subprocess.TimeoutExpired):
            process.kill()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._drop()
