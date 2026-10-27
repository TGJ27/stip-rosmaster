"""Supervise complete launch process groups, not only their parent processes."""
import collections
import os
import signal
import subprocess
import threading
import time


class Processes:
    def __init__(self):
        self.lock = threading.RLock()
        self.active = {}
        self.logs = collections.deque(maxlen=250)

    @staticmethod
    def group_alive(pgid):
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def status(self):
        with self.lock:
            return {name: {'running': self.group_alive(p.pid), 'pid': p.pid,
                           'returncode': p.poll(), 'parent_alive': p.poll() is None}
                    for name, p in self.active.items()}

    def start(self, name, command, env=None):
        with self.lock:
            old = self.active.get(name)
            if old and self.group_alive(old.pid):
                raise ValueError('{} process group already exists'.format(name))
            p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, bufsize=1, start_new_session=True, env=env)
            self.active[name] = p
            self.logs.append('{} START {}'.format(name, ' '.join(command)))
            threading.Thread(target=self._reader, args=(name, p), daemon=True).start()
            return {'name': name, 'pid': p.pid}

    def _reader(self, name, p):
        try:
            for line in p.stdout:
                with self.lock:
                    self.logs.append('{}: {}'.format(name, line.rstrip()[:380]))
        except Exception as e:
            with self.lock:
                self.logs.append('{} reader: {}'.format(name, e))

    def stop(self, name, seconds=6):
        with self.lock:
            p = self.active.get(name)
            if not p:
                return {'stopped': True}
            pgid = p.pid  # start_new_session=True creates a group with id=PID
            if not self.group_alive(pgid):
                return {'stopped': True}
            try:
                os.killpg(pgid, signal.SIGINT)
            except ProcessLookupError:
                return {'stopped': True}
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and self.group_alive(pgid):
            time.sleep(.1)
        if self.group_alive(pgid):
            os.killpg(pgid, signal.SIGTERM)
            time.sleep(.8)
        if self.group_alive(pgid):
            os.killpg(pgid, signal.SIGKILL)
            time.sleep(.15)
        with self.lock:
            self.logs.append('{} STOP {}\n'.format(name,'CLEAN' if not self.group_alive(pgid) else 'UNCERTAIN'))
        return {'stopped': not self.group_alive(pgid)}

    def stop_all(self):
        for name in list(self.active):
            try:
                self.stop(name)
            except Exception:
                pass
