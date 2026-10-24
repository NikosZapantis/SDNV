"""hub.py — newline-delimited JSON link between the emulation orchestrator and the dashboard.

The orchestrator (root, system Python with Mininet-WiFi) runs HubServer; the
dashboard (venv Python with Flask) runs HubClient. Messages are
{"type": init|frame|stats|event, "data": {...}} downstream and command objects
({"op": "dispatch", ...}) upstream. A late or reconnecting dashboard receives the
cached init/stats immediately, so "Connect to running emulation" just works.
"""

import json
import queue
import socket
import threading


def _line(obj):
    return (json.dumps(obj, separators=(',', ':'), ensure_ascii=False) + '\n').encode()


class HubServer:
    def __init__(self, host, port, on_command):
        self.on_command = on_command
        self.sock = socket.create_server((host, port))
        self.clients, self.cache, self.lock = [], {}, threading.Lock()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                return
            q = queue.Queue(maxsize=500)
            for kind in ('init', 'stats'):
                if kind in self.cache:
                    q.put(self.cache[kind])
            with self.lock:
                self.clients.append(q)
            threading.Thread(target=self._writer, args=(conn, q), daemon=True).start()
            threading.Thread(target=self._reader, args=(conn,), daemon=True).start()

    def _writer(self, conn, q):
        try:
            while True:
                data = q.get()
                if data is None:
                    break
                conn.sendall(data)
        except OSError:
            pass
        finally:
            with self.lock:
                if q in self.clients:
                    self.clients.remove(q)
            conn.close()

    def _reader(self, conn):
        try:
            for raw in conn.makefile('rb'):
                try:
                    self.on_command(json.loads(raw))
                except ValueError:
                    pass
        except OSError:
            pass

    def emit(self, kind, data):
        line = _line({'type': kind, 'data': data})
        if kind in ('init', 'stats'):
            self.cache[kind] = line
        with self.lock:
            for q in self.clients:
                try:
                    q.put_nowait(line)
                except queue.Full:            # slow client: drop rather than stall the emulation
                    pass

    def close(self):
        with self.lock:
            for q in self.clients:
                q.put(None)
        self.sock.close()


class HubClient(threading.Thread):
    def __init__(self, host, port, on_message, on_state, retry=2.0):
        super().__init__(daemon=True)
        self.addr, self.on_message, self.on_state, self.retry = (host, port), on_message, on_state, retry
        self.stop_ev, self.sock = threading.Event(), None

    def run(self):
        while not self.stop_ev.is_set():
            try:
                self.sock = socket.create_connection(self.addr, timeout=3)
                self.sock.settimeout(None)
                self.on_state('connected')
                for raw in self.sock.makefile('rb'):
                    msg = json.loads(raw)
                    self.on_message(msg['type'], msg['data'])
            except (OSError, ValueError):
                pass
            self.sock = None
            if not self.stop_ev.is_set():
                self.on_state('waiting')
                self.stop_ev.wait(self.retry)

    def send(self, obj):
        try:
            if self.sock:
                self.sock.sendall(_line(obj))
                return True
        except OSError:
            pass
        return False

    def stop(self):
        self.stop_ev.set()
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
