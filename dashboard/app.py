#!/usr/bin/env python3
"""app.py — Flask + Socket.IO dashboard (venv Python).

  sim  — steps the world in-process (Runner + NetModel); the config panel restarts it
  emu  — relays a running orchestrator over sdnv/hub.py (HubClient), commands go upstream

Socket.IO downstream: init | frame | stats | event (see sdnv/runner.py) and link (emu hub state).
Upstream: cmd {"op": dispatch|recall|scenario|engine|add|speed|pause|restart, ...}.

Usage:
  ~/sdv-venv/bin/python -m dashboard.app                    # sim, http://127.0.0.1:7000
  ~/sdv-venv/bin/python -m dashboard.app --vehicles 40 --rsus 12 --speed 2
  ~/sdv-venv/bin/python -m dashboard.app --mode emu         # attach to the orchestrator hub
"""

import argparse
import os
import socket
import sys
import threading

from flask import Flask
from flask_socketio import SocketIO, emit

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sdnv import config                      # noqa: E402
from sdnv.hub import HubClient               # noqa: E402
from sdnv.netmodel import NetModel           # noqa: E402
from sdnv.runner import Runner               # noqa: E402
from sdnv.world import World                 # noqa: E402

# config-panel keys a sim restart accepts, with sane bounds for an interactive map
RESTART = {'vehicles': (1, 200), 'emergency': (0, 20), 'rsus': (1, 40), 'rsu_range': (50, 1500),
           'seed': (0, 10 ** 6), 'auto_dispatch': (0, 3600), 'scenario': None, 'signals': None}

app = Flask(__name__, static_folder='static', static_url_path='/static')
sio = SocketIO(app, async_mode='threading')
backend = None


class Sim:
    """In-process world; restart() rebuilds it with overrides from the config panel."""

    def __init__(self, overrides):
        self.overrides, self.cache, self.lock = dict(overrides), {}, threading.Lock()
        self.runner = self.stop = self.thread = None

    def emit(self, kind, data):
        if kind in ('init', 'stats'):
            self.cache[kind] = data
        sio.emit(kind, data)

    def start(self, overrides=None):
        with self.lock:
            if self.thread:
                self.overrides['speed'] = self.runner.speed
                self.stop.set()
                self.thread.join()
            self.overrides.update(overrides or {})
            cfg = config.load(self.overrides)
            self.runner = Runner(World(cfg), NetModel(cfg), self.emit, cfg)
            self.stop = threading.Event()
            self.thread = threading.Thread(target=self.runner.run, args=(self.stop,), daemon=True)
            self.thread.start()

    def command(self, cmd):
        if cmd.get('op') == 'restart':
            self.start(_clean(cmd.get('config') or {}))
        elif self.runner:
            self.runner.submit(cmd)


class Relay:
    """Emulation: mirror the orchestrator's hub stream to every browser."""

    def __init__(self, cfg):
        self.cache = {'link': {'state': 'waiting', 'hub': f'{cfg["hub_host"]}:{cfg["hub_port"]}'}}
        self.client = HubClient(cfg['hub_host'], int(cfg['hub_port']), self.on_message, self.on_state)
        self.client.start()

    def on_message(self, kind, data):
        if kind in ('init', 'stats'):
            self.cache[kind] = data
        sio.emit(kind, data)

    def on_state(self, state):
        self.cache['link'] = dict(self.cache['link'], state=state)
        sio.emit('link', self.cache['link'])

    def command(self, cmd):
        if cmd.get('op') != 'restart':            # the testbed is restarted with run.sh, not from the browser
            self.client.send(cmd)


def _clean(raw):
    out = {}
    for key, bounds in RESTART.items():
        val = raw.get(key)
        if val in (None, ''):
            continue
        if bounds is None:
            out[key] = str(val)
            continue
        try:
            num = type(config.PARAMS[key][1])(float(val))
        except (TypeError, ValueError):
            continue
        out[key] = max(bounds[0], min(bounds[1], num))
    return out


def _free_port(host, port, tries=20):
    """First bindable port from `port` (macOS AirPlay Receiver holds 7000)."""
    for p in range(port, port + tries):
        with socket.socket() as s:
            try:
                s.bind((host, p))
                return p
            except OSError:
                continue
    return port


@app.route('/')
def index():
    return app.send_static_file('index.html')


@sio.on('connect')
def on_connect():
    for kind in ('link', 'init', 'stats'):
        if kind in backend.cache:
            emit(kind, backend.cache[kind])


@sio.on('cmd')
def on_cmd(cmd):
    if isinstance(cmd, dict):
        backend.command(cmd)


def main():
    global backend
    ap = argparse.ArgumentParser(description='SDNV live dashboard')
    ap.add_argument('--mode', choices=('sim', 'emu'), default=os.environ.get('SDNV_MODE', 'sim'))
    ap.add_argument('--host', default=os.environ.get('SDNV_DASH_HOST', '127.0.0.1'))
    ap.add_argument('--port', type=int)
    for key in ('map', 'scenario', 'vehicles', 'emergency', 'rsus', 'rsu_range', 'seed', 'auto_dispatch', 'speed'):
        ap.add_argument('--' + key.replace('_', '-'), dest=key)
    args = ap.parse_args()

    overrides = {k: v for k, v in vars(args).items() if k in config.PARAMS and v is not None}
    cfg = config.load(overrides)
    if args.mode == 'sim':
        backend = Sim(overrides)
        backend.start()
    else:
        backend = Relay(cfg)
    port = args.port or _free_port(args.host, int(cfg['dash_port']))
    print(f'SDNV dashboard ({args.mode}) → http://{args.host}:{port}', flush=True)
    sio.run(app, host=args.host, port=port, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
