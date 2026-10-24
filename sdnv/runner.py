"""runner.py — real-time loop shared by sim (dashboard in-process) and emu (orchestrator).

Every loop: apply queued commands → step the world (10 Hz physics, scaled by the
sim speed) → let the backend react (NetModel or the live testbed) → emit
  init   once (static map, RSUs, signal approaches)
  frame  ~5 Hz (positions, signal states, ECM cells)
  stats  ~1 Hz (vehicles, RSUs, KPIs, controller, missions)
  event  as they happen
through a single `emit(kind, data)` callback (Socket.IO or the TCP hub).
"""

import queue
import time

from .world import DT, KINDS


class Runner:
    def __init__(self, world, backend, emit, cfg, frame_hz=5.0, stats_hz=1.0):
        self.world, self.backend, self.emit, self.cfg = world, backend, emit, cfg
        self.speed = float(cfg.get('speed', 1.0)) if backend.mode == 'sim' else 1.0
        self.paused = False
        self.cmds = queue.Queue()
        self.frame_dt, self.stats_dt = 1.0 / frame_hz, 1.0 / stats_hz
        self._acc = 0.0

    def submit(self, cmd):
        self.cmds.put(cmd)

    def init_msg(self):
        d = self.world.describe()
        d['mode'], d['modeled'] = self.backend.mode, self.backend.modeled
        d['config'] = {k: self.cfg[k] for k in ('vehicles', 'emergency', 'rsus', 'rsu_range', 'seed',
                                                 'auto_dispatch', 'signals')}
        d['config'].update(scenario=self.world.scenario, speed=self.speed, dynamic=self.backend.dynamic,
                           vehicles=len(self.world.vehicles))
        return d

    def stats_msg(self):
        s = self.world.stats()
        self.backend.annotate(s, self.world)
        s['speed'], s['paused'], s['mode'] = self.speed, self.paused, self.backend.mode
        return s

    def _apply(self, c):
        op, w = c.get('op'), self.world
        if op == 'dispatch':
            w.dispatch(c.get('id'))
        elif op == 'recall':
            w.recall(c.get('id'))
        elif op == 'scenario':
            w.set_scenario(c.get('value'))
        elif op == 'engine':
            w.engine(c.get('id'), c.get('on'))
        elif op == 'add' and self.backend.dynamic and c.get('kind', 'car') in KINDS:
            w.add(c.get('kind', 'car'), max(1, min(20, int(c.get('n', 1)))))
        elif op == 'speed' and self.backend.mode == 'sim':
            self.speed = max(0.25, min(16.0, float(c.get('value', 1))))
        elif op == 'pause':
            self.paused = not self.paused

    def _step(self, n, emit_events=True):
        w, b = self.world, self.backend
        for _ in range(n):
            w.step(DT)
            b.tick(w, DT)
            events, alerts = w.drain()
            if alerts:
                b.on_alerts(w, alerts)
            if emit_events:
                for e in events:
                    self.emit('event', e)

    def run(self, stop):
        """Real-time loop until `stop` (threading.Event) is set."""
        self.emit('init', self.init_msg())
        tick = next_frame = next_stats = time.monotonic()
        while not stop.is_set():
            while not self.cmds.empty():
                try:
                    self._apply(self.cmds.get_nowait())
                except Exception as e:                       # a bad command must not kill the loop
                    self.world.log(f'command failed: {e}', 'warn')
            if not self.paused:
                self._acc += self.speed
                n, self._acc = int(self._acc), self._acc - int(self._acc)
                self._step(n)
            now = time.monotonic()
            if now >= next_frame:
                self.emit('frame', self.world.frame())
                next_frame = max(next_frame + self.frame_dt, now)
            if now >= next_stats:
                self.emit('stats', self.stats_msg())
                next_stats = max(next_stats + self.stats_dt, now)
            tick += DT
            delay = tick - time.monotonic()
            if delay > 0:
                stop.wait(delay)
            elif delay < -1.0:                                 # fell behind (heavy emu tick): resync
                tick = time.monotonic()

    def run_headless(self, duration, sample=1.0):
        """As fast as possible (sim only); returns one KPI row per `sample` seconds."""
        rows, nxt = [], sample
        while self.world.t < duration:
            self._step(1, emit_events=False)
            if self.world.t >= nxt:
                rows.append(kpi_row(self.stats_msg()))
                nxt += sample
        return rows


KPI_FIELDS = ('t', 'scenario', 'em_latency_ms', 'em_jitter_ms', 'em_tput_mbps', 'em_loss_pct', 'be_latency_ms',
              'be_tput_mbps', 'suppression_pct', 'policy_reaction_ms', 'emapt_p50', 'emapt_p90', 'emapt_p100',
              'mqtt_latency_ms', 'coverage_pct', 'ecm_cells', 'active_evs', 'handovers', 'yields', 'preemptions')


def kpi_row(stats):
    k = stats.get('kpi', {})
    em = k.get('emapt') or {}
    c = stats.get('counters', {})
    row = dict(k, t=stats['t'], scenario=stats['scenario'], emapt_p50=em.get('p50'), emapt_p90=em.get('p90'),
               emapt_p100=em.get('p100'), active_evs=sum(1 for v in stats['vehicles'] if v['siren']),
               handovers=c.get('handovers'), yields=c.get('yields'), preemptions=c.get('preemptions'))
    return {f: row.get(f) for f in KPI_FIELDS}
