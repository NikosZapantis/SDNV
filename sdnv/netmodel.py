"""netmodel.py — analytical RSU-cell network model for simulation mode.

Reproduces the qualitative behaviour of the sdnv-mininet-project v6 traffic
profile without Mininet: every vehicle offers elastic background TCP, a
dispatched emergency vehicle adds 10 Mbit/s UDP, and each RSU cell is a single
bottleneck of `cell_mbit`.

  baseline / no ECM : one FIFO → TCP fills the buffer (standing queue), drop-tail
                      losses hit the emergency UDP too.
  sdnv ECM cell     : vehicle-side HTB + strict priority → emergency class served
                      first, helper TCP capped to `helper_tcp_limit` (total).

All numbers are MODELLED (flagged `modeled: true`); emulation mode replaces this
class with measurements from the real testbed (emulator/emunet.py).
"""

import collections
import random
import statistics


class NetModel:
    mode, modeled, dynamic = 'sim', True, True

    def __init__(self, cfg):
        self.cfg = cfg
        self.rng = random.Random(int(cfg['seed']) + 101)
        self.acc = 0.0
        self.veh, self.cells, self.kpi = {}, [], {}
        self.bg_ref = {}
        self.reactions = collections.deque(maxlen=20)
        self.em_lat = collections.deque(maxlen=10)
        self.emapt = None
        self.msgs = 0
        self.ctrl_events = collections.deque(maxlen=12)
        self._ecm_seen = {}

    # ── model ─────────────────────────────────────────────────────────────
    def _cell(self, members, evs, ecm):
        c = self.cfg['cell_mbit'] * self.rng.uniform(0.97, 1.03)
        em_rate, bg = self.cfg['emergency_mbit'], self.cfg['bg_mbit']
        svc = 12.0 / c                                   # ms to serialise one 1500 B frame
        em_demand, n = em_rate * len(evs), max(1, len(members))
        if not ecm:
            total = em_demand + bg * len(members)
            if total <= c:
                rho = total / c
                q, loss = min(svc * rho / max(1e-3, 1 - rho), self.cfg['buffer_ms']), 0.0
                be_each = bg
            else:
                over = (total - c) / total
                q = self.cfg['buffer_ms'] * (0.55 + 0.45 * min(1.0, 2 * over))
                loss = min(0.6, 0.5 * over)
                be_each = max(0.0, c - em_demand * (1 - loss)) / n
            q *= self.rng.uniform(0.85, 1.15)
            return {'cap': c, 'em': em_rate * (1 - loss), 'loss': loss, 'be': {v: be_each for v in members},
                    'q_hp': q, 'q_be': q, 'util': min(1.0, total / c)}
        em_served = min(em_demand, 0.92 * c)
        loss = 1 - em_served / em_demand if em_demand else 0.0
        rho = em_served / c
        q_hp = min(svc * rho / max(1e-3, 1 - rho), 15.0) * self.rng.uniform(0.8, 1.2)
        helpers = [v for v in members if v not in evs]
        cap_each = min(bg, self.cfg['helper_tcp_limit'] / max(1, len(helpers)))
        room = max(0.0, c - em_served)
        be = {v: (0.1 if v in evs else cap_each) for v in members}
        scale = min(1.0, room / max(1e-6, sum(be.values())))
        be = {v: x * scale for v, x in be.items()}
        q_be = (5.0 + 10.0 * self.rng.random()) + q_hp * 0.5    # fq_codel target + HTB shaping
        return {'cap': c, 'em': em_served / max(1, len(evs)), 'loss': loss, 'be': be,
                'q_hp': q_hp, 'q_be': q_be, 'util': min(1.0, (em_served + sum(be.values())) / c)}

    def tick(self, world, dt):
        self.acc += dt
        if self.acc < 1.0:
            return
        self.acc = 0.0
        base = self.cfg['base_rtt_ms']
        self.veh, self.cells = {}, []
        for r in world.rsus:
            ecm = r.ecm_since is not None
            cell = self._cell(r.members, r.evs, ecm)
            self.cells.append({'load': round(sum(cell['be'].values()) + cell['em'] * len(r.evs), 2),
                               'util': round(cell['util'], 3), 'cap': round(cell['cap'], 1)})
            for v in r.members:
                ev = v in r.evs
                rtt = base + (cell['q_hp'] if ev else cell['q_be']) + self.rng.uniform(0, 1.5)
                # under ECM the helpers' telemetry (MQTT/1883) is in their HP class as well
                mqtt = base / 2 + (cell['q_hp'] if ev or ecm else cell['q_be'])
                self.veh[v.id] = {'tput': round(cell['em'] if ev else cell['be'][v], 2),
                                  'be': round(cell['be'][v], 3), 'rtt': round(rtt, 2), 'mqtt': round(mqtt, 2),
                                  'loss': round(cell['loss'] * 100, 2) if ev else 0.0,
                                  'cls': 'hp' if ev else 'be', 'rsu_actual': r.id}
                self.msgs += 1
                if v.policy == 'bg':
                    self.bg_ref[v.id] = 0.8 * self.bg_ref.get(v.id, cell['be'][v]) + 0.2 * cell['be'][v]
            # policy reaction + controller flows on each new ECM activation (modelled)
            if ecm and self._ecm_seen.get(r.idx) != r.ecm_seq:
                self._ecm_seen[r.idx] = r.ecm_seq
                ms = (20.0 + 25.0 * len(r.members)) * self.rng.uniform(0.9, 1.1)
                self.reactions.append(ms)
                self.ctrl_events.appendleft({'t': world.t, 'msg': f'ECM flows → {r.id} for '
                                                                   f'{",".join(e.id for e in r.evs)} ({ms:.0f} ms)'})
        self._kpis(world)

    def on_alerts(self, world, alerts):
        """EV awareness broadcast to its cell (UDP/6000): HP class only under ECM."""
        for a in alerts:
            r = world.rsus[a['rsu']]
            ecm = r.ecm_since is not None
            cell = self._cell(r.members, r.evs, ecm)
            q = cell['q_hp'] if ecm else cell['q_be']
            delays = sorted(self.cfg['base_rtt_ms'] / 2 + q + 0.15 * i + self.rng.uniform(0, 1.5)
                            for i in range(len(a['helpers'])))
            if delays:
                self.emapt = dict(_percentiles(delays), ev=a['ev'], rsu=r.id, n=len(delays))

    def _kpis(self, world):
        evs = [v for v in world.vehicles if v.siren and v.id in self.veh]
        civ = [v for v in world.vehicles if not v.siren and v.id in self.veh]
        em_rtt = _mean(self.veh[v.id]['rtt'] for v in evs)
        if em_rtt is not None:
            self.em_lat.append(em_rtt)
        helpers = [v for v in world.vehicles if v.policy == 'helper' and v.id in self.veh]
        ref = sum(self.bg_ref.get(v.id, self.cfg['bg_mbit']) for v in helpers)
        cur = sum(self.veh[v.id]['be'] for v in helpers)
        self.kpi = {
            'em_latency_ms': em_rtt,
            'em_jitter_ms': round(statistics.pstdev(self.em_lat), 2) if len(self.em_lat) > 1 and evs else None,
            'em_tput_mbps': _mean(self.veh[v.id]['tput'] for v in evs),
            'em_loss_pct': _mean(self.veh[v.id]['loss'] for v in evs),
            'be_latency_ms': _mean(self.veh[v.id]['rtt'] for v in civ),
            'be_tput_mbps': round(sum(self.veh[v.id]['be'] for v in civ), 2),
            'suppression_pct': round((1 - cur / ref) * 100, 1) if helpers and ref > 0 else None,
            'policy_reaction_ms': round(self.reactions[-1], 1) if self.reactions else None,
            'policy_reaction_avg_ms': _mean(self.reactions),
            'emapt': self.emapt,
            'mqtt_latency_ms': _mean(x['mqtt'] for x in self.veh.values()),
            'msgs': self.msgs,
            'coverage_pct': round(100.0 * len(self.veh) / max(1, len(world.vehicles)), 1),
            'ecm_cells': sum(1 for r in world.rsus if r.ecm_since is not None),
        }

    # ── views ─────────────────────────────────────────────────────────────
    def annotate(self, stats, world):
        for v in stats['vehicles']:
            v['net'] = self.veh.get(v['id'])
        for r, c in zip(stats['rsus'], self.cells or [{}] * len(stats['rsus'])):
            r.update(c)
        sdnv = world.scenario == 'sdnv'
        flows = [{'match': f'udp_dst={p}', 'prio': 100, 'dp': 'all', 'note': 'priority port'}
                 for p in (5001, 5003, 6000)] if sdnv else []
        flows += [{'match': f'ipv4_src={v.id}', 'prio': 200, 'dp': 'all', 'note': 'ECM'}
                  for v in world.vehicles if v.siren and sdnv]
        stats['kpi'] = self.kpi
        stats['controller'] = {'modeled': True, 'datapaths': len(world.rsus) + 1, 'flows': flows,
                               'events': list(self.ctrl_events)}

    def close(self):
        pass


def _mean(it):
    xs = [x for x in it if x is not None]
    return round(statistics.mean(xs), 2) if xs else None


def _percentiles(sorted_ms):
    """EMAPT-50/90/100 exactly as measurements/emapt_analyze.py computes them."""
    n = len(sorted_ms)
    pick = lambda p: round(sorted_ms[max(0, int(p / 100.0 * n) - 1)], 2)
    return {'p50': pick(50), 'p90': pick(90), 'p100': round(sorted_ms[-1], 2)}
