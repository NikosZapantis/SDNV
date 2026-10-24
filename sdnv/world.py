"""world.py — road network, vehicle mobility and RSU priority logic.

One World instance is the single source of truth for mobility in both modes:
  sim  — the dashboard steps it in-process and a NetModel estimates the network
  emu  — emulator/orchestrator.py steps it and mirrors every vehicle position
         onto a Mininet-WiFi station, so radio association, tc policies and
         measured traffic follow the vehicles on the map.

RSU logic (scenario "sdnv"):
  * ECM      — an RSU serving an emergency vehicle (siren on) enters Emergency
               Control Mode: the EV gets the high-priority policy, every other
               vehicle in the cell becomes a throttled helper (sdnv-mininet v6).
  * pre-emption — signalised RSU intersections on the EV route within range
               switch to green for the EV approach and hold it until it passes.
  * V2I yield — vehicles in coverage ahead of the EV are alerted early, slow
               down and pull over so the EV can pass.
Scenario "baseline" keeps flat policies, fixed signal cycles and siren-only
(short range, delayed) awareness.

Pure standard library: imported by both the system Python (Mininet-WiFi) and
the venv Python (Flask / Ryu).
"""

import bisect
import heapq
import json
import math
import random

DT = 0.1                 # physics step (s)
CAR_LEN = 4.5
LANE_OFF = 1.8           # right-hand traffic: draw vehicles right of the centreline
PULL_OFF = 2.6           # extra lateral offset when yielding / parked
LOOK = 140.0             # car-following look-ahead (m)
STOP_LINE = 7.0          # stop line distance before the intersection centre
MIN_GREEN, ALL_RED = 3.0, 1.5
KINDS = ('car', 'ambulance', 'police')
PREFIX = {'car': 'CAR', 'ambulance': 'AMB', 'police': 'POL'}
# IDM parameters: a_max, b_comf, s0, T
IDM_CAR = (1.4, 2.0, 2.0, 1.2)
IDM_EV = (2.6, 3.0, 1.5, 0.8)


def _heading(dx, dy):
    return math.degrees(math.atan2(dx, dy)) % 360.0


def _angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


class Edge:
    __slots__ = ('id', 'u', 'v', 'pts', 'cum', 'length', 'vmax', 'lanes', 'rank', 'name',
                 'veh', 'h_in', 'h_out')

    def __init__(self, idx, u, v, pts, vmax, lanes, rank, name):
        self.id, self.u, self.v, self.pts = idx, u, v, pts
        self.vmax, self.lanes, self.rank, self.name = max(vmax, 4.0), lanes, rank, name
        self.cum = [0.0]
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x2 - x1, y2 - y1))
        self.length = max(self.cum[-1], 1.0)
        self.veh = []
        self.h_out = _heading(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
        self.h_in = _heading(pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1])

    def at(self, s):
        """(x, y, ux, uy) at arc length s."""
        s = min(max(s, 0.0), self.cum[-1])
        i = min(bisect.bisect_right(self.cum, s) - 1, len(self.pts) - 2)
        seg = (self.cum[i + 1] - self.cum[i]) or 1e-9
        (x1, y1), (x2, y2) = self.pts[i], self.pts[i + 1]
        k = (s - self.cum[i]) / seg
        return x1 + (x2 - x1) * k, y1 + (y2 - y1) * k, (x2 - x1) / seg, (y2 - y1) / seg


class Map:
    def __init__(self, path):
        with open(path, encoding='utf-8') as f:
            d = json.load(f)
        self.name, self.bbox, self.size = d['name'], d['bbox'], d['size']
        self.lat0, self.lon0 = d['origin']
        self.kx, self.ky = d['scale']
        self.attribution = d.get('attribution', '')
        self.nodes = [tuple(p) for p in d['nodes']]
        self.out = [[] for _ in self.nodes]
        self.inc = [[] for _ in self.nodes]
        self.edges = []
        for i, (u, v, spd, lanes, rank, ni, mid) in enumerate(d['edges']):
            pts = [self.nodes[u]] + [tuple(p) for p in mid] + [self.nodes[v]]
            e = Edge(i, u, v, pts, spd, lanes, rank, d['names'][ni])
            self.edges.append(e)
            self.out[u].append(e)
            self.inc[v].append(e)
        self.pois = d.get('pois', [])
        self.rsu_nodes = d.get('rsus')

    def ll(self, x, y):
        return self.lat0 + y / self.ky, self.lon0 + x / self.kx

    def route(self, src, dst):
        """Fastest path (list of edges) by free-flow travel time."""
        if src == dst:
            return []
        dist, prev, pq = {src: 0.0}, {}, [(0.0, src)]
        while pq:
            d, n = heapq.heappop(pq)
            if n == dst:
                break
            if d > dist[n]:
                continue
            for e in self.out[n]:
                nd = d + e.length / e.vmax
                if nd < dist.get(e.v, 1e18):
                    dist[e.v], prev[e.v] = nd, e
                    heapq.heappush(pq, (nd, e.v))
        if dst not in prev:
            return None
        path, n = [], dst
        while n != src:
            e = prev[n]
            path.append(e)
            n = e.u
        return path[::-1]


class Signal:
    """Two-phase signal at an RSU intersection with emergency pre-emption."""

    def __init__(self, node, inc, green, yellow, rng):
        self.node, self.green, self.yellow = node, green, yellow
        self.group = self.groups(inc)
        self.phase, self.state, self.timer = rng.randint(0, 1), 'G', rng.uniform(0, green)
        self.preempt = None          # (vehicle id, group)

    @staticmethod
    def groups(inc):
        """Split approaches into two phases by axis (N–S vs E–W relative to the first approach)."""
        axis = inc[0].h_in
        return {e.id: (0 if abs((e.h_in - axis + 90.0) % 180.0 - 90.0) < 45.0 else 1) for e in inc}

    @staticmethod
    def signalisable(inc):
        return bool(inc) and len(set(Signal.groups(inc).values())) == 2

    def step(self, dt):
        self.timer += dt
        want = self.preempt[1] if self.preempt else None
        if self.state == 'G':
            if (want is not None and want != self.phase and self.timer >= MIN_GREEN) or \
               (want is None and self.timer >= self.green):
                self.state, self.timer = 'Y', 0.0
        elif self.state == 'Y' and self.timer >= self.yellow:
            self.state, self.timer = 'R', 0.0
        elif self.state == 'R' and self.timer >= ALL_RED:
            self.phase = want if want is not None else 1 - self.phase
            self.state, self.timer = 'G', 0.0

    def color(self, eid):
        return self.state if self.group.get(eid, self.phase) == self.phase and self.state != 'R' else 'R'


class RSU:
    __slots__ = ('idx', 'id', 'node', 'x', 'y', 'range', 'signal', 'evs', 'members', 'ecm_since', 'ecm_seq')

    def __init__(self, idx, node, xy, rng_m, signal):
        self.idx, self.id, self.node = idx, f'RSU-{idx + 1}', node
        self.x, self.y = xy
        self.range, self.signal = rng_m, signal
        self.evs, self.members, self.ecm_since, self.ecm_seq = [], [], None, 0


class Vehicle:
    __slots__ = ('idx', 'id', 'kind', 'ev', 'node', 'edge', 's', 'v', 'route', 'ri', 'factor', 'siren',
                 'parked', 'yield_until', 'notice', 'battery', 'dist', 'rsu', 'policy', 'mission',
                 'x', 'y', 'ux', 'uy', 'still', 'alert_seq', 'alert_rsu')

    def __init__(self, idx, vid, kind):
        self.idx, self.id, self.kind, self.ev = idx, vid, kind, kind != 'car'
        self.node = f'sta{idx + 1}'          # Mininet-WiFi station name in emu mode
        self.s = self.v = 0.0
        self.route, self.ri, self.edge = [], 0, None
        self.siren = self.parked = False
        self.yield_until, self.notice = -1.0, None
        self.battery, self.dist = 100.0, 0.0
        self.rsu, self.policy, self.mission = None, 'bg', None
        self.x = self.y = self.ux = self.uy = 0.0
        self.still, self.alert_seq, self.alert_rsu = 0.0, 0, None
        self.factor = 1.0


class World:
    def __init__(self, cfg):
        self.cfg = cfg
        self.rng = random.Random(cfg['seed'])
        self.map = Map(cfg['map'])
        self.t = 0.0
        self.scenario = cfg['scenario']
        self.events, self.alerts, self.missions = [], [], []
        self.counters = {'handovers': 0, 'yields': 0, 'preemptions': 0, 'dispatches': 0, 'ecm': 0}
        self._mission_id = 0
        self.signals = {}
        self.rsus = self._place_rsus(int(cfg['rsus']), float(cfg['rsu_range']))
        self._place_signals()
        rsu_at = {r.node: r.idx for r in self.rsus}
        self.approaches = [(rsu_at.get(n, -1), e) for n in self.signals for e in self.map.inc[n]]
        self.sig_name = {}
        for n in self.signals:
            roads = ' × '.join(sorted({e.name for e in self.map.inc[n] if e.name})[:2])
            self.sig_name[n] = self.rsus[rsu_at[n]].id if n in rsu_at else (roads or f'junction {n}')
        self.vehicles = []
        n_ev = int(cfg['emergency'])
        for i in range(int(cfg['vehicles'])):
            self.add('ambulance' if i < n_ev and i % 2 == 0 else 'police' if i < n_ev else 'car')
        self.next_dispatch = float(cfg['auto_dispatch']) * 0.25 if cfg['auto_dispatch'] else None
        self._associate()

    # ── setup ─────────────────────────────────────────────────────────────
    def _place_rsus(self, k, rng_m):
        """Greedy max-coverage of road length (weighted by road class) from real intersections."""
        m = self.map
        if m.rsu_nodes:
            chosen = list(m.rsu_nodes)[:k]
        else:
            samples, seen = [], set()
            for e in m.edges:
                key = (min(e.u, e.v), max(e.u, e.v), int(e.length))
                if key in seen:
                    continue
                seen.add(key)
                n = max(1, int(e.length // 25))
                for i in range(n):
                    x, y, _, _ = e.at((i + 0.5) * e.length / n)
                    samples.append((x, y, (1 + e.rank) * e.length / n))
            cands = [i for i in range(len(m.nodes))
                     if len({e.u for e in m.inc[i]} | {e.v for e in m.out[i]}) >= 3]
            r2 = rng_m * rng_m
            cover = {c: [j for j, (x, y, _) in enumerate(samples)
                         if (x - m.nodes[c][0]) ** 2 + (y - m.nodes[c][1]) ** 2 <= r2] for c in cands}
            bonus = {c: 1.15 if Signal.signalisable(m.inc[c]) else 1.0 for c in cands}
            covered, chosen = [False] * len(samples), []
            for _ in range(k):
                best, gain = None, 0.0
                for c in cands:
                    if c in chosen or any(math.dist(m.nodes[c], m.nodes[o]) < 0.75 * rng_m for o in chosen):
                        continue
                    g = sum(samples[j][2] for j in cover[c] if not covered[j]) * bonus[c]
                    if g > gain:
                        best, gain = c, g
                if best is None:
                    break
                chosen.append(best)
                for j in cover[best]:
                    covered[j] = True
        rsus = []
        for i, node in enumerate(chosen):
            sig = None
            if Signal.signalisable(m.inc[node]):
                sig = self.signals[node] = Signal(node, m.inc[node], self.cfg['green'], self.cfg['yellow'], self.rng)
            rsus.append(RSU(i, node, m.nodes[node], rng_m, sig))
        return rsus

    def _place_signals(self):
        """Fixed-time signals at major junctions too (pre-emptable over the RSU backhaul)."""
        if self.cfg.get('signals', 'major') != 'major':
            return
        m = self.map
        cands = [n for n in range(len(m.nodes)) if n not in self.signals and Signal.signalisable(m.inc[n])
                 and max(e.rank for e in m.inc[n]) >= 2
                 and len({e.u for e in m.inc[n]} | {e.v for e in m.out[n]}) >= 3]
        cands.sort(key=lambda n: -sum(e.rank for e in m.inc[n]))
        for n in cands:
            if all(math.dist(m.nodes[n], m.nodes[s]) >= 120.0 for s in self.signals):
                self.signals[n] = Signal(n, m.inc[n], self.cfg['green'], self.cfg['yellow'], self.rng)

    def add(self, kind='car', n=1):
        for _ in range(n):
            idx = len(self.vehicles)
            seq = sum(1 for v in self.vehicles if v.kind == kind) + 1
            v = Vehicle(idx, f'{PREFIX[kind]}-{seq:02d}' if kind == 'car' else f'{PREFIX[kind]}-{seq}', kind)
            v.factor = self.rng.uniform(0.85, 1.1)
            edges, weights = self.map.edges, [e.length for e in self.map.edges]
            for _try in range(200):
                e = self.rng.choices(edges, weights)[0]
                s = self.rng.uniform(4.0, max(4.5, e.length - 4.0))
                if all(abs(o.s - s) > 12.0 for o in e.veh):
                    break
            v.edge, v.s = e, s
            e.veh.append(v)
            e.veh.sort(key=lambda o: o.s)
            self._reroute(v, self._random_node(far_from=e.v))
            self._place(v)
            self.vehicles.append(v)

    def _random_node(self, far_from=None, min_dist=0.0):
        nodes = self.map.nodes
        for _ in range(50):
            n = self.rng.randrange(len(nodes))
            if far_from is None or math.dist(nodes[n], nodes[far_from]) >= min_dist:
                return n
        return self.rng.randrange(len(nodes))

    def _reroute(self, v, dst):
        path = self.map.route(v.edge.v, dst)
        if not path:
            path = self.map.route(v.edge.v, self._random_node()) or []
        v.route, v.ri = [v.edge] + path, 0

    # ── commands ──────────────────────────────────────────────────────────
    def log(self, msg, lvl='info'):
        self.events.append({'t': round(self.t, 1), 'lvl': lvl, 'msg': msg})

    def find(self, vid):
        return next((v for v in self.vehicles if v.id == vid), None)

    def set_scenario(self, scenario):
        if scenario in ('baseline', 'sdnv') and scenario != self.scenario:
            self.scenario = scenario
            for v in self.vehicles:
                if v.mission:
                    v.mission['scenario'] = 'mixed'
            self.log(f'Scenario switched to {scenario.upper()}', 'warn')

    def dispatch(self, vid=None):
        v = self.find(vid) if vid else None
        if v is None:
            idle = [o for o in self.vehicles if o.ev and not o.siren and not o.parked]
            if not idle:
                self.log('Dispatch ignored: no idle emergency vehicle', 'warn')
                return None
            v = min(idle, key=lambda o: (sum(1 for mi in self.missions if mi['id'] == o.id), o.idx))
        if not v.ev or v.siren:
            return None
        here = self.map.nodes[v.edge.v]
        hospitals = [p for p in self.map.pois if p['type'] == 'hospital'
                     and math.dist(self.map.nodes[p['node']], here) > 700]
        if v.kind == 'ambulance' and hospitals:
            poi = self.rng.choice(hospitals)
            dst, label = poi['node'], poi['name']
        else:
            dst = self._random_node(far_from=v.edge.v, min_dist=900)
            label = 'incident ' + (self.map.out[dst][0].name if self.map.out[dst] else f'#{dst}')
        v.parked = False
        self._reroute(v, dst)
        length = (v.edge.length - v.s) + sum(e.length for e in v.route[1:])
        self._mission_id += 1
        v.siren = True
        v.mission = {'n': self._mission_id, 'id': v.id, 'dest': label, 'start': self.t, 'dist_m': round(length),
                     'scenario': self.scenario, 'stops': 0, 'red': 0, 'stopped': False, 'sig': None}
        self.counters['dispatches'] += 1
        self.log(f'{v.id} dispatched → {label} ({length / 1000:.2f} km)', 'ev')
        return v

    def recall(self, vid):
        v = self.find(vid)
        if v and v.siren:
            v.siren, v.mission = False, None
            self.log(f'{v.id} mission cancelled', 'warn')

    def engine(self, vid, on=None):
        v = self.find(vid)
        if v:
            v.parked = (not v.parked) if on is None else (not on)
            if v.parked and v.siren:
                self.recall(vid)
            self.log(f'{v.id} engine {"OFF (parked)" if v.parked else "ON"}')

    # ── simulation step ──────────────────────────────────────────────────
    def step(self, dt=DT):
        self.t += dt
        for sig in self.signals.values():
            sig.step(dt)
        self._associate()
        self._priority()
        acc = [self._accel(v) for v in self.vehicles]
        occupied = set()
        for v, a in zip(self.vehicles, acc):
            self._move(v, a, dt)
            occupied.add(v.edge)
        for e in occupied:
            e.veh.sort(key=lambda o: o.s)
        if self.next_dispatch is not None and self.t >= self.next_dispatch:
            self.next_dispatch = self.t + float(self.cfg['auto_dispatch'])
            if not any(v.siren for v in self.vehicles):
                self.dispatch()

    def _associate(self):
        for v in self.vehicles:
            cur = v.rsu
            if cur is not None and (v.x - self.rsus[cur].x) ** 2 + (v.y - self.rsus[cur].y) ** 2 <= self.rsus[cur].range ** 2:
                continue
            best, bd = None, 1e18
            for r in self.rsus:
                d = (v.x - r.x) ** 2 + (v.y - r.y) ** 2
                if d <= r.range ** 2 and d < bd:
                    best, bd = r.idx, d
            if best != cur:
                v.rsu = best
                if cur is not None and best is not None:
                    self.counters['handovers'] += 1
                if v.siren:
                    self.log(f'{v.id} handover {self.rsus[cur].id if cur is not None else "—"} → '
                             f'{self.rsus[best].id if best is not None else "no coverage"}')

    def _ahead(self, v, horizon):
        """Yield (distance, edge, route index) for v's current and upcoming edges within horizon."""
        dist, ri = v.edge.length - v.s, v.ri
        while True:
            e = v.route[ri] if ri < len(v.route) else v.edge
            yield dist, e, ri
            ri += 1
            if ri >= len(v.route) or dist >= horizon:
                return
            dist += v.route[ri].length

    def _priority(self):
        sdnv = self.scenario == 'sdnv'
        sirens = [v for v in self.vehicles if v.siren]
        # ECM per RSU cell
        for r in self.rsus:
            r.evs, r.members = [], []
        for v in self.vehicles:
            if v.rsu is not None:
                self.rsus[v.rsu].members.append(v)
                if v.siren:
                    self.rsus[v.rsu].evs.append(v)
        for r in self.rsus:
            on = sdnv and bool(r.evs)
            if on and r.ecm_since is None:
                r.ecm_since, r.ecm_seq = self.t, r.ecm_seq + 1
                self.counters['ecm'] += 1
                self.log(f'{r.id}: ECM ON for {", ".join(e.id for e in r.evs)} '
                         f'({len(r.members) - len(r.evs)} helpers throttled)', 'ecm')
            elif not on and r.ecm_since is not None:
                self.log(f'{r.id}: ECM OFF after {self.t - r.ecm_since:.1f}s', 'ecm')
                r.ecm_since = None
        for v in self.vehicles:
            if v.siren:
                v.policy = 'ev' if sdnv else 'flat'
            elif sdnv and v.rsu is not None and self.rsus[v.rsu].ecm_since is not None:
                v.policy = 'helper'
            else:
                v.policy = 'bg'
            # emergency awareness broadcast whenever an EV joins a cell (both scenarios → EMAPT)
            if v.siren and v.rsu is not None and v.alert_rsu != v.rsu:
                v.alert_rsu, v.alert_seq = v.rsu, v.alert_seq + 1
                helpers = [o for o in self.rsus[v.rsu].members if o is not v]
                self.alerts.append({'ev': v.id, 'node': v.node, 'seq': v.alert_seq, 'rsu': v.rsu,
                                    't': self.t, 'helpers': [o.id for o in helpers]})
            elif not v.siren:
                v.alert_rsu = None
        # signal pre-emption (V2I: EV must be connected to an RSU)
        wanted = {}
        if sdnv:
            for ev in sirens:
                if ev.rsu is None:
                    continue
                for dist, e, _ri in self._ahead(ev, self.cfg['preempt_range']):
                    sig = self.signals.get(e.v)
                    if sig and dist <= self.cfg['preempt_range'] and e.v not in wanted:
                        wanted[e.v] = (ev.id, sig.group[e.id])
        for node, sig in self.signals.items():
            req = wanted.get(node)
            if req and (sig.preempt is None or sig.preempt[0] != req[0]):
                self.counters['preemptions'] += 1
                self.log(f'Signal {self.sig_name[node]} pre-empted for {req[0]}', 'sig')
            sig.preempt = req
        # yielding: V2I alert (sdnv, in coverage) or siren-only awareness (baseline / no coverage)
        noticed = set()
        for ev in sirens:
            v2i = sdnv and ev.rsu is not None
            horizon = self.cfg['alert_range'] if v2i else self.cfg['siren_range']
            for dist, e, _ri in self._ahead(ev, horizon):
                base = dist - e.length          # distance from the EV to the start of edge e
                for o in e.veh:
                    gap = base + o.s
                    if o is ev or o.ev or o.parked or gap <= -CAR_LEN or gap > horizon:
                        continue
                    if v2i and o.rsu is None and gap > self.cfg['siren_range']:
                        continue
                    noticed.add(o)
                    if o.notice is None:
                        o.notice = self.t
                    delay = 0.0 if v2i else 1.5
                    if self.t - o.notice >= delay:
                        if o.yield_until < self.t:
                            self.counters['yields'] += 1
                        o.yield_until = self.t + 1.5
        for v in self.vehicles:
            if v not in noticed and v.yield_until < self.t:
                v.notice, v.yield_until = None, -1.0

    def _ignore(self, v, o):
        return o.parked or (v.siren and not o.siren and o.yield_until > self.t)

    def _accel(self, v):
        if v.parked:
            return -4.0 if v.v > 0 else 0.0
        a_max, b, s0, T = IDM_EV if v.siren else IDM_CAR
        e = v.edge
        v0 = e.vmax * (1.3 if v.siren else v.factor) + (2.0 if v.siren else 0.0)
        if v.yield_until > self.t:
            v0 = min(v0, 2.5)
        gap, lead = 1e9, 0.0
        lst = e.veh
        for o in lst[lst.index(v) + 1:]:
            if not self._ignore(v, o):
                gap, lead = o.s - v.s - CAR_LEN, o.v
                break
        if gap > 1e8:
            for dist, cur, ri in self._ahead(v, LOOK):
                if dist > LOOK:
                    break
                sig = self.signals.get(cur.v)
                if sig is not None:
                    col = sig.color(cur.id)
                    stop = dist - STOP_LINE
                    if col != 'G' and stop > -1.0:
                        can_clear = col == 'Y' and stop < v.v * v.v / (2 * b)
                        creep = v.siren and v.v < 2.0 and stop < 3.0   # EV crosses red cautiously
                        if not can_clear and not creep:
                            if v.mission and stop < 30 and v.mission['sig'] != cur.v:
                                v.mission['sig'] = cur.v           # EV forced to brake for a red
                                v.mission['red'] += 1
                            gap, lead = max(stop, 0.1), 0.0
                            break
                nxt = v.route[ri + 1] if ri + 1 < len(v.route) else None
                if nxt is None:
                    break
                if _angle_diff(cur.h_in, nxt.h_out) > 50:
                    v0 = min(v0, math.sqrt(6.0 ** 2 + 2 * b * max(dist - 4.0, 0.0)))
                found = False
                for o in nxt.veh:
                    if o is not v and not self._ignore(v, o):
                        gap, lead, found = dist + o.s - CAR_LEN, o.v, True
                        break
                if found:
                    break
        gap = max(gap, 0.1)
        s_star = s0 + max(0.0, v.v * T + v.v * (v.v - lead) / (2 * math.sqrt(a_max * b)))
        acc = a_max * (1 - (v.v / v0) ** 4 - (s_star / gap) ** 2)
        return max(-9.0, min(acc, a_max))

    def _move(self, v, a, dt):
        pv = v.v
        v.v = max(0.0, v.v + a * dt)
        ds = (pv + v.v) * 0.5 * dt
        v.s += ds
        # EV energy model from Software-Defined-Vehicles-Mininet-MQTT (rolling + aero drag, 20 % regen)
        kmh, km = v.v * 3.6, ds / 1000.0
        v.dist += km
        drain = (0.10 + 0.000104 * kmh * kmh) * km if not v.parked else 0.001 * dt / 2.0
        if v.v < pv and pv > 0:
            drain *= 1.0 - 0.20 * (pv - v.v) / pv
        v.battery = max(0.0, v.battery - drain)
        if v.battery < 8.0 and not v.siren:
            v.battery = 100.0                      # swap/recharge so long runs keep going
        if v.mission:
            moving = v.v > 1.0
            if not moving and not v.mission['stopped']:
                v.mission['stops'] += 1
            v.mission['stopped'] = not moving
        while v.s >= v.edge.length:
            v.s -= v.edge.length
            if v.ri + 1 >= len(v.route):
                self._arrive(v)
            nxt = v.route[v.ri + 1] if v.ri + 1 < len(v.route) else None
            if nxt is None:                       # defensive: no route → stay at the end of the edge
                v.s = v.edge.length - 0.1
                break
            v.edge.veh.remove(v)
            v.ri += 1
            v.edge = nxt
            nxt.veh.append(v)
        v.still = v.still + dt if v.v < 0.1 and not v.parked else 0.0
        if v.still > 120.0 and not self._at_red(v):
            self._respawn(v)
        self._place(v)

    def _at_red(self, v):
        sig = self.signals.get(v.edge.v)
        return sig is not None and sig.color(v.edge.id) != 'G' and v.edge.length - v.s < 60

    def _respawn(self, v):
        v.edge.veh.remove(v)
        e = self.rng.choices(self.map.edges, [x.length for x in self.map.edges])[0]
        v.edge, v.s, v.v, v.still = e, e.length * 0.5, 0.0, 0.0
        e.veh.append(v)
        e.veh.sort(key=lambda o: o.s)
        self._reroute(v, self._random_node(far_from=e.v))
        self.log(f'{v.id} was gridlocked — respawned', 'warn')

    def _arrive(self, v):
        if v.mission:
            m = v.mission
            dur = self.t - m['start']
            rec = {'n': m['n'], 'id': v.id, 'dest': m['dest'], 'scenario': m['scenario'], 'dist_m': m['dist_m'],
                   'time_s': round(dur, 1), 'stops': m['stops'], 'red': m['red'],
                   'kmh': round(m['dist_m'] / max(dur, 1e-6) * 3.6, 1)}
            self.missions.append(rec)
            del self.missions[:-50]
            self.log(f'{v.id} arrived at {m["dest"]} in {int(dur // 60)}:{int(dur % 60):02d} '
                     f'[{m["scenario"]}] — {m["stops"]} stops, {m["red"]} red signals', 'ev')
            v.siren, v.mission = False, None
        self._reroute(v, self._random_node(far_from=v.edge.v, min_dist=400))

    def _place(self, v):
        v.x, v.y, v.ux, v.uy = v.edge.at(v.s)

    # ── views ─────────────────────────────────────────────────────────────
    def latlon(self, v):
        off = LANE_OFF + (PULL_OFF if v.parked or v.yield_until > self.t else 0.0)
        return self.map.ll(v.x + v.uy * off, v.y - v.ux * off)

    def describe(self):
        m, roads, seen = self.map, [], set()
        for e in m.edges:
            key = (min(e.u, e.v), max(e.u, e.v), int(e.length))
            if key not in seen:
                seen.add(key)
                roads.append([e.rank, [[round(c, 6) for c in m.ll(x, y)] for x, y in e.pts]])
        appr = []
        for ridx, e in self.approaches:
            x, y, ux, uy = e.at(e.length - STOP_LINE)
            appr.append([ridx] + [round(c, 6) for c in m.ll(x + uy * LANE_OFF, y - ux * LANE_OFF)])
        s, w, n, ea = m.bbox
        return {
            'map': {'name': m.name, 'bounds': [[s, w], [n, ea]], 'attribution': m.attribution},
            'roads': roads,
            'rsus': [{'id': r.id, 'll': [round(c, 6) for c in m.ll(r.x, r.y)], 'range': r.range,
                      'signal': r.signal is not None} for r in self.rsus],
            'approaches': appr,
            'pois': [{'name': p['name'], 'type': p['type'], 'll': [round(c, 6) for c in m.ll(*m.nodes[p['node']])]}
                     for p in m.pois],
        }

    def frame(self):
        vs = []
        for v in self.vehicles:
            lat, lon = self.latlon(v)
            flags = (1 if v.siren else 0) | (2 if v.yield_until > self.t else 0) | \
                    (4 if v.parked else 0) | (8 if v.v < 0.5 else 0)
            vs.append([v.id, round(lat, 6), round(lon, 6), round(_heading(v.ux, v.uy)), round(v.v * 3.6, 1),
                       flags, -1 if v.rsu is None else v.rsu, KINDS.index(v.kind)])
        lights = []
        for _ri, e in self.approaches:
            sig = self.signals[e.v]
            c = sig.color(e.id)
            # lower-case g = green held by an emergency pre-emption for this approach
            lights.append('g' if c == 'G' and sig.preempt and sig.preempt[1] == sig.group[e.id] else c)
        return {'t': round(self.t, 1), 'scenario': self.scenario, 'v': vs, 'L': ''.join(lights),
                'ecm': [r.idx for r in self.rsus if r.ecm_since is not None]}

    def status(self, v):
        if v.parked:
            return 'parked'
        if v.siren:
            return 'emergency'
        if v.yield_until > self.t:
            return 'yielding'
        return 'stopped' if v.v < 0.5 else 'driving'

    def stats(self):
        routes = {}
        for v in self.vehicles:
            if v.siren:
                pts = [self.latlon(v)]
                for e in v.route[v.ri:]:
                    pts.extend(self.map.ll(x, y) for x, y in e.pts[1:])
                routes[v.id] = [[round(a, 5), round(b, 5)] for a, b in pts]
        active = [{'n': v.mission['n'], 'id': v.id, 'dest': v.mission['dest'], 'scenario': v.mission['scenario'],
                   'dist_m': v.mission['dist_m'], 'time_s': round(self.t - v.mission['start'], 1),
                   'stops': v.mission['stops'], 'red': v.mission['red'], 'active': True}
                  for v in self.vehicles if v.mission]
        return {
            't': round(self.t, 1),
            'scenario': self.scenario,
            'vehicles': [{'id': v.id, 'kind': v.kind, 'node': v.node, 'speed': round(v.v * 3.6, 1),
                          'battery': round(v.battery, 1), 'dist_km': round(v.dist, 3),
                          'rsu': None if v.rsu is None else self.rsus[v.rsu].id, 'policy': v.policy,
                          'status': self.status(v), 'siren': v.siren, 'road': v.edge.name} for v in self.vehicles],
            'rsus': [{'id': r.id, 'n': len(r.members), 'evs': [e.id for e in r.evs], 'ecm': r.ecm_since is not None,
                      'ecm_s': round(self.t - r.ecm_since, 1) if r.ecm_since is not None else 0,
                      'signal': (r.signal.state + str(r.signal.phase)) if r.signal else None,
                      'preempt': r.signal.preempt[0] if r.signal and r.signal.preempt else None} for r in self.rsus],
            'missions': active + self.missions[::-1][:12],
            'routes': routes,
            'counters': dict(self.counters),
        }

    def drain(self):
        ev, al = self.events, self.alerts
        self.events, self.alerts = [], []
        return ev, al
