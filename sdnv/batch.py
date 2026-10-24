#!/usr/bin/env python3
"""batch.py — headless baseline-vs-SDNV sweep in simulation mode (≈300× real time).

Counterpart of sdnv-mininet-project's scale_run_multi_*.sh + ci_summary.py:
runs every (scenario × vehicle count × seed) combination, then writes
  results/batch_<ts>_runs.csv     one row per run
  results/batch_<ts>_summary.csv  mean ± 95 % CI per scenario × vehicles

Usage:
  python3 -m sdnv.batch --vehicles 10,20,30 --runs 5 --duration 900
"""

import argparse
import csv
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sdnv import config                      # noqa: E402
from sdnv.netmodel import NetModel           # noqa: E402
from sdnv.runner import Runner               # noqa: E402
from sdnv.world import World                 # noqa: E402

T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228}
METRICS = ('ev_kmh', 'ev_time_s', 'ev_red', 'ev_stops', 'em_latency_ms', 'em_tput_mbps', 'em_loss_pct',
           'be_latency_ms', 'suppression_pct', 'policy_reaction_ms', 'emapt_p90', 'mqtt_latency_ms', 'missions')


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return statistics.mean(xs) if xs else None


def one_run(scenario, vehicles, seed, duration, base):
    cfg = config.load(dict(base, scenario=scenario, vehicles=vehicles, seed=seed))
    world = World(cfg)
    rows = Runner(world, NetModel(cfg), lambda *_: None, cfg).run_headless(duration)
    ms = world.missions
    active = [r for r in rows if r['active_evs']]
    return {
        'scenario': scenario, 'vehicles': vehicles, 'seed': seed, 'missions': len(ms),
        'ev_kmh': _mean(m['kmh'] for m in ms), 'ev_time_s': _mean(m['time_s'] for m in ms),
        'ev_red': _mean(m['red'] for m in ms), 'ev_stops': _mean(m['stops'] for m in ms),
        **{k: _mean(r[k] for r in active) for k in ('em_latency_ms', 'em_tput_mbps', 'em_loss_pct',
                                                    'suppression_pct', 'policy_reaction_ms', 'emapt_p90')},
        'be_latency_ms': _mean(r['be_latency_ms'] for r in rows),
        'mqtt_latency_ms': _mean(r['mqtt_latency_ms'] for r in rows),
    }


def summarise(runs):
    out = []
    for key in sorted({(r['vehicles'], r['scenario']) for r in runs}):
        group = [r for r in runs if (r['vehicles'], r['scenario']) == key]
        row = {'vehicles': key[0], 'scenario': key[1], 'runs': len(group)}
        for m in METRICS:
            xs = [r[m] for r in group if r[m] is not None]
            row[m] = round(statistics.mean(xs), 3) if xs else None
            row[m + '_ci95'] = (round(T95.get(len(xs) - 1, 1.96) * statistics.stdev(xs) / len(xs) ** 0.5, 3)
                                if len(xs) > 1 else None)
        out.append(row)
    return out


def write(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description='Headless SDNV sweep (simulation mode)')
    ap.add_argument('--scenarios', default='baseline,sdnv')
    ap.add_argument('--vehicles', default='10,20,30')
    ap.add_argument('--runs', type=int, default=3)
    ap.add_argument('--duration', type=float, default=900.0, help='simulated seconds per run')
    ap.add_argument('--auto-dispatch', type=float, default=45.0)
    ap.add_argument('--out', default=os.path.join(config.ROOT, 'results'))
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    base = {'auto_dispatch': args.auto_dispatch}
    combos = [(s, int(n), seed) for n in args.vehicles.split(',') for s in args.scenarios.split(',')
              for seed in range(1, args.runs + 1)]
    runs, t0 = [], time.time()
    for i, (s, n, seed) in enumerate(combos, 1):
        runs.append(one_run(s, n, seed, args.duration, base))
        r = runs[-1]
        print(f'[{i}/{len(combos)}] {s:8s} v={n:<3d} seed={seed}  missions={r["missions"]:<3d} '
              f'EV {r["ev_kmh"] or 0:5.1f} km/h  red={r["ev_red"] or 0:4.2f}  '
              f'EV lat={r["em_latency_ms"] or 0:6.1f} ms  ({time.time() - t0:.0f}s)', flush=True)
    ts = time.strftime('%Y%m%d_%H%M%S')
    write(os.path.join(args.out, f'batch_{ts}_runs.csv'), runs)
    summary = summarise(runs)
    write(os.path.join(args.out, f'batch_{ts}_summary.csv'), summary)
    print(f'\n{"veh":>4} {"scenario":9} {"EV km/h":>9} {"red":>6} {"EV lat ms":>10} {"EV loss%":>9} '
          f'{"supp%":>7} {"EMAPT90":>8}')
    for r in summary:
        f = lambda k, w, d=1: f'{r[k]:{w}.{d}f}' if r[k] is not None else ' ' * (w - 1) + '—'
        print(f'{r["vehicles"]:>4} {r["scenario"]:9} {f("ev_kmh", 9)} {f("ev_red", 6, 2)} {f("em_latency_ms", 10)} '
              f'{f("em_loss_pct", 9)} {f("suppression_pct", 7)} {f("emapt_p90", 8)}')
    print(f'\nresults → {args.out}/batch_{ts}_{{runs,summary}}.csv')


if __name__ == '__main__':
    main()
