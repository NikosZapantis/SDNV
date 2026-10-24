"""config.py — single source of runtime parameters.

Defaults < environment (config/sdnv.env, exported by run.sh) < explicit overrides
(CLI flags or the dashboard config panel).
"""

import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# key: (env var, default)
PARAMS = {
    # scenario
    'map':                 ('SDNV_MAP', 'maps/athens.json'),
    'scenario':            ('SDNV_SCENARIO', 'sdnv'),          # baseline | sdnv
    'vehicles':            ('SDNV_NUM_VEHICLES', 20),          # total, emergency included
    'emergency':           ('SDNV_NUM_EMERGENCY', 2),
    'rsus':                ('SDNV_NUM_RSUS', 10),
    'rsu_range':           ('SDNV_RSU_RANGE', 400.0),          # metres (AP and station radio range)
    'signals':             ('SDNV_SIGNALS', 'major'),          # major = RSU + major junctions | rsu
    'seed':                ('SDNV_SEED', 7),
    'auto_dispatch':       ('SDNV_AUTO_DISPATCH', 75.0),       # s between automatic dispatches, 0 = off
    'speed':               ('SDNV_SPEED', 1.0),                # sim-time factor (sim mode only)
    # road priority (RSU logic)
    'green':               ('SDNV_GREEN', 18.0),
    'yellow':              ('SDNV_YELLOW', 3.0),
    'alert_range':         ('SDNV_ALERT_RANGE', 150.0),        # V2I yield alert ahead of an EV (sdnv)
    'siren_range':         ('SDNV_SIREN_RANGE', 25.0),         # audible-only awareness (baseline)
    'preempt_range':       ('SDNV_PREEMPT_RANGE', 250.0),      # signal pre-emption look-ahead
    # traffic profile (mirrors sdnv-mininet-project params_scale_v6.env)
    'emergency_rate':      ('EMERGENCY_RATE', '10m'),
    'bg_mbit':             ('SDNV_BG_MBIT', 8.0),              # background TCP offered per vehicle
    'helper_tcp_limit':    ('SDNV_HELPER_TCP_LIMIT_TOTAL_MBIT', 4.0),   # helper TCP cap per ECM cell (total)
    'cell_mbit':           ('SDNV_CELL_MBIT', 24.0),           # sim model: RSU cell capacity
    'base_rtt_ms':         ('SDNV_BASE_RTT_MS', 4.0),          # sim model
    'buffer_ms':           ('SDNV_BUFFER_MS', 180.0),          # sim model: FIFO standing-queue ceiling
    # emulation
    'backhaul_mbit':       ('SDNV_BACKHAUL_MBIT', 20.0),
    'rsu_qos':             ('SDNV_RSU_QOS', 1),
    'wifi_mode':           ('SDNV_WIFI_MODE', 'g'),
    'pos_hz':              ('SDNV_POS_HZ', 2.0),
    # processes / ports
    'hub_host':            ('SDNV_HUB_HOST', '127.0.0.1'),
    'hub_port':            ('SDNV_HUB_PORT', 7100),
    'dash_port':           ('SDNV_DASH_PORT', 7000),
    'ryu_ip':              ('RYU_IP', '127.0.0.1'),
    'ryu_port':            ('RYU_PORT', 6653),
    'ryu_api':             ('RYU_API', 'http://127.0.0.1:8080'),
    'venv':                ('SDNV_VENV', os.path.expanduser('~/sdv-venv')),
}

PORTS = {
    'emergency': int(os.environ.get('SDNV_EMERGENCY_PORT', 5001)),
    'best_effort': int(os.environ.get('SDNV_BEST_EFFORT_PORT', 5002)),
    'probe_hp': int(os.environ.get('SDNV_LATENCY_PORT', 5003)),
    'probe_be': int(os.environ.get('SDNV_BE_LATENCY_PORT', 5004)),
    'emapt': int(os.environ.get('SDNV_EMAPT_PORT', 6000)),
    'mqtt': 1883,
}


def rate_mbit(text):
    t = str(text).strip().lower().replace('bit', '')
    mult = {'k': 1e-3, 'm': 1.0, 'g': 1e3}.get(t[-1:], None)
    return float(t[:-1]) * mult if mult else float(t)


def load(overrides=None):
    cfg = {}
    for key, (env, default) in PARAMS.items():
        raw = os.environ.get(env)
        val = default
        if raw not in (None, ''):
            try:
                val = type(default)(raw)
            except ValueError:
                pass
        cfg[key] = val
    for key, val in (overrides or {}).items():
        if key in PARAMS and val not in (None, ''):
            cfg[key] = type(PARAMS[key][1])(val)
    if not os.path.isabs(cfg['map']):
        cfg['map'] = os.path.join(ROOT, cfg['map'])
    cfg['emergency_mbit'] = rate_mbit(cfg['emergency_rate'])
    cfg['vehicles'] = max(1, cfg['vehicles'])
    cfg['emergency'] = max(0, min(cfg['emergency'], cfg['vehicles']))
    if cfg['scenario'] not in ('baseline', 'sdnv'):
        cfg['scenario'] = 'sdnv'
    return cfg
