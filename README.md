# SDNV Live Testbed (work in progress)

A live map of software-defined networked vehicles. It merges two earlier projects:

- **sdnv-mininet-project**: the Mininet-WiFi testbed. RSU access points, a Ryu controller that prioritises emergency traffic, vehicle-side `tc` policies (baseline vs SDNV) and the KPIs (latency, throughput, suppression, policy reaction time, EMAPT).
- **Software-Defined-Vehicles-Mininet-MQTT**: the Flask + Socket.IO dashboard, the in-vehicle MQTT agents and the EV battery model.

The goal is one dashboard with a real OpenStreetMap map. Vehicles drive on real roads, RSUs cover the area, and emergency vehicles get priority both on the road and in the network.

## Status

| Part | State |
|---|---|
| OSM map builder (`tools/fetch_map.py`) and bundled map of central Athens (`maps/athens.json`) | done |
| World engine: roads, car-following, RSUs, signals, priority logic (`sdnv/world.py`) | done |
| Network model for simulation mode (`sdnv/netmodel.py`) | done |
| Real-time runner and orchestrator↔dashboard link (`sdnv/runner.py`, `sdnv/hub.py`) | done |
| Headless baseline-vs-SDNV sweep (`sdnv/batch.py`) | done |
| Ryu controller, vehicle `tc` policy script, in-vehicle MQTT agent | next |
| Emulation orchestrator (Mininet-WiFi driven by the world engine) | next |
| Dashboard (Leaflet map, KPIs, charts, controls) | next |
| `run.sh` / `install.sh` automation | next |

## How it fits together

```
                 ┌────────────── sdnv/world.py (single source of truth) ──────────────┐
                 │ OSM road graph · IDM car-following · RSU coverage & handover        │
                 │ signals + emergency pre-emption · V2I yielding · ECM per RSU cell   │
                 └───────────────┬───────────────────────────────┬────────────────────┘
          sim mode (no root)     │                               │  emu mode (root, Mininet-WiFi)
     sdnv/netmodel.py (modelled) │                               │  orchestrator → station positions,
                                 ▼                               ▼  tc policies, Ryu ECM rules (measured)
                       sdnv/runner.py ── frames / stats / events ──► dashboard (Flask + Socket.IO)
                                                    (emu: via sdnv/hub.py TCP JSON-lines)
```

The same world engine drives both modes, so the map behaves the same way in each:

- **sim mode** runs anywhere (WSL, laptop) with network KPIs from an analytical model. They're flagged `modeled`.
- **emu mode** mirrors each vehicle onto a Mininet-WiFi station, so association, `tc` policies and traffic are real and the KPIs are measured.

## RSU priority logic (`sdnv/world.py`)

**SDNV scenario:**

- **ECM (Emergency Control Mode):** an RSU serving an emergency vehicle with its siren on gives the EV the high-priority policy. Every other vehicle in that cell becomes a throttled helper (the sdnv-mininet v6 logic, applied per cell).
- **Signal pre-emption:** signals on the EV's route within range switch to green for its approach and hold until it passes. RSUs are connected over the backhaul, so this also works at major junctions that have no RSU.
- **V2I yielding:** vehicles in coverage ahead of the EV are alerted early, slow down and pull over.

**Baseline scenario:** flat policies, fixed signal cycles, and only siren-range awareness (short range, delayed reaction).

## What runs today

Pure Python 3.8+ standard library. No root, no extra packages.

```bash
# Baseline vs SDNV sweep (simulation, ≈300× real time) → results/batch_<ts>_{runs,summary}.csv
python3 -m sdnv.batch --vehicles 10,20,30 --runs 5 --duration 900

# Build a map for another area from OpenStreetMap (bbox = S,W,N,E)
python3 tools/fetch_map.py --bbox 37.9705,23.7220,37.9875,23.7520 \
    --name "Athens — Omonia / Syntagma / Evangelismos" --out maps/athens.json
```

The bundled map covers about 2.6 × 1.9 km with 313 junctions, 38 km of directed roads and 5 hospitals. With the defaults it gets 10 RSUs at 400 m range (about 91% road coverage) and 35 signals.

Parameters are environment variables with defaults in `sdnv/config.py` (for example `SDNV_NUM_VEHICLES`, `SDNV_NUM_RSUS`, `SDNV_RSU_RANGE`, `SDNV_SCENARIO`, `EMERGENCY_RATE`, `SDNV_BG_MBIT`, `SDNV_HELPER_TCP_LIMIT_TOTAL_MBIT`). They mirror `params_scale_v6.env` from sdnv-mininet-project.

## Next steps

1. `controller/sdnv_controller.py`: install the priority-port rules proactively. In sdnv-mininet-project they're installed in `packet_in`, but IPv4 already matches the priority-1 NORMAL rule, so those packets never reach the controller. Also: per-EV ECM rules with OVS queue tagging, flow statistics, and the REST API from the MQTT project.
2. `vehicle/policy.sh`: the baseline, background, SDNV emergency and helper profiles as one atomic `tc -batch` per change.
3. `vehicle/sdv_agent.py`: MQTT telemetry, RTT probes on the HP (5003) and BE (5004) ports, and an EMAPT burst on UDP/6000.
4. `emulator/`: the orchestrator plus the Mininet-WiFi backend (positions, handover, policies, iperf measurements).
5. `dashboard/`: Leaflet map with live vehicles, RSU coverage, signals, flows and EV routes, plus KPI tiles, charts and controls.
6. `run.sh` (`sim | emu | stop | batch`) and `install.sh`.

## Map data

© OpenStreetMap contributors, ODbL.
