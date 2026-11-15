#!/usr/bin/env bash
# run.sh — sim | emu | stop | batch
#
#   ./run.sh sim [--vehicles 30 --rsus 12 --speed 2 ...]   dashboard + in-process simulation (background)
#   ./run.sh stop                                          stop the dashboard
#   ./run.sh batch [--vehicles 10,20,30 --runs 5 ...]      headless baseline-vs-SDNV sweep
#   ./run.sh emu                                           Mininet-WiFi emulation (Linux, root) — not implemented yet
#
# Parameters: config/sdnv.env (exported if present) or environment variables, see sdnv/config.py.
set -euo pipefail
cd "$(dirname "$0")"

VENV="${SDNV_VENV:-$HOME/sdv-venv}"
PID=logs/dashboard.pid
LOG=logs/dashboard.log
mkdir -p logs
if [ -f config/sdnv.env ]; then set -a; . config/sdnv.env; set +a; fi

stop() {
  if [ -f "$PID" ] && kill -0 "$(cat "$PID")" 2>/dev/null; then
    kill "$(cat "$PID")" && echo "dashboard stopped"
  fi
  rm -f "$PID"
}

mode="${1:-sim}"
shift || true
case "$mode" in
  sim)
    [ -x "$VENV/bin/python" ] || { echo "venv not found at $VENV, run ./install.sh first"; exit 1; }
    stop >/dev/null
    nohup "$VENV/bin/python" -m dashboard.app --mode sim "$@" >"$LOG" 2>&1 &
    echo $! >"$PID"
    for _ in $(seq 1 50); do
      url=$(grep -o 'http://[^ ]*' "$LOG" 2>/dev/null | head -1) && [ -n "$url" ] && break
      kill -0 "$(cat "$PID")" 2>/dev/null || { cat "$LOG"; exit 1; }
      sleep 0.2
    done
    echo "SDNV dashboard (sim) → ${url:-see $LOG}"
    echo "logs: $LOG · stop: ./run.sh stop"
    ;;
  stop)
    stop
    ;;
  batch)
    exec python3 -m sdnv.batch "$@"
    ;;
  emu)
    echo "emu mode needs Linux + Mininet-WiFi (root); the orchestrator (emulator/) is not implemented yet."
    echo "The dashboard can already attach to it: $VENV/bin/python -m dashboard.app --mode emu"
    exit 1
    ;;
  *)
    sed -n '2,9p' "$0"
    exit 1
    ;;
esac
