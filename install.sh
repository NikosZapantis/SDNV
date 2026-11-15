#!/usr/bin/env bash
# install.sh — create the dashboard venv (SDNV_VENV, default ~/sdv-venv).
# Emulation prerequisites (Mininet-WiFi, Open vSwitch, Ryu, mosquitto) are Linux-only and not handled yet.
set -euo pipefail
cd "$(dirname "$0")"

VENV="${SDNV_VENV:-$HOME/sdv-venv}"
PYTHON="${PYTHON:-python3}"

"$PYTHON" -c 'import sys; sys.exit(sys.version_info < (3, 8))' || { echo "Python 3.8+ required"; exit 1; }
[ -x "$VENV/bin/python" ] || "$PYTHON" -m venv "$VENV"
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q -r requirements.txt
echo "venv ready: $VENV"
echo "next: ./run.sh sim"
