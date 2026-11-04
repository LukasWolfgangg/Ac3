#!/usr/bin/env bash
# Create a local virtualenv and install the analysis dependencies.
# Usage: ./setup.sh   then:  source .venv/bin/activate && cd scripts && python r250c.py
set -euo pipefail
cd "$(dirname "$0")"
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
echo "Done. Activate with: source .venv/bin/activate"
echo "Scripts must be run from scripts/ (they load arm.py and arm_pts.csv from the working directory)."
