#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install 'bosdyn-client==5.2.0'
printf '%s\n' 'SDK installed. Run: PYTHONPATH=. .venv/bin/python examples/sdk_demo.py'
