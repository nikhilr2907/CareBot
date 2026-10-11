#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$ROOT/tools/lift_calibrator/server.py" --port 8767 "$@"
