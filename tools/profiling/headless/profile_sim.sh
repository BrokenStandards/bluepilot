#!/usr/bin/env bash
# Profile a full openpilot stack driving in the MetaDrive simulator, headless.
#
# No desktop, no GPU and no comma device required - everything runs against an
# Xvfb virtual display with mesa software rendering. Intended for CI runners and
# cloud dev containers.
#
# Usage:
#   tools/profiling/headless/profile_sim.sh [warmup_seconds] [sample_seconds]
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$(cd "$DIR/../../../" && pwd)"
cd "$ROOT"

WARMUP=${1:-45}
SAMPLE=${2:-30}
OUT=${OUT:-/tmp/op_profile}

export PATH="$ROOT/.venv/bin:$PATH"
export PYTHONPATH="$(dirname "$ROOT"):$PYTHONPATH"

# openpilot's UI and MetaDrive both need a display; Xvfb provides one
source selfdrive/test/setup_xvfb.sh
export LIBGL_ALWAYS_SOFTWARE=1

mkdir -p "$OUT"

cleanup() {
  echo "--- shutting down ---"
  pkill -f run_bridge.py 2>/dev/null || true
  pkill -f "system.manager" 2>/dev/null || true
  pkill -f manager.py 2>/dev/null || true
  sleep 2
  pkill -9 -f manager.py 2>/dev/null || true
}
trap cleanup EXIT

echo "--- launching openpilot (sim mode) ---"
./tools/sim/launch_openpilot.sh > "$OUT/manager.log" 2>&1 &

echo "--- launching MetaDrive bridge ---"
./tools/sim/run_bridge.py > "$OUT/bridge.log" 2>&1 &

echo "--- warming up for ${WARMUP}s (model JIT, sim spawn, engagement) ---"
sleep "$WARMUP"

echo "--- sampling for ${SAMPLE}s ---"
python3 -u tools/profiling/headless/sample_procs.py --duration "$SAMPLE" --out "$OUT"

echo
echo "logs:     $OUT/manager.log, $OUT/bridge.log"
echo "profiles: $OUT/*.speedscope.json"
