#!/usr/bin/env bash
# Profile the full bluepilot stack headless, driven by the MetaDrive simulator.
#
#   OUT=/tmp/prof tools/profiling/headless/profile_sim.sh [WARMUP_S] [SAMPLE_S]
#
# Defaults: 210s warmup (MetaDrive takes 2-3 min to spawn under software GL and
# ignition goes high late -- sampling earlier profiles an offroad stack), 40s sample.
#
# Produces in $OUT:
#   cpu.json / stdout table   per-process CPU% over the window (/proc based)
#   <proc>.speedscope.json    py-spy --gil profile per Python process
#   manager.log, bridge.log   stack + sim logs for postmortem
#
# See tools/profiling/PROFILING_GUIDE.md for what these numbers do and do not mean
# in a container: relative/A-B use only; modeld and ui numbers are invalid here.
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$DIR/../../.." && pwd)"
cd "$ROOT"

WARMUP="${1:-210}"
SAMPLE="${2:-40}"
OUT="${OUT:-/tmp/prof}"
mkdir -p "$OUT"

# software-GL display for MetaDrive (and the UI, if unblocked)
if [ -z "$DISPLAY" ]; then
  source selfdrive/test/setup_xvfb.sh
fi

cleanup() {
  echo "== teardown"
  [ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null || true
  [ -n "$MANAGER_PID" ] && kill "$MANAGER_PID" 2>/dev/null || true
  sleep 3
  pkill -f 'system.manager' 2>/dev/null || true
  pkill -f 'run_bridge' 2>/dev/null || true
}
trap cleanup EXIT

echo "== launching stack (manager)"
./tools/sim/launch_openpilot.sh > "$OUT/manager.log" 2>&1 &
MANAGER_PID=$!

echo "== launching MetaDrive bridge"
./tools/sim/run_bridge.py > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!

echo "== warmup ${WARMUP}s (MetaDrive spawn + ignition + engage)"
sleep "$WARMUP"

if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "ERROR: bridge died during warmup -- see $OUT/bridge.log" >&2
  tail -20 "$OUT/bridge.log" >&2
  exit 1
fi

echo "== sampling ${SAMPLE}s: per-process CPU + py-spy (--gil)"
python3 "$DIR/cpu_sample.py" "$SAMPLE" --json "$OUT/cpu.json" > "$OUT/cpu.txt" &
CPU_PID=$!
python3 "$DIR/sample_procs.py" "$OUT" "$SAMPLE" || true
wait "$CPU_PID"

echo
cat "$OUT/cpu.txt"
echo
echo "== top frames (self time, GIL-held) -- full files in $OUT"
python3 "$DIR/top_frames.py" "$OUT"/*.speedscope.json --n 8 --min-pct 2 || true
