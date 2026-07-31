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

# Container-specific process blocks, appended to the sim launcher's own BLOCK list:
#  - soundd: crash-loops on OSError PortAudio library not found (no audio device here);
#    the restart churn pollutes the CPU numbers
#  - models_manager: mid-run it can activate a model bundle and flip
#    ModelRunnerTypeCache to tinygrad, which stops the stock modeld under us
#  - mapd/mapd_manager: the mapd binary is fetched on demand and can't download
#    here; the resulting process_not_running error event keeps the stack
#    un-engageable, so the bridge's auto-engage never fires
export BLOCK="${BLOCK},soundd,models_manager,mapd,mapd_manager"

# A stale ModelRunnerTypeCache (e.g. tinygrad, cached by a previous models_manager run
# with no bundle actually active) stops stock modeld and leaves NO model running --
# the stack then never becomes engageable and the bridge's auto-engage never fires.
# Pin the stock runner for the profile.
python3 - <<'PY'
from openpilot.common.params import Params
p = Params()
p.remove("ModelRunnerTypeCache")
p.remove("ModelManager_ActiveBundle")
print("model runner cache cleared")
PY

# Teardown must kill the whole process GROUPS: manager children setproctitle()
# themselves (argv[0] becomes e.g. "selfdrive.car.card"), so name-based pkill
# misses them, the leaked stack keeps running, and the next profile run gets two
# stacks fighting over msgq -- which both corrupts the numbers and blocks
# engagement. setsid gives each launch its own group; kill -- -PGID sweeps it.
cleanup() {
  echo "== teardown"
  [ -n "$BRIDGE_PID" ] && kill -- -"$BRIDGE_PID" 2>/dev/null || true
  [ -n "$MANAGER_PID" ] && kill -- -"$MANAGER_PID" 2>/dev/null || true
  sleep 3
  [ -n "$BRIDGE_PID" ] && kill -9 -- -"$BRIDGE_PID" 2>/dev/null || true
  [ -n "$MANAGER_PID" ] && kill -9 -- -"$MANAGER_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "== launching stack (manager)"
setsid ./tools/sim/launch_openpilot.sh > "$OUT/manager.log" 2>&1 &
MANAGER_PID=$!

echo "== launching MetaDrive bridge"
setsid ./tools/sim/run_bridge.py > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!

echo "== warmup ${WARMUP}s (MetaDrive spawn + ignition + engage)"
sleep "$WARMUP"

if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "ERROR: bridge died during warmup -- see $OUT/bridge.log" >&2
  tail -20 "$OUT/bridge.log" >&2
  exit 1
fi

echo "== engagement state"
python3 "$DIR/check_engaged.py" 2>&1 | tee "$OUT/engaged.txt" || true

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
