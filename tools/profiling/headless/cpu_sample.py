#!/usr/bin/env python3
"""Per-process CPU split for the openpilot stack, from /proc, stdlib only.

Samples utime+stime deltas for every process whose cmdline looks like part of the
stack (python -m selfdrive.*/system.*/sunnypilot.*, or a known native binary), over
a window, and prints %CPU of one core sorted descending.

This is the container-side equivalent of what test_onroad.py::test_cpu_usage
computes from procLog on device -- loggerd is blocked in the sim launcher, so
there is no rlog to read here.

Usage: cpu_sample.py [duration_s] [--json OUT]
"""
import json
import os
import sys
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")

MARKERS = ("selfdrive.", "system.", "sunnypilot", "bluepilot", "run_bridge",
           "pandad", "locationd_llk", "modeld", "mapd", "manager.py")


def stack_pids():
  procs = {}
  for pid in os.listdir("/proc"):
    if not pid.isdigit():
      continue
    try:
      with open(f"/proc/{pid}/cmdline", "rb") as f:
        cmd = f.read().decode(errors="replace").replace("\x00", " ").strip()
    except OSError:
      continue
    if (any(m in cmd for m in MARKERS)
        and not any(x in cmd for x in ("cpu_sample", "claude", "py-spy", "sample_procs", "top_frames"))):
      # label: the python module if present, else the binary
      label = cmd
      for tok in cmd.split():
        if any(m in tok for m in MARKERS) and not tok.startswith("-"):
          label = os.path.basename(tok) if "/" in tok else tok
          break
      procs[int(pid)] = label
  return procs


def cpu_ticks(pid):
  try:
    with open(f"/proc/{pid}/stat") as f:
      parts = f.read().rsplit(")", 1)[1].split()
    return int(parts[11]) + int(parts[12])  # utime + stime
  except (OSError, IndexError):
    return None


def main():
  duration = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 30.0
  out_json = None
  if "--json" in sys.argv:
    out_json = sys.argv[sys.argv.index("--json") + 1]

  procs = stack_pids()
  t0 = time.monotonic()
  start = {pid: cpu_ticks(pid) for pid in procs}
  time.sleep(duration)
  elapsed = time.monotonic() - t0
  rows = []
  for pid, label in procs.items():
    a, b = start.get(pid), cpu_ticks(pid)
    if a is None or b is None:
      continue  # died mid-window
    pct = 100.0 * (b - a) / CLK_TCK / elapsed
    rows.append((pct, pid, label))
  rows.sort(reverse=True)

  print(f"# window={elapsed:.1f}s  ncpu={os.cpu_count()}")
  print(f"{'CPU%':>7}  {'PID':>7}  PROCESS")
  total = 0.0
  for pct, pid, label in rows:
    total += pct
    print(f"{pct:>6.1f}%  {pid:>7}  {label}")
  print(f"{total:>6.1f}%  {'':>7}  TOTAL")

  if out_json:
    with open(out_json, "w") as f:
      json.dump({"window_s": elapsed, "procs": [{"cpu_pct": p, "pid": pid, "label": l} for p, pid, l in rows]}, f, indent=2)


if __name__ == "__main__":
  main()
