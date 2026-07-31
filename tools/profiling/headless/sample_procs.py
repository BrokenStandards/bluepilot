#!/usr/bin/env python3
"""Sample CPU usage and py-spy profiles of running openpilot processes.

Designed for headless boxes (CI runners, cloud dev containers) where there is no
desktop, no GPU and no comma device attached. Point it at a running openpilot
(e.g. the MetaDrive sim from tools/sim) and it will:

  1. discover the managed processes by name,
  2. record a wall-clock CPU% budget per process over the sample window,
  3. capture a py-spy speedscope profile for each python process.

Usage:
  python3 tools/profiling/headless/sample_procs.py --duration 30 --out /tmp/prof
"""

import argparse
import json
import os
import shutil
import subprocess
import time

import psutil

# processes we never care about profiling
IGNORE = {"sample_procs.py", "py-spy"}


def _title(p: psutil.Process) -> str:
  """The manager setproctitle()s each child, so cmdline[0] is the process name.

  psutil's .name() reads /proc/pid/stat, which truncates to 15 characters.
  """
  try:
    cmdline = p.cmdline()
    if cmdline and cmdline[0]:
      return os.path.basename(cmdline[0])
    return p.name()
  except (psutil.NoSuchProcess, psutil.AccessDenied):
    return "?"


def find_openpilot_procs() -> dict[int, str]:
  """Map pid -> friendly name for the manager and everything under it."""
  managers = []
  for p in psutil.process_iter(["pid", "cmdline"]):
    cmdline = " ".join(p.info["cmdline"] or [])
    if "manager.py" in cmdline or "system.manager" in cmdline:
      managers.append(p)

  procs = {}
  for m in managers:
    try:
      procs[m.pid] = _title(m)
      for child in m.children(recursive=True):
        name = _title(child)
        if name not in IGNORE:
          procs[child.pid] = name
    except (psutil.NoSuchProcess, psutil.AccessDenied):
      continue
  return procs


def measure_cpu(pids: dict[int, str], duration: float) -> list[dict]:
  """Wall-clock CPU% per process over `duration` seconds."""
  handles = {}
  for pid, name in pids.items():
    try:
      p = psutil.Process(pid)
      p.cpu_percent(None)  # prime the counter
      handles[pid] = (name, p)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
      continue

  time.sleep(duration)

  rows = []
  for pid, (name, p) in handles.items():
    try:
      rows.append({
        "pid": pid,
        "name": name,
        "cpu_percent": round(p.cpu_percent(None), 2),
        "rss_mb": round(p.memory_info().rss / 1e6, 1),
        "num_threads": p.num_threads(),
      })
    except (psutil.NoSuchProcess, psutil.AccessDenied):
      continue
  return sorted(rows, key=lambda r: -r["cpu_percent"])


def record_pyspy(pids: dict[int, str], duration: float, out_dir: str) -> list[str]:
  """py-spy record each process into a speedscope profile."""
  py_spy = shutil.which("py-spy")
  if py_spy is None:
    print("py-spy not found, skipping stack sampling (pip install py-spy)")
    return []

  procs = []
  for pid, name in pids.items():
    out = os.path.join(out_dir, f"{name}-{pid}.speedscope.json")
    # --gil only samples threads actually holding the GIL. Without it the
    # openpilot processes look busy in whatever `time.sleep()` their parameter
    # polling threads are parked in, which is wall clock, not CPU.
    cmd = [py_spy, "record", "--format", "speedscope", "--duration", str(int(duration)),
           "--rate", "100", "--subprocesses", "--gil", "-o", out, "--pid", str(pid)]
    procs.append((name, out, subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)))

  written = []
  for name, out, p in procs:
    p.wait()
    if os.path.exists(out):
      written.append(out)
    else:
      print(f"  no profile for {name} (not a python process, or it exited)")
  return written


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--duration", type=float, default=30.0, help="sample window in seconds")
  parser.add_argument("--out", default="/tmp/op_profile", help="output directory")
  parser.add_argument("--no-pyspy", action="store_true", help="only collect CPU%%, skip stack sampling")
  args = parser.parse_args()

  os.makedirs(args.out, exist_ok=True)

  pids = find_openpilot_procs()
  if not pids:
    raise SystemExit("no openpilot processes found - is the manager running?")
  print(f"found {len(pids)} processes: {', '.join(sorted(pids.values()))}\n")

  spy = None
  if not args.no_pyspy:
    print(f"recording py-spy profiles for {args.duration}s...")
    spy = record_pyspy(pids, args.duration, args.out)

  print(f"measuring CPU for {args.duration}s...")
  rows = measure_cpu(pids, args.duration)

  summary = os.path.join(args.out, "cpu_summary.json")
  with open(summary, "w") as f:
    json.dump(rows, f, indent=2)

  total = sum(r["cpu_percent"] for r in rows)
  print(f"\n{'process':<28}{'CPU %':>10}{'RSS MB':>10}{'threads':>10}")
  print("-" * 58)
  for r in rows:
    if r["cpu_percent"] > 0.05:
      print(f"{r['name']:<28}{r['cpu_percent']:>10.2f}{r['rss_mb']:>10.1f}{r['num_threads']:>10}")
  print("-" * 58)
  print(f"{'TOTAL':<28}{total:>10.2f}")

  print(f"\nwrote {summary}")
  if spy:
    print(f"wrote {len(spy)} speedscope profiles to {args.out}")
    print("view them at https://speedscope.app (drag and drop, runs fully client side)")


if __name__ == "__main__":
  main()
