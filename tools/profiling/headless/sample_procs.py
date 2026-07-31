#!/usr/bin/env python3
"""py-spy every Python process in the running openpilot stack, in parallel.

Writes one speedscope JSON per process to the output dir. Uses --gil so the
numbers approximate CPU time holding the GIL rather than wall clock -- without
it, every params/polling thread sitting in time.sleep() shows up as a hot spot
(see PROFILING_GUIDE.md pitfall #1). The flip side (pitfall #2): work done in C
with the GIL released (numpy, capnp, raylib, acados) is INVISIBLE here --
cross-check against cpu_sample.py before concluding a process is idle.

Usage: sample_procs.py OUT_DIR [duration_s] [--rate HZ]
"""
import os
import subprocess
import sys

MARKERS = ("selfdrive.", "system.", "sunnypilot.", "bluepilot.")


def python_stack_procs():
  procs = {}
  for pid in os.listdir("/proc"):
    if not pid.isdigit():
      continue
    try:
      with open(f"/proc/{pid}/cmdline", "rb") as f:
        argv = f.read().decode(errors="replace").split("\x00")
    except OSError:
      continue
    if not argv or "python" not in os.path.basename(argv[0]):
      continue
    mod = next((a for a in argv if any(m in a for m in MARKERS)), None)
    if mod:
      procs[int(pid)] = mod.replace("/", ".").removesuffix(".py")
  return procs


def main():
  out_dir = sys.argv[1]
  duration = sys.argv[2] if len(sys.argv) > 2 else "30"
  rate = sys.argv[sys.argv.index("--rate") + 1] if "--rate" in sys.argv else "100"
  os.makedirs(out_dir, exist_ok=True)

  procs = python_stack_procs()
  if not procs:
    print("no stack processes found -- is the sim running?", file=sys.stderr)
    sys.exit(1)

  jobs = []
  for pid, name in sorted(procs.items()):
    out = os.path.join(out_dir, f"{name}.speedscope.json")
    cmd = ["py-spy", "record", "--pid", str(pid), "--duration", duration, "--rate", rate,
           "--gil", "--format", "speedscope", "--output", out]
    jobs.append((name, subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)))
    print(f"sampling {name} (pid {pid}) for {duration}s")

  failed = 0
  for name, p in jobs:
    _, err = p.communicate()
    if p.returncode != 0:
      failed += 1
      print(f"FAILED {name}: {err.decode(errors='replace').strip().splitlines()[-1] if err else p.returncode}", file=sys.stderr)
  print(f"done: {len(jobs) - failed}/{len(jobs)} profiles in {out_dir}")


if __name__ == "__main__":
  main()
