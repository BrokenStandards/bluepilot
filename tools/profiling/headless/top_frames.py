#!/usr/bin/env python3
"""Print the hottest frames from a py-spy speedscope profile.

speedscope.app is nicer, but on a headless box you usually just want the top of
the list in the terminal.

Usage:
  python3 tools/profiling/headless/top_frames.py /tmp/op_profile/ui-1234.speedscope.json
  python3 tools/profiling/headless/top_frames.py /tmp/op_profile/*.speedscope.json --top 15
"""

import argparse
import json
import os
from collections import defaultdict


def summarize(path: str, top: int):
  with open(path) as f:
    prof = json.load(f)

  frames = prof["shared"]["frames"]
  self_time: dict[int, float] = defaultdict(float)
  total_time: dict[int, float] = defaultdict(float)
  grand_total = 0.0

  for p in prof["profiles"]:
    # py-spy emits "sampled" profiles: parallel lists of stacks and weights
    if p.get("type") != "sampled":
      continue
    for stack, weight in zip(p["samples"], p["weights"], strict=True):
      grand_total += weight
      if stack:
        self_time[stack[-1]] += weight
      for fid in set(stack):
        total_time[fid] += weight

  if grand_total == 0:
    print(f"{os.path.basename(path)}: no samples")
    return

  def label(fid: int) -> str:
    fr = frames[fid]
    name = fr.get("name", "?")
    file = os.path.basename(fr.get("file", "") or "")
    line = fr.get("line", "")
    return f"{name} ({file}:{line})" if file else name

  print(f"\n=== {os.path.basename(path)} — {grand_total:.1f} samples ===")
  print(f"{'self%':>7} {'total%':>8}  frame")
  for fid, t in sorted(self_time.items(), key=lambda kv: -kv[1])[:top]:
    print(f"{100 * t / grand_total:>7.1f} {100 * total_time[fid] / grand_total:>8.1f}  {label(fid)}")


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("profiles", nargs="+", help="speedscope json files")
  parser.add_argument("--top", type=int, default=20, help="frames to show per profile")
  args = parser.parse_args()

  for path in args.profiles:
    summarize(path, args.top)


if __name__ == "__main__":
  main()
