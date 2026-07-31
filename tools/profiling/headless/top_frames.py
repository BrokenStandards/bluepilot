#!/usr/bin/env python3
"""Summarize speedscope JSONs from sample_procs.py: top self-time frames per process.

Self time = samples where the frame is at the top of the stack. Speedscope
'sampled' profiles store stacks as frame-index lists with per-sample weights.

Usage: top_frames.py FILE.speedscope.json [...] [--n TOP_N] [--min-pct PCT]
"""
import json
import sys
from collections import defaultdict


def summarize(path, top_n, min_pct):
  with open(path) as f:
    data = json.load(f)
  frames = data["shared"]["frames"]
  name = path.split("/")[-1].replace(".speedscope.json", "")

  for prof in data["profiles"]:
    if prof.get("type") != "sampled":
      continue
    self_w = defaultdict(float)
    total_w = 0.0
    for stack, weight in zip(prof["samples"], prof["weights"], strict=True):
      total_w += weight
      if stack:
        self_w[stack[-1]] += weight
    if total_w == 0:
      print(f"\n== {name} [{prof.get('name', '')}]: no samples (fully idle under --gil, or C-bound)")
      continue
    print(f"\n== {name} [{prof.get('name', '')}]  ({total_w:.0f} weight)")
    rows = sorted(self_w.items(), key=lambda kv: -kv[1])[:top_n]
    for idx, w in rows:
      pct = 100.0 * w / total_w
      if pct < min_pct:
        break
      fr = frames[idx]
      loc = f"{fr.get('file', '?')}:{fr.get('line', '?')}"
      # trim long paths to the interesting tail
      for root in ("/bluepilot/", "/openpilot/", "site-packages/"):
        if root in loc:
          loc = loc.split(root, 1)[1]
          break
      print(f"  {pct:5.1f}%  {fr['name']}  ({loc})")


def main():
  args = [a for a in sys.argv[1:] if not a.startswith("--")]
  top_n = int(sys.argv[sys.argv.index("--n") + 1]) if "--n" in sys.argv else 12
  min_pct = float(sys.argv[sys.argv.index("--min-pct") + 1]) if "--min-pct" in sys.argv else 1.0
  for path in args:
    try:
      summarize(path, top_n, min_pct)
    except Exception as e:
      print(f"\n== {path}: unreadable ({e})", file=sys.stderr)


if __name__ == "__main__":
  main()
