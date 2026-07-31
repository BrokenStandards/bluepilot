# Profiling best practices for bluepilot

Companion to `PROFILING_GUIDE.md` (which covers *where* to run what and what the
numbers mean per environment). This document covers *how to measure without
lying to yourself*: the methodology comma and sunnypilot actually use, the
discipline each tool requires, and this fork's known gaps. Every claim carries a
reference; the reference list is at the bottom.

Written 2026-07-31 against `bp-7.0` @ `1bbda0e`, from upstream
commaai/openpilot master @ `a2ee4df` and sunnypilot master, verified by
fetching the cited sources.

---

## 1. The architecture of trustworthy measurement (comma's model)

Comma's performance work is a strict two-tier split [1][2]:

- **GitHub Actions gates functionality only** — plus coarse wall-clock
  timeouts. No CPU thresholds, no absolute numbers. Shared runners vary too
  much between jobs for thresholds not to flap.
- **Every absolute number is gated on the device farm** (Jenkins, ~10 devices
  continuously replaying routes, with continuous re-runs on master to expose
  flaky perf regressions): per-process CPU%, PSS memory, per-process *power
  in watts*, service timing jitter, UI draw time, model execution time [2][3][4].

The enabling design decision: **all of it is self-instrumentation that lands in
the rlog** — `procLog` at 0.5 Hz, `uiDebug.cpuTimeMillis`,
`modelV2.modelExecutionTime`, etc. `test_onroad.py` doesn't measure anything
live; it reads the log afterwards. Consequence worth internalizing: **any real
drive is a profile.** Pull an rlog off a device and you can run the exact CI
analysis on your laptop (`DEBUG=1` makes `test_onroad.py` skip the manager and
load the latest local segment) [3].

Follow the same split here: the container/sim harness (`headless/`) is for
A/B on the same box and for in-process attribution; absolute budgets live in
`selfdrive/test/test_onroad.py` and only mean something on hardware.

## 2. Warmup, sampling, and statistics

What upstream's device gate actually does, as discipline to copy [3]:

- **Wait for validity, not for time.** `test_onroad.py` starts its clock when
  `carState` first appears (30 s timeout); `test_power_draw.py` waits up to
  30 s for *valid messages AND power in the expected range* rather than
  sleeping a fixed interval [4]. Our sim harness equivalent: don't sample
  until the bridge reports `Engaged: True`.
- **Discard startup.** 25 s sample, first 8 s (`LOG_OFFSET`) dropped from
  every timing and memory analysis, computed per-service as
  `frequency × LOG_OFFSET` [3].
- **Gate with slack.** Per-process CPU fails at
  `max(expected × 1.8, expected + 5)`; service frequency must be within 3% of
  nominal; max/min interval ratio and relative standard deviation are bounded
  per service [3].
- **Multiple runs, same box.** pyperf's methodology (20 worker processes,
  warmup values discarded, instability flagged via stddev/mean and max/mean)
  is the reference for microbenchmarks [12]; for stack-level A/B, run baseline
  and candidate in the same session on the same machine — measurement bias
  from environment differences is a documented, published effect (Mytkowicz
  et al., "Producing Wrong Data Without Doing Anything Obviously Wrong",
  ASPLOS'09, cited by LLVM's benchmarking guide) [11]. Note the tooling
  disagreement on ASLR: LLVM says disable it for stability; pyperf says never
  disable it — randomize across runs instead. Either is defensible; *pick one
  and keep it fixed across the A and B you compare* [11][12].
- **Two independent windows minimum.** The seed guide's container baseline
  agreed within 0.5% per process across two runs — that's the sanity bar
  before believing any per-process delta.

## 3. Memory: PSS, not RSS

Since late 2025, upstream's proclogd logs PSS (`smaps_rollup`: Pss, Pss_Anon,
Pss_Shmem) and the memory report accounts in PSS [5]. RSS double-counts MSGQ's
shared-memory segments in every subscriber, which in a 20+ process pub/sub
system overstates memory dramatically and, worse, *attributes* it wrongly.
Use `selfdrive/debug/mem_usage.py` (already PSS-aware in this fork) and treat
any RSS-based comparison across processes as invalid.

## 4. Tool discipline

### py-spy (first-line Python profiler; comma's standard since 2021 [6])

- **Sampling rate: never 100 Hz on this stack.** py-spy defaults to 100 Hz [7]
  — exactly the control-loop rate. A sampler synchronized with the loop
  aliases: it can catch every iteration at the same phase and systematically
  over/under-represent whatever runs at that phase. Use an off-round rate
  (99 or 101; Brendan Gregg's tools conventionally use 49/99/97/997 for the
  same reason [9]). Our `sample_procs.py` defaults to 100 — pass `--rate 99`.
- **`--gil` shows GIL-holding time; that is both its value and its lie.** It
  removes the `time.sleep()` polling threads that otherwise dominate (guide
  pitfall #1), but the py-spy README states it "will miss activity in
  extensions that release the GIL while still active" [7] — numpy, capnp,
  raylib, acados. **Pair every `--gil` pass with the per-process CPU table**
  (`cpu_sample.py`); a high-CPU process with an empty `--gil` profile works in
  C — take it to `perf`, not py-spy. Upstream's own `profile.sh` doesn't pass
  `--gil` at all [8]; keep this fork's `--gil` rule, with the cross-check.
- `--subprocesses` on the manager PID covers the whole Python stack in one
  attach. `--native` adds C frames at higher overhead. `--nonblocking` trades
  pause-free sampling for occasional partial stacks. Attaching to a running
  PID needs ptrace rights (root, or `CAP_SYS_PTRACE` in containers) [7].
- 5 s of samples (upstream's default [8]) is enough for "what is this process
  doing", not for ranking frames within a few percent — use 30–60 s for that.

### cProfile (structure and call counts — never for benchmarking)

cProfile is deterministic: every call event is traced. The CPython docs state
the overhead lands on Python code and **not on C functions**, "so the C code
would seem faster than any Python one" [10]. On a Python+C stack like this one
it systematically mis-ranks, and its per-event dispatch cost accumulates
exactly in hot 100 Hz loops. Use it to answer "what calls what, how many
times" (`selfdrive/ui/tests/profile_onroad.py` is the in-tree example), then
switch to sampling profilers for time attribution. When viewing `.stats`:
tuna's README documents that the pstats format only records immediate-parent
data, so SnakeViz's reconstructed subtree timings can be wrong — tuna
deliberately renders only what the format actually contains [13].

### perf (the native side py-spy cannot see)

Canonical pipeline [9][14]:

```bash
perf record -F 99 -g -p <pid> -- sleep 30
perf script | stackcollapse-perf.pl | flamegraph.pl > out.svg
```

- perf sees kernel stacks and all GIL-released C work — it is the complement
  of `--gil`, not a substitute.
- The default frame-pointer unwinder produces "bogus call graphs" on
  `-fomit-frame-pointer` builds (perf's own documentation) — use
  `--call-graph dwarf` when stacks look truncated [14].
- For A/B: FlameGraph's differential mode (`--negate`) and consistent palette
  (`--cp`) exist precisely so two SVGs can be compared [9].

### Perfetto / ftrace (scheduler questions only)

When the question is "why did the 100 Hz loop miss its deadline" rather than
"what is it computing": tracebox records `sched_switch` + `sched_waking` with
ns accuracy; scheduling latency is the gap between the waking event and the
start of the `sched_slice`, and end-state `R+` ("Runnable (Preempted)")
directly identifies preempted iterations [15]. The in-tree device scripts are
`tools/profiling/perfetto/` (note `copy.sh`/`traces.sh` still point at the
dead `selfdrive/debug/profiling/` path). This is the right tool for the
`commIssue`/"process not communicating" class of problem — the June 2026
CPU-cap and UI-priority fixes (afdd528, 2f74cf1) were exactly this category.

### Import/startup time

`python -X importtime` prints per-module cumulative/self import cost, but its
own docs warn it may break with multi-threaded imports [10] — openpilot
processes spawn threads at import, so run it on single modules. The only
startup-time budget in the tree is `test_onroad.py`'s "manager publishes
`managerState` within 15 s" [3].

## 5. What sunnypilot adds (and doesn't)

- **No perf documentation exists** in sunnypilot (docs/, README_SP,
  CONTRIBUTING — grep-verified) [16]. Their device CI runs the same
  `test_onroad.py` with byte-identical budgets and **zero entries for their
  own added processes** — the fork inherited that gap (see §6).
- Their one added tool: `sunnypilot/tools/memory_profiler/mem_usage.py`
  (PR #1622) — per-segment memory/CPU trends + HTML report from route logs,
  present in this fork [17].
- **The recurring sunnypilot perf theme is removing per-frame Params polling**
  (PR #1802 "ui: remove per-frame param sync", #1534 "ui: param watcher",
  #1564 "common: system param watcher") [18] — the same class of fix this
  fork applied to the Ford car layer (see `bluepilot/selfdrive/car/
  bp_ford_settings.py`).
- Sync hazard: sunnypilot master moved statsd to
  `openpilot.sunnypilot.system.statsd` while this fork still budgets
  `system.statsd` — the next sync silently breaks that `PROCS` key [16].

## 6. This fork's CPU-budget gaps (device CI is currently not gating)

`test_onroad.py::PROCS` keys on `cmdline[0]`. On a BluePilot device today:

| process | why it breaks the gate |
|---|---|
| `soundd` → `selfdrive.ui.bp.soundd_bp` | PROCS only has `selfdrive.ui.soundd`; CPU lookup finds no procLog entry → `❌ NO METRICS FOUND ❌` → **test fails unconditionally on BP devices** |
| `locationd_llk` (native `./locationd`) | no PROCS key → coverage subtest fails unconditionally |
| `mapd_manager` (`sunnypilot.mapd.mapd_manager`, always_run) | no PROCS key → coverage subtest fails |
| `mapd` | NativeProcess launched via `bash -c`, so `cmdline[0]` is `bash` — unmatchable as specified |
| `modeld_tinygrad`, `statsd_sp`, `manage_sunnylinkd`, `sunnylink_registration_manager`, `backup_manager`, `bp_portal`, `bp_route_preprocessor`, `models_manager` | param-gated; no budgets; each needs a budget or a verified exclusion |

Until these get budgets (or explicit exclusions), **on-device CPU regression
testing for this fork does not work** — treat "device CI green" as *not*
implying CPU health. Fixing this table is the single highest-leverage
profiling-infrastructure change available.

Also known-broken on non-8-core boxes: `selfdrive/debug/live_cpu_and_temp.py`
hardcodes 8 cores.

## 7. Quick reference: what to run

Container (this box):

```bash
OUT=/tmp/prof uv run tools/profiling/headless/profile_sim.sh 240 45   # full-stack sim profile
python3 tools/profiling/headless/top_frames.py /tmp/prof/*.speedscope.json
uv run python selfdrive/debug/check_timings.py carState modelV2 controlsState  # live jitter
uv run selfdrive/ui/tests/profile_onroad.py --headless <local_rlog>   # UI logic under cProfile
```

Device:

```bash
pytest selfdrive/test/test_onroad.py -s          # the canonical gate (once §6 is fixed)
python selfdrive/debug/live_cpu_and_temp.py      # live procLog view (8-core assumption!)
tools/profiling/perfetto/record.sh               # scheduler traces
```

Offline, from any device rlog: `DEBUG=1 pytest selfdrive/test/test_onroad.py`,
`selfdrive/debug/mem_usage.py`, `sunnypilot/tools/memory_profiler/`,
`tools/jotpluggler` (supports `--stream` against a live device).

Built-in instrumentation you get for free in every log: `procLog` (0.5 Hz,
per-core + per-process CPU, PSS), `modelV2.modelExecutionTime`,
`longitudinalPlan.solverExecutionTime`, `longitudinalPlan.processingDelay`,
`uiDebug.cpuTimeMillis` / `drawTimeMillis`, `carState.cumLagMs` (Ratekeeper
lag), per-camera `timestampSof/Eof`, `deviceState` per-core usage and temps.
UI-only env hooks: `PROFILE_STARTUP`, `PROFILE_RENDER=N`, `SHOW_FPS`,
`STRICT_MODE` (kills the UI below 50% of target FPS).

---

## References

1. comma CI split: https://github.com/commaai/openpilot/blob/master/.github/workflows/tests.yaml ; https://github.com/commaai/openpilot/blob/master/Jenkinsfile
2. Device farm / testing closet: https://blog.comma.ai/dev-speed/ ; https://github.com/commaai/openpilot/issues/33059
3. Device perf gate: https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/test/test_onroad.py
4. Power gating + wait-for-valid warmup: https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/test/test_power_draw.py
5. PSS accounting: https://github.com/commaai/openpilot/blob/master/openpilot/system/proclogd.py ; https://github.com/commaai/openpilot/pull/36110 ; https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/test/mem_usage.py
6. py-spy adoption (over pyflame): https://github.com/commaai/openpilot/pull/22864 ; still first-line in 2025: https://github.com/commaai/openpilot/pull/35902 , https://github.com/commaai/openpilot/pull/36926
7. py-spy semantics (--gil, --idle, --native, --subprocesses, rate, ptrace): https://github.com/benfred/py-spy/blob/master/README.md
8. Upstream py-spy wrapper: https://github.com/commaai/openpilot/blob/master/tools/scripts/profiling/py-spy/profile.sh
9. Flame graphs, off-round sampling rates, differential mode: https://github.com/brendangregg/FlameGraph/blob/master/README.md ; https://github.com/iovisor/bcc/blob/master/tools/profile.py
10. cProfile C-code caveat; importtime multi-thread caveat: https://github.com/python/cpython/blob/3.12/Doc/library/profile.rst ; https://github.com/python/cpython/blob/main/Doc/using/cmdline.rst
11. Benchmarking discipline, measurement bias, ASLR: https://llvm.org/docs/Benchmarking.html (Mytkowicz et al., ASPLOS'09)
12. pyperf methodology: https://github.com/psf/pyperf (doc/ run_benchmark + system tune)
13. pstats parent-only limitation: https://github.com/nschloe/tuna/blob/main/README.md ; https://github.com/jiffyclub/snakeviz/blob/master/README.rst
14. perf record, --call-graph dwarf: https://github.com/torvalds/linux/blob/master/tools/perf/Documentation/perf-record.txt
15. Scheduler tracing: https://github.com/google/perfetto/blob/master/docs/data-sources/cpu-scheduling.md ; https://github.com/google/perfetto/blob/master/docs/quickstart/linux-tracing.md
16. sunnypilot state: https://raw.githubusercontent.com/sunnypilot/sunnypilot/master/openpilot/selfdrive/test/test_onroad.py ; https://raw.githubusercontent.com/sunnypilot/sunnypilot/master/Jenkinsfile
17. sunnypilot memory profiler: https://github.com/sunnypilot/sunnypilot/pull/1622
18. sunnypilot param-polling removals: https://github.com/sunnypilot/sunnypilot/pull/1802 , #1534, #1564
19. tools/profiling deletion upstream: https://github.com/commaai/openpilot/commit/62b97fabf79d848264c79f1fc44857b79851896f (now tools/scripts/profiling/)
20. Sim CI acceptance criteria (for re-enabling simulator_driving): https://github.com/commaai/openpilot/issues/30693
