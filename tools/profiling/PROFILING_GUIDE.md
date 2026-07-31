# Profiling bluepilot: where to run what

Written against `bp-7.0` @ `1bbda0e`, 2026-07-31. This is a guide to the
*infrastructure* - how to get a trustworthy profile, in which environment, and
what each environment's numbers do and do not mean. It deliberately says nothing
about how to fix the code it measures.

Everything marked **verified** below was actually run in a 4-core x86_64 Linux
container during the session that wrote this. Everything marked **unverified**
is read from the repo and needs confirming on real hardware.

---

## 1. The short version

| environment | can it profile? | what its numbers are worth |
|---|---|---|
| cloud container / GH Actions runner | **yes, verified** | relative only - use for A/B against a baseline on the same box |
| Linux desktop with a GPU | yes, unverified | same as above, plus the UI becomes measurable |
| comma 3X / comma 4 over ssh | yes, unverified | **authoritative** - this is the only environment whose absolute numbers matter |
| offline, from a route log | yes, unverified here | authoritative *if* the log came off a device |

No desktop is required to profile the control stack. A desktop or device *is*
required to say anything about the UI.

---

## 2. Environment A - headless container or CI runner

**Verified.** Full stack builds, the MetaDrive simulator drives, openpilot
engages, all 23 onroad processes run.

Harness lives in `tools/profiling/headless/` (added on the
`claude/bp-7.0-profile-ahtsz4` branch). See its README for usage. In short:

```bash
git submodule update --init --recursive
git lfs pull                      # fonts and models are LFS; scons fails without them
./tools/setup_dependencies.sh
scons -j$(nproc)
uv pip install py-spy
tools/profiling/headless/profile_sim.sh 210 40    # warmup, sample
```

### Setup gotchas hit in practice

These cost real time; they are listed so the next session does not rediscover them.

1. **`git lfs pull` is mandatory, and not just for models.** scons fails on
   `selfdrive/assets/fonts/Audiowide-Regular.fnt` with `raylib failed to load
   font data` when the `.ttf` is still a 133-byte LFS pointer. The error does
   not mention LFS.
2. **`git submodule update --init --recursive`** - without it scons dies on
   `No tool module 'rednose_filter' found`.
3. **uv must be new enough to know the pinned Python.** `.python-version` pins
   3.12.13; uv 0.8.x fails with `No download found for request:
   cpython-3.12.13`. `uv self update` may be blocked (it calls
   `api.github.com`); `pip install -U uv` works.
4. **First scons compiles the driving model with tinygrad** and dominates build
   time on a small runner. Budget ~15 min on 4 cores.
5. **Xvfb is already vendored** (`pyproject.toml`) and wired up by
   `selfdrive/test/setup_xvfb.sh`. MetaDrive renders through mesa software GL,
   so no `/dev/dri` is needed. MetaDrive downloads its assets from
   `github.com/commaai/metadrive/releases` on first run.
6. **MetaDrive takes ~2-3 min to spawn** under software GL before ignition goes
   high. Warm up for at least 200 s before sampling or you will profile an
   offroad stack.

### What is valid here

- Per-process CPU split of the pure-Python control stack (card, controlsd,
  plannerd, locationd, selfdrived, radard, torqued, lagd, paramsd).
- In-process attribution for those, via py-spy.
- A/B comparison of a change against its baseline **on the same box**.

### What is not valid here

- **modeld.** Runs tinygrad on CPU. On the device it runs on the GPU/DSP.
  Ignore it entirely.
- **ui.** Burns ~50% CPU here with almost no GIL time, i.e. all of it is inside
  raylib on llvmpipe. This measures software rasterisation, not the device.
- **Anything absolute.** The device is aarch64 with 8 cores and different clocks.

### Network dependency to know about

Anything built on `tools/lib/logreader` - `selfdrive/test/process_replay`,
`tools/replay`, `selfdrive/ui/tests/profile_onroad.py`'s default route - pulls
segments from `commadataci.blob.core.windows.net` (see
`tools/lib/openpilotci.py`) or the comma API. In the session that wrote this,
that host was **blocked by egress policy**, so route replay was unavailable and
the simulator was used to generate load instead.

If your environment allows that host, replay-based profiling is the better tool
and you should prefer it - it replays real driving data instead of a synthetic
track. `LogReader` also accepts a local path, so a route copied off a device
works without any network at all.

---

## 3. Environment B - Linux desktop with a GPU

**Unverified**, but nothing in the repo suggests friction. Same setup as A. The
difference that matters:

- The UI becomes worth profiling, because raylib hits a real GL driver instead
  of llvmpipe. `selfdrive/ui/tests/profile_onroad.py` is the tool - it drives
  the real layout tree from a route log at 60 fps under cProfile, feeds a
  synthetic YUV buffer through VisionIPC, and writes a `.stats` file. It takes a
  route argument and honours `--headless`.
- MetaDrive spawns in seconds rather than minutes, so sim iteration is practical.
- `tools/profiling/py-spy/profile.sh` exists but shells out to `google-chrome`
  to view the SVG; on a headless box use `tools/profiling/headless/top_frames.py`
  or upload the speedscope JSON to speedscope.app.

A desktop is still *not* the device. Treat its numbers the same way as A's.

---

## 4. Environment C - the device (comma 3X / comma 4)

**Unverified in this session** - no hardware attached. This is where absolute
numbers come from.

### Getting a shell

- `op ssh` → `tools/scripts/ssh.py`
- `op adb` → `tools/scripts/adb_ssh.sh`, which forwards every cereal service
  port over adb (ports are derived from a FNV-1a hash of the service name) and
  then sshes in over a forwarded port. This is the one to use if you want to
  run `cereal` subscribers on your laptop against a live device.

Note comma devices are ssh-first; adb is a transport, not the primary interface.
The Jenkins device CI drives them purely over ssh (see `Jenkinsfile`,
`deviceStage`).

### The canonical device profiling harness

`selfdrive/test/test_onroad.py` is it, and it is already wired into Jenkins:

```
step("onroad tests", "pytest selfdrive/test/test_onroad.py -s", [timeout: 60])
```

It runs the manager for 25 s, then asserts three things:

- **CPU**, from the `procLog` messages in the resulting rlog, against the
  per-process budgets in the `PROCS` dict. Budget is `MAX_TOTAL_CPU = 350`
  across 8 cores. A process over `max(expected * 1.8, expected + 5)` fails.
- **Timings**, per service, against max/min ratio and relative standard
  deviation bounds in `TIMINGS`.
- **Log sizes**, per segment, against `LOGS_SIZE`.

It is marked `@pytest.mark.tici`, so it only runs on hardware.

`PROCS` doubles as the reference for what "normal" looks like on device. For
orientation, the entries most relevant to this fork's stack:

```
selfdrive.ui.ui                40.0     selfdrive.car.card             26.0
selfdrive.locationd.locationd  25.0     selfdrive.modeld.modeld        22.0
./pandad                       19.0     selfdrive.controls.controlsd   16.0
selfdrive.selfdrived.selfdrived 16.0    selfdrive.locationd.lagd       11.0
```

### Deeper device tooling already in the tree

- `selfdrive/debug/live_cpu_and_temp.py` - live per-process CPU and temps off
  the `procLog` stream. Works remotely against a device via the adb port
  forwarding above.
- `selfdrive/debug/cpu_usage_stat.py` - psutil sampling of named processes with
  min/max/running-average.
- `tools/profiling/perfetto/` - `build.sh` cross-compiles tracebox for arm64,
  `copy.sh` scps it to the device, `record.sh` records a scheduling trace,
  `traces.sh` pulls it back, `server.sh` runs trace_processor locally. This is
  the right tool for scheduler/IRQ latency questions.
- `tools/profiling/ftrace.sh` - workqueue events via `/sys/kernel/tracing`.
- `tools/profiling/watch-irqs.sh`.
- `tools/profiling/snapdragon/` - GPU/DSP profiling via Qualcomm's Snapdragon
  Profiler. **Its README is stale**: it refers to
  `selfdrive/debug/profiling/snapdragon/` and to `selfdrive/debug/adb.sh`,
  neither of which exists any more (the tree moved to `tools/profiling/`, and
  the adb helper is now `tools/scripts/adb_ssh.sh`). It also needs a Qualcomm
  developer account and a specific 2021.5 build. Budget time for this one.

Since the UI is the largest non-model consumer and its cost is invisible off
hardware, the Snapdragon profiler is probably the only way to answer UI
questions properly.

---

## 5. Offline analysis from a route log

This is the cheapest way to get device-accurate numbers without holding a
device, and it is underused.

`test_onroad.py::test_cpu_usage` does not measure anything live - it reads
`procLog` messages out of the rlog and differences the cpu-time counters. So any
rlog from a real drive can be analysed the same way, anywhere. `test_onroad.py`
already has a hook for this: set `DEBUG` in the environment and `setup_class`
skips running the manager and loads the most recent local segment instead.

`openpilot.tools.lib.log_time_series.msgs_to_time_series` plus
`tools/lib/logreader.LogReader` (which accepts a local path) is the general
form. `tools/jotpluggler` is the interactive version.

Recommended workflow for the optimisation branches: capture a route on device
before and after a change, pull the rlogs, compare `procLog` offline. No CI, no
sim, no software-GL caveats.

---

## 6. Recommendations for CI

### 6.1 Fix `test_onroad.py`'s process table for this fork

**Concrete and verified by inspection.** `PROCS` is keyed on `cmdline[0]`, i.e.
the module path. This fork changes and adds processes that are not in it:

- `soundd` runs `selfdrive.ui.bp.soundd_bp` when `is_bluepilot()`, but `PROCS`
  only has `selfdrive.ui.soundd`. The name-coverage subtest still passes on the
  substring `soundd`, but the CPU lookup finds no matching procLog entry and
  reports `❌ NO METRICS FOUND ❌`, which fails the test.
- Fork-added processes with no budget entry at all: `locationd_llk`,
  `modeld_tinygrad`, `mapd`, `mapd_manager`, `models_manager`, `statsd_sp`,
  `manage_sunnylinkd`, `sunnylink_registration_manager`, `backup_manager`,
  `bp_portal`, `bp_route_preprocessor`.

Which of those are actually running onroad depends on params
(`EnableWebRoutesServer` for the portal, sunnylink enablement, the tinygrad
model toggle), so the fix is to add budgets for the ones that do and confirm the
rest are correctly excluded. Until that happens, on-device CPU regression
testing for this fork is not actually working, and any onroad CI result should
be read with that in mind.

### 6.2 The simulator job is disabled

`.github/workflows/tests.yaml`, job `simulator_driving`:

```yaml
if: false  # FIXME: Started to timeout recently
```

The sim does work headless - this session ran it repeatedly. The timeout is
plausibly just the MetaDrive spawn under software GL (2-3 min) colliding with
the job's `timeout-minutes: 2`. Worth re-enabling with a realistic timeout
before writing it off. Note `process_replay` is also disabled (`if: false  #
disable process_replay for forks`).

### 6.3 Do not put performance gates in GitHub Actions

Shared runners vary enough between jobs that CPU thresholds will flap. Two
things that *are* worth doing in Actions:

- Keep the sim job as a **functional** gate: does the stack engage and drive.
  No timing assertions.
- Archive the profile as an artifact rather than asserting on it, so a human or
  a follow-up job can diff it.

Performance gates belong on the Jenkins device runners, where `test_onroad.py`
already lives.

### 6.4 If you want a CPU regression signal in Actions anyway

Run the same commit twice - baseline and candidate - in the *same* job on the
*same* runner and compare, rather than comparing against a stored number. That
cancels out most of the runner variance. Expect to need ~6 min per sample
(build cache + 200 s warmup + sample).

---

## 7. Pitfalls that will produce wrong answers

**Read this before trusting any profile.**

1. **py-spy without `--gil` measures wall clock, not CPU.** Nearly every
   openpilot process has a parameter-polling thread sitting in `time.sleep()`.
   In a first pass here, `params_thread`, `_params_refresh_worker` and
   `prime_state._worker_thread` all appeared as 6-12% "hot spots". They are
   sleeps. `tools/profiling/headless/sample_procs.py` now passes `--gil`.
2. **`--gil` hides work done outside the GIL.** raylib, capnp, numpy and acados
   all release it. A process burning CPU whose GIL profile looks empty is doing
   its work in C - cross-check against the per-process CPU number before
   concluding it is idle.
3. **Blocking message reads look like hot frames.** `SubMaster.update`,
   `recv_one`, `drain_sock_raw` and `send` in `cereal/messaging/__init__.py`
   dominate several profiles. Mostly they are waiting.
4. **Measure before believing a hypothesis.** `params.get_bool` measures ~2 us
   and `log_from_bytes(CarParams)` ~1.8 us on this box, which makes the
   parameter-polling loops that *looked* expensive cost under 0.05% of a core.
5. **Sample after warmup.** Ignition goes high late; sampling early profiles an
   offroad stack with none of the interesting processes running.

---

## 8. Baseline for comparison

Container, 4-core Xeon @ 2.8 GHz, MetaDrive sim, engaged, 45 s window. Two
independent runs agreed within 0.5% per process (159.4% and 159.8% total).

| process | CPU % |
|---|---|
| modeld | 58.1 |
| ui | 49.2 |
| card | 12.8 |
| locationd | 11.1 |
| selfdrived | 8.4 |
| controlsd | 6.4 |
| locationd_llk (native) | 2.7 |
| hardwared | 2.4 |
| plannerd | 2.3 |
| everything else | < 1.5 each |

Reproduce with:

```bash
OUT=/tmp/prof tools/profiling/headless/profile_sim.sh 210 40
python3 tools/profiling/headless/top_frames.py /tmp/prof/*.speedscope.json
```

Treat these as a fingerprint of *this* box, useful for A/B, not as a target.
Device targets are in `test_onroad.py::PROCS`.

---

## 9. Change made to get here

`selfdrive/test/helpers.py::set_params_enabled()` accepted openpilot's terms but
not sunnypilot's, while `system/hardware/hardwared.py:292` gates startup on
both. Every consumer of that helper - the MetaDrive sim included - sat offroad
forever with `"accepted_terms_sp": false` in the `Startup blocked` log line. The
fix accepts `HasAcceptedTermsSP` too.

This is worth knowing before debugging any other test that "hangs" offroad.

---

## 10. Addenda from the second profiling session (2026-07-31, container)

Operational gotchas hit while reproducing this guide's setup — the harness in
`headless/` now handles all of them automatically:

1. **A stale `ModelRunnerTypeCache` kills the profile.** If `models_manager`
   ever cached runner=tinygrad with no `ModelManager_ActiveBundle`, the stock
   `modeld` is stopped and nothing replaces it; the stack never becomes
   engageable and the bridge's auto-engage (`tools/sim/bridge/common.py:181`)
   never fires. Clear both params before launching.
2. **Block `soundd`, `models_manager`, `mapd`, `mapd_manager` in a container.**
   soundd crash-loops on missing PortAudio; models_manager can flip the model
   runner mid-profile; mapd's binary can't download and its
   `process_not_running` error event keeps the stack un-engageable.
3. **Manager children `setproctitle()` themselves** — argv[0] is
   `selfdrive.car.card`, not `python`. Any "find the Python processes" logic
   must go through `/proc/<pid>/exe`.
4. **uv shadowing:** an old `~/.local/bin/uv` silently wins over a
   pip-upgraded `/usr/local/bin/uv` and fails with `No interpreter found for
   Python 3.12.13`. `rm ~/.local/bin/uv` (or fix PATH) before `uv python
   install 3.12.13`.
5. **Upstream deleted `tools/profiling/` entirely** (commaai/openpilot
   62b97fab, "profiling is scripts quality", now `tools/scripts/profiling/`);
   sunnypilot inherited the deletion. This fork is the last holder of the full
   tree — expect future syncs to try to remove it, and re-point
   `perfetto/copy.sh` + `traces.sh` (they still reference the long-dead
   `selfdrive/debug/profiling/` paths).

Methodology, tool discipline, and the fork's CI budget gaps now live in
`BEST_PRACTICES.md` next to this file.
