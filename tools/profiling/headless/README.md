# Headless profiling

Profiling bluepilot without a comma device, a desktop or a GPU. Everything here
runs in a plain x86_64 Linux container - a GitHub Actions runner, a cloud dev
container, or a local docker image.

The existing tooling in `tools/profiling/` assumes either a device
(`snapdragon/`, `ftrace.sh`, `watch-irqs.sh`) or a desktop with a browser
(`py-spy/profile.sh` shells out to `google-chrome`). This directory covers the
headless case.

## What works headless

| target | how | needs |
|---|---|---|
| full stack driving | `profile_sim.sh` (MetaDrive + Xvfb) | nothing extra |
| per-process CPU / RSS | `sample_procs.py` | a running manager |
| python stacks | `sample_procs.py` (py-spy) | `pip install py-spy` |
| UI render loop | `selfdrive/ui/tests/profile_onroad.py --headless` | a route log |
| C++ processes | `valgrind --tool=callgrind` | - |

`Xvfb` is already a vendored dependency (`pyproject.toml`), and
`selfdrive/test/setup_xvfb.sh` wires it up. MetaDrive renders through mesa's
software GL, so no `/dev/dri` is needed.

## Setup

```bash
git submodule update --init --recursive
git lfs pull                 # fonts and models are LFS; scons fails without them
./tools/setup_dependencies.sh
scons -j$(nproc)
uv pip install py-spy
```

The first `scons` compiles the driving model with tinygrad, which dominates the
build time on a small runner.

## Profiling the simulator

```bash
tools/profiling/headless/profile_sim.sh 45 30   # 45s warmup, 30s sample
```

This launches the manager in sim mode plus the MetaDrive bridge, waits for the
model to JIT and for openpilot to engage, then reports per-process CPU and
writes a [speedscope](https://speedscope.app) profile per python process to
`/tmp/op_profile`.

Numbers from a container are **relative, not absolute** - the comma 3X/4 is
aarch64 with a GPU/DSP running the model, so absolute CPU% will not match a
device. Use this to find algorithmic hot spots and to compare a change against
its baseline on the same box, not to predict on-device load.

## Profiling against an existing running stack

```bash
python3 tools/profiling/headless/sample_procs.py --duration 30 --out /tmp/prof
```

## Reading the output

Two different signals, don't mix them up:

- **`cpu_summary.json` / the printed table** is ground truth for how the CPU
  budget is split across processes. It comes from `/proc`.
- **The speedscope profiles** show where time goes *inside* a process. They are
  sampled with `py-spy --gil`, so a frame's weight is Python CPU time, not wall
  clock. This matters: nearly every openpilot process has a parameter polling
  thread sitting in `time.sleep()`, and a naive sample makes those threads look
  like hot spots. Time spent inside C extensions that release the GIL (raylib,
  capnp, numpy, acados) is *not* attributed - use the per-process CPU number to
  spot that case, i.e. a process burning CPU whose GIL profile looks empty.

## Baseline (bp-7.0, 2026-07-31)

4-core Xeon @ 2.8 GHz container, MetaDrive sim, engaged, 45s window. Two runs
agreed within 0.5%. Total 159% of 4 cores.

| process | CPU % | notes |
|---|---|---|
| modeld | 58.1 | tinygrad on CPU; runs on the GPU/DSP on device, ignore |
| ui | 49.2 | almost no GIL time - it is all inside raylib on software GL |
| card | 12.8 | 25% messaging send, ~6% `convert_carControlSP`, ~5% CAN parse |
| locationd | 11.1 | 43% in rednose `predict_and_observe`, ~10% in `_finite_check` |
| selfdrived | 8.4 | |
| controlsd | 6.4 | |
| everything else | < 3 each | |

Worth a look, in order:

1. **`locationd._finite_check`** runs `np.isfinite(x).all() and np.isfinite(P).all()`
   on the full state and covariance after *every* observation, and costs about a
   tenth of locationd. The covariance matrix dominates it. Checking it less
   often, or only checking the state vector, would be nearly free.
2. **`convert_carControlSP`** (`selfdrive/car/helpers.py`) runs at 100 Hz in card
   and measures 15 us/call here, 44% of it in a single `struct.to_dict()` that
   deep-converts the whole message just to rebuild dataclasses from it. Copying
   only the fields the dataclasses actually use avoids the round trip.
3. **The UI** is the second largest consumer but its cost is invisible to a GIL
   profiler, so all of it is in raylib. That is software GL in a container and
   says nothing about the device - this one needs a real comma 3X/4 or at least
   a box with a GPU before drawing conclusions.

Things that look hot but are not: every `params_thread`, `_params_refresh_worker`
and `prime_state._worker_thread`. Those are `time.sleep()` frames. Parameter
reads measure ~2 us, so even the UI's 35-reads-at-5 Hz refresh is under 0.05% of
a core.

## Caveats

- **Route replay needs network access to comma's log storage.** Anything built
  on `tools/lib/logreader` (`selfdrive/test/process_replay`, the UI profiler's
  default route, `tools/replay`) pulls segments from
  `commadataci.blob.core.windows.net` or the comma API. In a restricted
  environment those hosts may be blocked, in which case the sim is the way to
  generate load. A locally captured route works too - `LogReader` accepts a
  path.
- **modeld is slow on CPU.** Without a GPU the driving model runs far below
  20 Hz, so the sim drives sluggishly and modeld dominates every profile.
  Reading the other processes' numbers is still valid; comparing them against
  modeld is not.
- **Software GL caps the UI.** UI frame timings from a container measure the
  python and raylib CPU side, not the device's GPU path.
