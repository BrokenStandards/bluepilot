# Headless full-stack profiling

Profiles the whole bluepilot stack in a container or CI runner, using the
MetaDrive simulator to generate onroad load. No GPU, no device, no route logs
needed. Read `../PROFILING_GUIDE.md` first — it defines what these numbers are
worth (relative / A-B use only) and the pitfalls that make profiles lie.

## One-time setup

```bash
git submodule update --init --recursive
git lfs pull                      # fonts + models are LFS; scons fails without them
pip3 install -U uv                # uv must know the pinned python (.python-version)
uv sync
./tools/setup_dependencies.sh
uv run scons -j$(nproc)           # first build compiles the model with tinygrad, ~15 min on 4 cores
uv pip install py-spy
```

## Run

```bash
OUT=/tmp/prof uv run tools/profiling/headless/profile_sim.sh 210 40   # warmup, sample
```

Produces in `$OUT`:

| file | what |
|---|---|
| `cpu.txt` / `cpu.json` | per-process CPU% over the window (`/proc`-based; loggerd is blocked in the sim so there is no rlog/procLog to read) |
| `<proc>.speedscope.json` | py-spy `--gil` profile per Python process — view at speedscope.app or with `top_frames.py` |
| `manager.log`, `bridge.log` | stack + sim logs for postmortem |

```bash
python3 tools/profiling/headless/top_frames.py /tmp/prof/*.speedscope.json
```

## The three tools, separately

- `cpu_sample.py [dur] [--json f]` — per-process CPU split, any time the stack is up.
- `sample_procs.py OUT [dur] [--rate hz]` — py-spy every stack Python process in parallel.
- `top_frames.py *.speedscope.json` — top self-time frames per process.

## Interpreting

1. Start from `cpu.txt` — that is ground truth for *where* CPU goes.
2. Use the speedscope profiles for *why*, remembering `--gil` semantics:
   a process with high CPU% but an empty GIL profile does its work in C
   (numpy / capnp / raylib / acados) — profile that with `perf`, not py-spy.
3. `modeld` (tinygrad on CPU here, GPU/DSP on device) and `ui` (llvmpipe here,
   Adreno on device) numbers are **invalid** in a container. Ignore them.
4. A/B only against a baseline captured on the same box in the same session.
