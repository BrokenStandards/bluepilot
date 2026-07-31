# Full-stack profile + optimization verdicts — 2026-07-31

Environment: 4-core x86_64 container, MetaDrive sim, full stack, stock modeld.
Two independent 45 s windows (`/tmp/prof3`, `/tmp/prof4` in that session);
per-process agreement ±0.8% on the control stack. Run 3 engaged (cycling with
`locationdTemporaryError` soft-disables — box oversubscription, the sim itself
costs ~1.6 cores); run 4 did not engage; CPU splits matched anyway.
**Relative numbers only** (see PROFILING_GUIDE.md); ui/modeld invalid here.

## CPU split (control stack, % of one core)

| process | run3 | run4 | top GIL self-time frames |
|---|---|---|---|
| locationd | 18.6 | 18.6 | rednose `predict_and_observe` 42% |
| card | 18.0 | 17.3 | `messaging.send` 31%, `convert_carControlSP` 4.3% |
| selfdrived | 11.9 | 11.2 | `messaging.send` 33%, `update_events` ~11.5% |
| controlsd | 9.2 | 8.7 | `messaging.send` 30% |
| plannerd | 3.7 | 3.5 | SubMaster/log_from_bytes bookkeeping |

## Verdicts (each verified → optimized → adversarially reviewed)

### 1. `cereal.messaging.send` ≈ 11–12% of a core — REAL; fix belongs upstream

Not serialization: `to_bytes()` is 0.7–1.2 µs. The cost is one
`tkill(SIGUSR2)` **per registered reader per publish**, GIL held
(`PubSocket.send` has no `with nogil:`). carState has 13 registered readers;
12 are spuriously signaled 100×/s. Measured ~25 µs + 10–12 µs/reader/publish.
Publish audit: nothing published needlessly often; BP's Ford-only topics
contribute zero here (~2–3 ms/s on a Ford device).

**Action:** wake-fanout suppression prototype (3.9× on 13-reader topics, all
tests + adversarial lost-wake test pass) archived in
`proposals/msgq-wake-suppression.md` — **for an upstream commaai/msgq PR
only**; a fork-carried missed-wake bug would freeze safety-critical processes.
**Fork guardrail adopted instead:** every new subscriber to a 100 Hz topic
costs every publisher ~10 µs/publish plus a 100 Hz spurious wake — keep BP
additions off hot topics.

### 2. locationd 18.6% — REAL CPU, but a **sim-fidelity artifact** (fixed)

`predict_and_observe` is the Cython/C++ EKF (GIL not released, so py-spy
attributes all of it to one Python line). The container number is ~4×
overstated because the sim bridge sent IMU at **500 Hz** (5 duplicated
messages per 100 Hz tick) vs the device's ~104 Hz. At device rates the EKF
costs ~3% of a core — the known cost of the math, not a hotspot.

**Adopted:** `tools/sim/lib/simulated_sensors.py` now sends one accel + one
gyro per tick. End-to-end (real locationd process, msgq, /proc CPU):
28.5–32% → **9.5–9.9%** of a core; filter health improved (velocity converges
faster — the 5× burst was overweighting IMU vs camera odometry). Sim-only
file; zero device risk.

### 3. `selfdrived.update_events` ~0.7% of a core of pure waste (fixed)

Upstream-normal code, not fork-added: a 58-process `managerState` scan every
10 ms against a message that changes at 2 Hz (~98% redundant), plus a
`radarErrors.to_dict()` allocating a 4-bool dict per frame.

**Adopted:** `recv_frame`-keyed memoization of the `not_running` set — exact
under all call patterns including the pre-initialization early-return window
(the reviewer refuted the simpler `updated`-flag gating for precisely that
window) — and explicit bool reads for the radar elif chain (16/16 combos
re-verified equivalent against `cereal/car.capnp` `RadarData.Error`).
Benchmark: 75 → 4.6 µs/frame on those lines. Both changes are candidates for
upstreaming to comma.

### 4. card conversions ~1% of a core (fixed)

`convert_carControlSP` cost 98 µs/frame, 76% of it `struct.to_dict()`
recursively dictifying the whole message (deprecated fields and the dead
`params` list included) just to throw half away. **BluePilot's own additions
measured innocent:** `FordSettingsBP` default_factory 0.76 µs, the snapshot
attach 0.06 µs.

**Adopted:** direct capnp reader-attribute conversion: 87 → 36 µs/frame
(2.4×), pinned by a schema-coverage test (fails if `CarControlSP` gains a
field the converter doesn't handle) plus a 200-example fuzz equivalence test
against the old converter (`selfdrive/car/tests/test_helpers.py`). Documented
benign delta: unset sub-structs now yield capnp schema defaults
(`radarTrackId` −1 vs accidental 0; enums plain `str`) — reachable only
before the first `carControlSP` arrives, no consumer distinguishes.

### Explicit not-worth-its

- Decimating sunnypilot shadow topics (~1% core): changes SubMaster
  `alive`/`freq_ok` semantics for consumers — rejected.
- `with nogil:` on `PubSocket.send`: fixes profiler attribution, not CPU, for
  single-threaded publishers — deferred to upstream discussion.
- rednose/locationd code changes: cost is inherent EKF math once sim rates
  are fixed.

## Net effect on the container profile (expected)

locationd ~−9% core (sim fidelity), selfdrived ~−0.7%, card ~−0.5%; msgq
proposal (if upstreamed) ~−8–10% more across the stack. On device, the
percentages shrink but the mechanisms are identical; the sim fix makes every
future container profile more device-faithful.
