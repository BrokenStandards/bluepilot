# BluePilot Longitudinal Research: Driving Model Output → Alpha Long / Experimental Braking → Ford Coast Behavior

**Scope of the question:** where the driving model's longitudinal output is defined and documented, how BluePilot turns that into ACC commands under alpha long and experimental mode, what BluePilot's Ford-specific longitudinal tuning / coast band actually is, and how that compares with Ford's stock coasting behavior — with the goal of finding why the car uses friction braking to hold speed where it should be coasting.

All line numbers are against the tree at commit `1bbda0e` (branch `bp-7.0`).

---

## 0. TL;DR — where the unnecessary braking comes from

Ranked by how much each one contributes to "brakes instead of coasts":

1. **The brake-actuation threshold is shallower than Ford's own coast band.** BluePilot asserts `AccBrkDecel_B_Rq` (friction brakes) at **−0.14 m/s²**, but the Ford gas channel is designed to carry deceleration requests all the way down to **−0.5 m/s²** (`MIN_GAS`) as an engine-braking / lift-throttle request. The entire band **[−0.5, −0.14] m/s² is Ford's designed coast region, and BluePilot converts it into friction braking.** Effective coast band on BluePilot is only ~[−0.14, 0) — and only ~[−0.06, 0) once brakes are already engaged, because of hysteresis.
   `opendbc_repo/opendbc/sunnypilot/car/ford/longitudinal_ext.py:62-65`, `opendbc_repo/opendbc/car/ford/values.py:36-39`
2. **The planner has no coast dead-band on its output.** `openpilot`'s coast logic (`get_coast_accel`) only ever clamps the **upper** (throttle) limit. The lower limit is always `ACCEL_MIN = −3.5`. Nothing in the planner says "this decel is small enough, just coast" — that decision is delegated entirely to the car port, and the car port's threshold is the −0.14 above.
   `selfdrive/controls/lib/longitudinal_planner.py:113,128-131,172-175`
3. **In `pacing` state, BluePilot rewrites a "no gas" request into a "hold 0.0 m/s²" request.** `min_follow_gas = 0.0` clips `op_gas` upward, so an inactive-gas request (−5.0) becomes a *0.0 m/s² propulsion request* — the PCM will hold speed rather than let the car coast down.
   `longitudinal_ext.py:181-185,200`
4. **The decel rate limiter is extremely slow (0.1 m/s³) and resets to 0 whenever the lead is lost.** With no lead, `bp_accel` is forced to exactly `0` and stored in `bp_accel_last`; when a lead reappears, decel can only ramp at 0.002 m/s² per 20 ms scan. Reaching −1.0 m/s² takes ~10 s. During that ramp the car holds speed.
   `longitudinal_ext.py:59,193-197,203-206,236-237`
5. **Experimental mode can only ever add braking, never remove it.** `output_a_target = min(e2e, mpc)`. The e2e model's `desiredAcceleration` is never allowed to *raise* the target above the MPC's, so experimental mode is a strictly-more-decel path.
   `selfdrive/controls/lib/longitudinal_planner.py:163-170`
6. **`AccPrpl_A_Pred` is hard-wired to inactive.** BluePilot parameterized the signal but always passes `INACTIVE_GAS` (−5.0), so the one channel Ford uses to telegraph an upcoming *smooth* decel is unused.
   `longitudinal_ext.py:255`, `opendbc_repo/opendbc/sunnypilot/car/ford/fordcan_ext.py` (`create_acc_msg`)

---

## 1. Driving model longitudinal output — the documentation source

There is no prose spec in `docs/`; the authoritative definition of the model's longitudinal output is the **cereal schema plus the modeld constants/slices**. Those four files are the "documentation source":

### 1.1 Wire schema — `cereal/log.capnp`

```capnp
struct Action {                        # cereal/log.capnp:1075-1079
  desiredCurvature      @0 :Float32;
  desiredAcceleration   @1 :Float32;   # m/s^2, the e2e longitudinal output
  shouldStop            @2 :Bool;
}
```

- `ModelDataV2.action` — `cereal/log.capnp:999`
- `DrivingModelData.action` — `cereal/log.capnp:931`
- `MetaData.disengagePredictions` — `cereal/log.capnp:1037`
- `DisengagePredictions.gasPressProbs` — `cereal/log.capnp:1064` (this is the signal the coast band keys off, see §3.1)
- `LongitudinalPlan.allowThrottle` — `cereal/log.capnp:1158`

### 1.2 Output tensor layout — `selfdrive/modeld/constants.py`

```python
class Plan:                                    # constants.py:67-72
  POSITION            = slice(0, 3)
  VELOCITY            = slice(3, 6)            # <- used for desired_accel
  ACCELERATION        = slice(6, 9)            # <- used for desired_accel
  T_FROM_CURRENT_EULER= slice(9, 12)
  ORIENTATION_RATE    = slice(12, 15)

class Meta:                                    # constants.py:74-85
  GAS_DISENGAGE  = slice(1, 31, 6)             # t = 2,4,6,8,10 s
  BRAKE_DISENGAGE= slice(2, 31, 6)
  GAS_PRESS      = slice(31, 55, 4)            # t = 0,2,4,6,8,10 s  <- coast band input
  BRAKE_PRESS    = slice(32, 55, 4)
```

Also relevant: `ACTION_WIDTH = 2` (`constants.py:41`), `IDX_N = 33` and `T_IDXS` (`constants.py:8-10`).

### 1.3 Where `action` is produced

**Stock modeld** (`selfdrive/modeld/modeld.py:40-65`) — prefers the model's dedicated `action` head when the loaded model exposes one, else derives it from the `plan` head:

```python
if 'action' not in model_output:
    desired_accel, should_stop = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
                                                    plan[:,Plan.ACCELERATION][:,0],
                                                    ModelConstants.T_IDXS,
                                                    action_t=long_action_t)
else:
    desired_accel = model_output['action'][0,1]
    should_stop   = (v_ego < 0.3 and desired_accel < 0.1)
desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)
```

`LONG_SMOOTH_SECONDS = 0.3` (`modeld.py:35`), `long_delay = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS` (`modeld.py:222`).

**sunnypilot modeld_v2** — the runner BluePilot actually uses for custom model bundles — is different in two ways worth knowing (`sunnypilot/modeld_v2/modeld.py:241-256`):

- It **never uses the `action` head.** It always calls `get_accel_from_plan(...)` off the `plan` head.
- `LONG_SMOOTH_SECONDS` defaults to **0.0**, not 0.3 (`modeld.py:92`), so `long_delay = CP.longitudinalActuatorDelay + 0.0 = 0.15 s` for Ford, and the action horizon is `0.15 + DT_MDL = 0.20 s` (`modeld.py:323,421`).

Which runner is live is selected at `system/manager/process_config.py:125` (`selfdrive.modeld.modeld`, stock runner) vs `:180` (`sunnypilot/modeld_v2`, tinygrad runner).

### 1.4 Where `action` and the meta probabilities get published

`selfdrive/modeld/fill_model_msg.py:97` (`modelV2.action = action`) and `:132-141` (`disengage_predictions.gasPressProbs = net_output_data['meta'][0, Meta.GAS_PRESS]`). sunnypilot equivalent: `sunnypilot/modeld_v2/fill_model_msg.py:85,119`.

### 1.5 The accel-from-plan math

`selfdrive/controls/lib/drive_helpers.py:43-57` — this is the function that defines what "desired acceleration" means numerically:

```python
v_target = np.interp(action_t, t_idxs, speeds)
a_target = 2 * (v_target - v_now) / action_t - a_now
should_stop = (v_now < vEgoStopping and a_target < 0.1)
```

It is a constant-jerk inversion over `action_t`, so a small model velocity error at short `action_t` is amplified into a large `a_target`. With Ford's `action_t = 0.20 s`, a 0.1 m/s velocity shortfall alone produces `a_target ≈ −1.0 m/s²` — which is well past the −0.14 brake threshold. **This is a direct mechanism for spurious brake pulses while holding a constant set speed.**

---

## 2. How braking is applied under alpha long and experimental mode

### 2.1 Alpha long is what gates openpilot longitudinal on Ford at all

Upstream only offers alpha long on radar-less Fords (`opendbc_repo/opendbc/car/ford/interface.py:56-60`). BluePilot overrides that:

```python
# opendbc_repo/opendbc/sunnypilot/car/ford/interfaces_ext.py:44-56
ret.alphaLongitudinalAvailable = True          # every Ford platform
ret.openpilotLongitudinalControl = bool(alpha_long)   # the toggle is authoritative
if ret.openpilotLongitudinalControl:
    ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.LONG_CONTROL.value
else:
    ret.safetyConfigs[-1].safetyParam &= ~FordSafetyFlags.LONG_CONTROL.value
```

So on BluePilot, alpha long **off** = Ford's own ACC (plus ICBM button emulation); alpha long **on** = the full openpilot planner → `ACCDATA` path described below. The toggle lives at `selfdrive/ui/layouts/settings/developer.py:81-85` (`AlphaLongitudinalEnabled`).

Also applied in the same function (`interfaces_ext.py:36-37`):

```python
ret.steerActuatorDelay = 0.22          # upstream: 0.2
ret.longitudinalTuning.kpV = [0.]      # BluePilot zeroes the proportional gain
```

Combined with the stock Ford `kiV = [0.5]` (`interface.py:43-44`), the long PID is **I-only plus feedforward** (`selfdrive/controls/lib/longcontrol.py:86-89`). Ford leaves `longitudinalActuatorDelay` at the 0.15 s default (`opendbc_repo/opendbc/car/interfaces.py:256`) and `vEgoStopping/stopAccel/stoppingDecelRate` at 0.5 / −2.0 / 0.8 (`interfaces.py:247-249`).

### 2.2 The planner chain (alpha long on)

`selfdrive/controls/lib/longitudinal_planner.py`:

| Step | Line | Effect |
|---|---|---|
| accel clip built | `113` | `[ACCEL_MIN(−3.5), get_max_accel(v_ego)]` — **lower bound is never raised** |
| turn limiting | `36-47,115` | only lowers the upper bound |
| coast band | `124-131` | only lowers the upper bound (§3.1) |
| SCC / SLA targets | `134` | may lower `v_cruise` (sunnypilot Smart Cruise Control, Speed Limit Assist) |
| MPC solve | `139-145` | ACC path |
| MPC → a_target | `157-159` | `get_accel_from_plan(..., action_t = 0.15 + 0.05 = 0.20 s)` |
| e2e blend | `160-170` | **experimental mode** |
| slew + final clip | `172-175` | clip limits themselves slew-limited ±0.05 per cycle |

### 2.3 Experimental mode specifically

```python
# longitudinal_planner.py:163-170
if self.is_e2e(sm):
    output_a_target = min(output_a_target_e2e, output_a_target_mpc)
    self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
    if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
else:
    output_a_target = output_a_target_mpc
```

`is_e2e()` is sunnypilot's, at `sunnypilot/selfdrive/controls/lib/longitudinal_planner.py`:

```python
def is_e2e(self, sm):
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
        return experimental_mode
    return experimental_mode and self.dec.mode() == "blended"
```

Two consequences:

- Experimental mode is a **min()**, i.e. monotonically-more-braking. The model can add decel but can never veto the MPC's decel, so it cannot restore coasting.
- With **Dynamic Experimental Control** (`DynamicExperimentalControl` param) active, e2e is gated to `blended` mode. DEC's `blended` request is driven by lead presence, MPC FCW, "slowness" (`v_ego <= 1.025 × v_cruise`) and trajectory-endpoint shortfall vs `SLOW_DOWN_DIST` (`sunnypilot/selfdrive/controls/lib/dec/dec.py:206-300`, constants at `dec/constants.py`). Note the **slowness** term: being slightly under set speed itself pushes toward blended mode, which then pulls in the e2e `min()`.

### 2.4 ACC path costs (the "maintain speed" pressure)

`selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`:

```python
X_EGO_OBSTACLE_COST = 3.       # :35
J_EGO_COST          = 5.       # :39
A_CHANGE_COST       = 200.     # :40
DANGER_ZONE_COST    = 100.     # :41
COMFORT_BRAKE       = 2.5      # :56
STOP_DISTANCE       = 6.0      # :57
CRUISE_MIN_ACCEL    = -1.2     # :58
CRUISE_MAX_ACCEL    =  1.6     # :59
```

The cruise term is a *fake obstacle* placed ahead of the car (`long_mpc.py:330-336`):

```python
v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
v_upper = v_ego + (T_IDXS * CRUISE_MAX_ACCEL * 1.05)
v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)
cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)
```

Overspeeding relative to `v_cruise` moves the cruise obstacle inside the safe distance, and `X_EGO_OBSTACLE_COST` then commands decel down to `CRUISE_MIN_ACCEL = −1.2 m/s²`. On a downgrade that is exactly the "braking to hold the set speed" case — and −1.2 is 8.6× deeper than the −0.14 brake threshold.

---

## 3. The coast band: upstream openpilot vs BluePilot's Ford tuning

### 3.1 openpilot's coast band (throttle-side only)

`selfdrive/controls/lib/longitudinal_planner.py:23-34,124-131`:

```python
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3   # fitted from data, xx/projects/allow_throttle/compute_coast_accel.py

...
throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]   # p(gas pressed at t=2 s)
self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

if not self.allow_throttle:
    clipped_accel_coast = max(accel_coast, accel_clip[0])
    clipped_accel_coast_interp = np.interp(v_ego, [2.5, 5.0], [accel_clip[1], clipped_accel_coast])
    accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)
```

Semantics, precisely:

- `get_coast_accel` is the measured **free-rolling deceleration of the vehicle** as a function of road pitch. Flat road → **−0.30 m/s²**. A 2° downgrade → **−0.10 m/s²**. So "coast" in openpilot's vocabulary means *this* number.
- `gasPressProbs[1]` is the model's probability that a human would be on the gas ~2 s from now. Below 0.4, openpilot refuses to apply throttle and **caps the upper accel limit at coast accel**.
- **This never lowers the accel floor.** `accel_clip[0]` stays `ACCEL_MIN = −3.5`. There is no upstream mechanism that says "don't brake for small decel demands". The reference maneuver test is `selfdrive/test/longitudinal_maneuvers/test_longitudinal.py:153-165`.
- `longitudinalPlan.allowBrake` is hard-coded `True` (`longitudinal_planner.py:197`) and `allowThrottle` is published (`:198`) but consumed **only by the UI** (`selfdrive/ui/onroad/model_renderer.py:291`, `selfdrive/ui/mici/onroad/model_renderer.py:343`, `bluepilot/ui/widgets/debug/long_debug_panel.py:92`). **No car controller reads it.**

### 3.2 BluePilot's Ford-specific longitudinal tuning

`opendbc_repo/opendbc/sunnypilot/car/ford/longitudinal_ext.py` — the file's own header documents its design:

```
  - Speed deadband: BP long engages above 50 mph, disengages below 45 mph
  - Lead classification: gaining (closing in), pacing (matching), trailing (falling behind)
  - Gas limits per state: zero gas when gaining within 1.5s, capped gas when pacing
  - Rate-limited accel changes to avoid stomping the brakes
  - TTC-based emergency bypass for imminent collision scenarios
  - Mutual exclusion: brake_actuate forces gas to INACTIVE_GAS
```

Constants (`longitudinal_ext.py:57-66`):

| Constant | Value | Meaning |
|---|---|---|
| `MAX_URBAN_SPEED_MPH` | 45.0 | BP long off below this; on above 50 (`:108-114`) |
| `following_accel_ROC` | 0.002 | max accel change per 20 ms scan = **0.1 m/s³** |
| `brake_actuate_target` | **−0.14** | assert `AccBrkDecel_B_Rq` below this |
| `brake_actuate_release` | **−0.06** | release above this |
| `precharge_actuate_target` | −0.12 | assert `AccBrkPrchg_B_Rq` below this |
| `precharge_actuate_release` | −0.06 | release above this |

These values predate the extraction into `longitudinal_ext.py` — they are identical in the inline version at commit `e1aa136` (`git show e1aa136:opendbc_repo/opendbc/car/ford/carcontroller.py`, lines 139-144).

State machine (`longitudinal_ext.py:151-206`):

```python
if lead:
    gaining  = v_rel < -0.1
    trailing = v_rel >  0.1
    pacing   = otherwise

if gaining and lead_time_sec < 1.5:  max_follow_gas = min_follow_gas = 0.0
if pacing:                           max_follow_gas = 0.2 + accel_due_to_pitch; min_follow_gas = 0.0
if lead is None:                     max_follow_accel = min_follow_accel = 0

bp_gas   = clip(op_gas,   min_follow_gas,   max_follow_gas)
bp_accel = clip(op_accel, min_follow_accel, max_follow_accel)

if ttc_sec > 8.0 and lead_time_sec > 0.5:
    bp_accel = clip(bp_accel, self.bp_accel_last - 0.002, 999)   # decel slew only
```

Gate for applying any of it (`:219-234`):

```python
apply_bp_long = (not disable_BP_long_UI and self.bpSpeedAllow and
                 not gasPressed and not brakePressed and
                 (lead is None or v_lead_mph > 40.0))
```

**When `apply_bp_long` is False — which is all driving below ~50 mph, and any lead slower than 40 mph — the car falls through to `op_accel` / `op_gas` with `op_brake_actuate` governed by the same −0.14 / −0.06 hysteresis** (`:100-106,230-234`). So the shallow brake threshold governs essentially all driving, whether or not the follow logic is active.

Two toggles feed this (`selfdrive/ui/bp/layouts/settings/bluepilot.py:544-560`):

- `disable_BP_long_UI` — "Bypass BP Longitudinal Control / Use stock longitudinal logic instead of BluePilot TTC/coasting tuning."
- `disable_downhill_comp_UI` — "Disable pitch-based brake/gas compensation when going downhill." Applied at `opendbc_repo/opendbc/car/ford/carcontroller.py:251-254`, clamping `accel_due_to_pitch` to ≥ 0. Shipped in 6.0.1 as *"Enhanced ACC coasting behavior with option to disable downhill assist for vehicles that handle it natively"* (`BP_CHANGES.json:49`).

### 3.3 Where the two interact badly

`accel_due_to_pitch = sin(pitch) × 9.81` (`carcontroller.py:247-249`) is **added** to `op_accel` before the brake decision (`longitudinal_ext.py:101`). A 2° downgrade contributes **−0.34 m/s²** — on its own more than double the −0.14 brake threshold. That is why the downhill-disable toggle exists, and it is a blunt fix: it clamps the term to 0 rather than widening the coast band.

Meanwhile the planner's own coast model says a 2° downgrade should free-roll at −0.10 m/s², i.e. **the car would naturally hold or gently lose speed with no actuation at all.** The two models of "downhill" are inconsistent: the planner treats pitch as *reduced available deceleration* (raises the coast accel toward 0), the car port treats pitch as *added deceleration demand* (pushes toward the brakes).

---

## 4. Ford stock coasting behavior

### 4.1 The three ACCDATA channels and what they mean

`opendbc_repo/opendbc/car/ford/fordcan.py:120-145` (stock builder) and `opendbc_repo/opendbc/sunnypilot/car/ford/fordcan_ext.py` (`create_acc_msg`, BluePilot builder):

| Signal | Range | Role |
|---|---|---|
| `AccPrpl_A_Rq` | [−5, 5.23] m/s² | **Propulsion request.** Negative = lift-throttle / engine-brake / regen request. |
| `AccPrpl_A_Pred` | [−5, 5.23] m/s² | Predicted accel. Stock hardcodes −5.0; BluePilot parameterized it but still always sends −5.0. |
| `AccBrkTot_A_Rq` | [−20, 11.94] m/s² | Analog brake magnitude. **Does nothing without the actuation bits.** |
| `AccBrkPrchg_B_Rq` | bool | Pre-charge the brake system. |
| `AccBrkDecel_B_Rq` | bool | **Actually apply the friction brakes.** |

The semantics are documented in-tree in the pre-refactor carcontroller comment (`git show e1aa136:opendbc_repo/opendbc/car/ford/carcontroller.py`, longitudinal block):

> `accel` is the analog signal to the brakes in m/s2
> `gas` is the analog signal to the accelerator in m/s2
> **`brake_actuate` is the signal to actually press the brakes (negative accel without `brake_actuate` results in engine braking)**
> For hybrids/EV the ford PCM determines when to use brake pedal versus regen, there is no way for openpilot to affect this.

And on the inactive-gas quirk:

> this is a quirk in the ford PCM, if you are not using gas, it has to be set to −5.0 m/s2 or you will get a cruise fault

### 4.2 The stock coast band, from the numbers

`opendbc_repo/opendbc/car/ford/values.py:36-39`:

```python
ACCEL_MAX    =  2.0
ACCEL_MIN    = -3.5
MIN_GAS      = -0.5    # floor of the *propulsion* channel
INACTIVE_GAS = -5.0    # "no gas request" sentinel
```

Panda enforces the same envelope (`opendbc_repo/opendbc/safety/modes/ford.h:497-509`):

```c
.max_accel = 5641,       //  1.9999 m/s^2   (AccBrkTot_A_Rq)
.min_accel = 4231,       // -3.4991 m/s^2
.inactive_accel = 5128,  // -0.0008 m/s^2
.max_gas = 700,          //  2.0 m/s^2      (AccPrpl_A_Rq / _Pred)
.min_gas = 450,          // -0.5 m/s^2
.inactive_gas = 0,       // -5.0 m/s^2
```

**The band from 0 down to −0.5 m/s² is the propulsion channel's negative region.** Ford's ACC uses it for exactly what we want: request a mild deceleration, and let the PCM decide *how* — throttle lift, downshift, engine braking, or (on hybrids/EVs) regen. Friction brakes are only involved when the discrete actuation bits are set.

Openpilot's stock handling honors this (`opendbc_repo/opendbc/car/ford/carcontroller.py:243-244`):

```python
if not CC.longActive or op_gas < CarControllerParams.MIN_GAS:
    op_gas = CarControllerParams.INACTIVE_GAS
```

i.e. requests in `[−0.5, 0)` are sent through as negative propulsion (coast); only below −0.5 does the gas channel give up. Ford also declines to accelerate near set speed (`opendbc_repo/opendbc/car/ford/interface.py:23-29`, upper limit tapers to 0.2 m/s² within 0.4 m/s of cruise speed).

### 4.3 Side-by-side

| Decel demand (pitch-compensated) | Ford stock ACC intent | BluePilot today |
|---|---|---|
| 0 to −0.06 | coast, gas ≈ 0 | coast (brakes released) |
| −0.06 to −0.12 | coast on negative `AccPrpl_A_Rq` | coast, **unless** brakes already latched (hysteresis holds them to −0.06) |
| −0.12 to −0.14 | coast / engine brake | **precharge asserted** |
| **−0.14 to −0.50** | **engine brake / regen — no friction brakes** | **`AccBrkDecel_B_Rq` = 1, gas forced to `INACTIVE_GAS`** ← the gap |
| below −0.50 | friction brakes | friction brakes |

The mutual-exclusion rule (`longitudinal_ext.py:247-249`) is what makes it exclusive rather than additive:

```python
if brake_actuate:
    gas = CarControllerParams.INACTIVE_GAS
```

Once `brake_actuate` latches at −0.14, the negative-propulsion coast channel is *abandoned* for the whole event — the car cannot engine-brake and friction-brake in a graded way; it jumps straight to friction. On a hybrid/EV this also means the PCM's regen arbitration is handed a brake request rather than a coast request.

---

## 5. Specific defects worth fixing

### 5.1 Brake threshold undercuts the gas floor (primary)

`brake_actuate_target = −0.14` sits **inside** the propulsion channel's usable range (down to −0.5). Widening it toward −0.45…−0.5 would make the two channels contiguous instead of overlapping, and would let all mild decel ride the coast path. Suggested shape:

```
brake_actuate_target   ≈ MIN_GAS + margin   (e.g. -0.45)
brake_actuate_release  ≈ -0.20
precharge_actuate_target stays shallow (-0.12) — precharge is cheap and hides latency
```

Precharge is the right tool for the shallow band: it removes brake lash *without* commanding deceleration, so a deeper `brake_actuate_target` costs nothing in response time.
`opendbc_repo/opendbc/sunnypilot/car/ford/longitudinal_ext.py:62-65`

### 5.2 `pacing` turns "no gas" into "hold 0.0 m/s²"

```python
# longitudinal_ext.py:181-185
if pacing:
    max_follow_gas = 0.2 + accel_due_to_pitch
    min_follow_gas = 0.0
...
bp_gas = clip(op_gas, min_follow_gas, max_follow_gas)   # :200
```

`op_gas` is `INACTIVE_GAS = −5.0` whenever the planner wants less than −0.5 m/s². Clipping that to a floor of `0.0` produces a **0.0 m/s² propulsion request** — an active "hold this speed" command to the PCM, precisely the opposite of coasting. The floor should be `INACTIVE_GAS` (pass through) or `MIN_GAS`, not `0.0`.

Secondary: when `disable_downhill_comp_UI` is off, `accel_due_to_pitch < −0.2` on any downgrade steeper than ~1.2° makes `max_follow_gas < min_follow_gas`; per numpy's documented `clip` behavior the upper bound wins, so the band inverts silently.

### 5.3 Decel slew rate is 0.1 m/s³ and resets to zero on lead loss

```python
self.following_accel_ROC = 0.002                                    # :59  → 0.1 m/s^3 at 50 Hz
if lead is None:  max_follow_accel = min_follow_accel = 0           # :193-197
if ttc_sec > 8.0 and lead_time_sec > 0.5:
    bp_accel = clip(bp_accel, self.bp_accel_last - 0.002, 999)      # :205-206
self.bp_accel_last = bp_accel                                       # :237
```

With no lead, `bp_accel_last` is pinned to 0. On lead reacquisition the slew starts from 0, so a −1.0 m/s² demand takes ~10 s to reach. The intent ("dampen initial brake hit") is right but the rate is roughly 25× slower than a comfortable jerk limit (2.5 m/s³). Recommend: raise the ROC substantially, and seed `bp_accel_last` from `op_accel` rather than 0 when no lead is present.

### 5.4 The planner never expresses "coast is enough"

`allowThrottle` is computed, published, and read only by the UI. A car-port-visible coast intent — e.g. publishing `accel_coast` on `longitudinalPlan`, or a `LongCtrlState.coasting` — would let Ford choose the propulsion channel deliberately instead of the car port inferring it from a magnitude threshold.
`selfdrive/controls/lib/longitudinal_planner.py:33-34,126,197-198`

### 5.5 Short action horizon amplifies model noise into brake pulses

`action_t = longitudinalActuatorDelay + DT_MDL = 0.20 s` for Ford, and sunnypilot's modeld_v2 sets `LONG_SMOOTH_SECONDS = 0.0` (vs stock 0.3), removing the low-pass on `desiredAcceleration`. Given `a_target = 2(v_target − v_now)/action_t − a_now`, small velocity noise becomes large accel commands. Worth checking on a real route whether raising `LONG_SMOOTH_SECONDS` for the active bundle reduces brake-bit chatter.
`sunnypilot/modeld_v2/modeld.py:92,241-246,323`

### 5.6 Pitch is modeled two contradictory ways

Planner: pitch raises the coast ceiling (`get_coast_accel`, `longitudinal_planner.py:33-34`).
Car port: pitch adds decel demand toward the brakes (`carcontroller.py:247-249` + `longitudinal_ext.py:101`).
The downhill toggle papers over this by zeroing the term. Reconciling the two — e.g. comparing `op_accel` against `get_coast_accel(pitch)` rather than against a fixed −0.14 — would make the brake threshold pitch-aware in the correct direction.

### 5.7 `AccPrpl_A_Pred` is dead

`accel_pred_send = CarControllerParams.INACTIVE_GAS` unconditionally (`longitudinal_ext.py:255`), even though `fordcan_ext.create_acc_msg` accepts it as a parameter. Prior code marked this deliberately: *"TODO return to this signal later, it might help with highway control, but sending values ford doesn't like causes ACC to cancel"* (`git show e1aa136:...carcontroller.py`). Panda permits it in `[−0.5, 2.0]` plus the −5.0 inactive sentinel (`ford.h:506-509,517-519`), so it is testable within existing safety limits.

---

## 6. Instrumentation already available

`bluepilot/ui/widgets/debug/long_debug_panel.py` plots desired vs actual accel and gas vs brake at 20 Hz, and shows Should Stop / Throttle Allowed / Brake Allowed (`:88-113`). It reads `carControl.actuators.gas`/`.brake` and `longitudinalPlan` — enough to confirm the −0.14 crossings on a route, but it does **not** currently show `brake_actuate` / `precharge_actuate` or `bp_long_used`. Adding those three to `controllerStateBP` would make the coast-vs-brake band directly observable, since `LongitudinalResult` already carries `bp_long_used` (`longitudinal_ext.py:26-35,260-269`).

---

## Appendix — file index

**Driving model output**
- `cereal/log.capnp:931,999,1037,1064,1075-1079,1158`
- `selfdrive/modeld/constants.py:8-10,41,67-85`
- `selfdrive/modeld/modeld.py:34-36,40-65,222,318-320`
- `selfdrive/modeld/fill_model_msg.py:97,132-141`
- `sunnypilot/modeld_v2/modeld.py:88-94,241-256,323,421`
- `sunnypilot/modeld_v2/fill_model_msg.py:85,119`
- `system/manager/process_config.py:87-93,125,180`

**Planner / coast band / experimental mode**
- `selfdrive/controls/lib/longitudinal_planner.py:20-34,69-87,92-95,113-131,157-175,197-198`
- `selfdrive/controls/lib/drive_helpers.py:6,43-57`
- `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py:35-44,56-59,73-87,316-340`
- `selfdrive/controls/lib/longcontrol.py:63-92`
- `sunnypilot/selfdrive/controls/lib/longitudinal_planner.py` (`is_e2e`, `update_targets`)
- `sunnypilot/selfdrive/controls/lib/dec/dec.py:132-300`, `dec/constants.py`
- `selfdrive/test/longitudinal_maneuvers/test_longitudinal.py:153-165`

**BluePilot Ford longitudinal tuning**
- `opendbc_repo/opendbc/sunnypilot/car/ford/longitudinal_ext.py` (whole file; constants `:57-66`)
- `opendbc_repo/opendbc/sunnypilot/car/ford/interfaces_ext.py:36-56`
- `opendbc_repo/opendbc/sunnypilot/car/ford/fordcan_ext.py` (`create_acc_msg`)
- `opendbc_repo/opendbc/car/ford/carcontroller.py:62-66,229-273`
- `selfdrive/ui/bp/layouts/settings/bluepilot.py:544-560`
- `bluepilot/ui/widgets/debug/long_debug_panel.py:58-113`
- `BP_CHANGES.json:49`, `README.md:191-192,205`

**Ford stock behavior**
- `opendbc_repo/opendbc/car/ford/values.py:23-42`
- `opendbc_repo/opendbc/car/ford/fordcan.py:120-145`
- `opendbc_repo/opendbc/car/ford/interface.py:23-29,32-60`
- `opendbc_repo/opendbc/car/interfaces.py:28-29,247-256`
- `opendbc_repo/opendbc/safety/modes/ford.h:14-15,495-535`
- `git show e1aa136:opendbc_repo/opendbc/car/ford/carcontroller.py` — pre-refactor inline longitudinal block with the original signal-semantics comments
