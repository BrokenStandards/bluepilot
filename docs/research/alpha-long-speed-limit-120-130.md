# Alpha Long & the 120/130 km/h Set-Speed Requirement — Research Report

**Branch:** `claude/sunnypilot-alpha-long-speed-4ulm44` (based on `claude/bluepilot-speed-limit-overlay-h0djc7`)
**Date:** 2026-08-01
**Question:** Why does Speed Limit Assist demand the cluster be set to exactly 120 or 130 km/h
(70/80 mph) under alpha longitudinal, is that hardware-required on Ford, and why does alpha long
drive slower than the map speed limit when ICBM tracks it well?

## Verdict

1. **The 120/130 requirement is pure sunnypilot software protocol.** There is no Ford
   hardware, CAN, or panda-safety constraint tied to those numbers. BluePilot inherited the
   code byte-identical from upstream sunnypilot PR #833 ("Longitudinal: Speed Limit Assist",
   merged 2025-09-30, commit `dc0fd4c`). No commit message, PR text, or code comment justifies
   the specific values.
2. **What *is* real:** on Ford, alpha long runs with `pcmCruise = True`, so the driver's
   cluster set speed is a hard ceiling on openpilot's target (planner `min()`), and the Ford
   PCM additionally limits acceleration to 0.2 m/s² starting ~2 m/s below the set speed.
   The set speed therefore must sit *comfortably above* the desired travel speed — but **any**
   such value is hardware-equivalent. The exact-match against 120/130 is a driver-consent
   handshake, not physics.
3. **Much of the "alpha long picks arbitrarily low speeds" behavior traces to two latent bugs**
   in the speed limit resolver (monotonic-vs-epoch clock mismatch; indefinite stale-limit
   latching), not to deliberate speed selection. See "Bugs" below.

## 1. Where the requirement lives

- Constant: `PCM_LONG_REQUIRED_MAX_SET_SPEED = {metric: (120, 130) km/h, imperial: (70, 80) mph}`
  — `sunnypilot/selfdrive/controls/lib/speed_limit/__init__.py:11`.
- Gate: `pcm_op_long = CP.openpilotLongitudinalControl and CP.pcmCruise`
  (`speed_limit_assist.py:89`). On Ford, `pcmCruise` defaults `True`
  (`opendbc_repo/opendbc/car/interfaces.py:243`) and is never cleared; BluePilot's alpha-long
  toggle sets `openpilotLongitudinalControl = True` on **all** Ford platforms
  (`opendbc_repo/opendbc/sunnypilot/car/ford/interfaces_ext.py:51`). So every BluePilot Ford
  with alpha long lands on this path.
- Required value selection (`speed_limit_assist.py:185-190`): **120** when a known
  (offset-adjusted) limit is below `CONFIRM_SPEED_THRESHOLD` (80 km/h / 50 mph); **130**
  when the limit is ≥ threshold **or unknown**.
- Confirmation is **exact integer equality** on rounded display units
  (`speed_limit_assist.py:115-116`). Setting 140 (or anything else) never activates SLA;
  `preActive` times out to `inactive` after 15 s (`:288-290`).
- While `active`/`adapting`, *any* cluster change drops SLA to `inactive`
  (`:246-248`, `:256-258`), and under pcm-long `inactive` is a dead end (`:292-294` is a bare
  `pass`) until longitudinal is disengaged and re-engaged.
- Driver-facing surface: alert "Speed Limit Assist: set to {120|130} km/h to engage"
  (`sunnypilot/selfdrive/selfdrived/events.py`, `PCM_LONG_REQUIRED_MAX_SET_SPEED` callback).

### Why two values (the 120↔130 dance)

Because confirmation is `==`, a single constant would auto-confirm every new limit with zero
driver input. The pair turns the stalk into a consent channel: a newly detected sub-80 km/h
limit flips the requirement 130→120 and the driver must press SET− within 15 s to consent to
the slowdown; fresh engagement with a high/no limit demands exactly 130. Limits ≥ 80 that
change while active auto-apply without a handshake. The exact-match also doubles as override
detection (any other cluster movement = driver takeover → `inactive`).

### The official rationale (docs & upstream history)

sunnypilot's docs (docs.sunnypilot.ai, speed-limit page) explain: on PCM-cruise cars
*"the stock cruise module controls the instrument cluster display, sunnypilot cannot show the
actual speed limit on the cluster"*, so the cluster is parked at a *"fixed high value"* that
*"signals that SLA is managing the target speed"* while *"actual speed limiting happens at the
software level."* The constant began life on the PR #833 branch as a single
`REQUIRED_INITIAL_CRUISE_SPEED = 80 mph` ("TODO-SP: customizable with params") and was later
split into the (120, 130)/(70, 80) pair. The forum theory ("so the cluster doesn't look insane
on city streets") is not the stated reason — the cluster display simply cannot be commanded on
PCM cars, and the fixed value is the activation/consent mechanism.

## 2. Ford hardware audit — nothing requires 120/130

- `grep` audit of `opendbc_repo/opendbc/car/ford/`, `opendbc_repo/opendbc/sunnypilot/car/ford/`,
  and `opendbc_repo/opendbc/safety/modes/ford.h`: no speed constant of 120/130/70/80 exists.
  Safety validates only accel/gas bounds on ACCDATA (`ford.h` `FORD_LONG_LIMITS`); it never
  checks a set speed. `EngBrakeData` is RX-only (cruise state + brake), never the set-speed field.
- The only set-speed-shaped TX signal, `ACCDATA.AccVeh_V_Trg`, is **hardcoded to
  `V_CRUISE_MAX` = 145 km/h** regardless of the driver's cluster value
  (`ford/carcontroller.py:257` → `fordcan_ext.py:149`).
- The genuine coupling: with `pcmCruise=True`, the stock-button set speed
  (`EngBrakeData.Veh_V_DsplyCcSet`, `ford/carstate.py:99`) becomes openpilot's `v_cruise`,
  which (a) caps the planner target via `min()`
  (`sunnypilot/selfdrive/controls/lib/longitudinal_planner.py:65-73`), and (b) feeds
  `get_pid_accel_limits` (`ford/interface.py:24-29`), which ramps allowed accel from
  `ACCEL_MAX` down to 0.2 m/s² between `cruise_speed − 2 m/s` and `cruise_speed − 0.4 m/s` —
  the comment records real PCM behavior ("PCM doesn't allow acceleration near cruise_speed").

**Conclusion:** the set speed must exceed the desired travel speed by roughly ≥ 2 m/s
(~7 km/h) for full accel authority; beyond that, all values are equivalent. The exact 120/130
is a removable software convention.

## 3. How alpha long picks its speed; where map data enters

Chain: `mapd` → `/dev/shm` params → `liveMapDataSP` → `SpeedLimitResolver`
(car-camera vs map per `SpeedLimitPolicy`, plus user offset) → `SpeedLimitAssist` → planner
`min()` over `{cruise set speed, SCC-vision, SCC-map, SLA}`
(`sunnypilot/selfdrive/controls/lib/longitudinal_planner.py:65-73`) → MPC `v_cruise`.

- Posted map limits affect control **only** through SLA and **only** when
  `SpeedLimitMode == assist` (default *information* mode displays only).
- The SCC map controller carries curvature-based curve speeds (`MapTargetVelocities`),
  not posted limits.
- Nothing under alpha long ever *raises* the cluster set speed: ICBM only runs when
  `pcmCruiseSpeed` is False, which requires `openpilotLongitudinalControl == False`
  (`sunnypilot/selfdrive/car/interfaces.py:62-64`;
  `intelligent_cruise_button_management/controller.py:164-165`). The
  "BluePilot: For PCM cars using ICBM" branch inside SLA's pcm state machine
  (`speed_limit_assist.py:275-285`) is therefore unreachable in the way its comment implies —
  functionally a no-op under alpha long.

### Why ICBM tracks map speed well

Under stock ACC, ICBM keeps openpilot's own internal set speed, snaps it to limit+offset,
walks the *physical* cluster there via injected SET+/SET− CAN presses, and Ford's factory ACC
does the driving with factory tuning. None of SLA's latching, the 15 s handshake window, or
BluePilot's conservative alpha-long accel tuning (`longitudinalTuning.kpV = [0.]`,
`interfaces_ext.py:37`) is in that loop.

## 4. Bugs found (verified, to be fixed on this branch)

1. **Dead map-data staleness guard — clock-domain mismatch.**
   `speed_limit_resolver.py:126`:
   `gps_fix_age = time.monotonic() - gps_data.unixTimestampMillis * 1e-3`.
   `unixTimestampMillis` is Unix-epoch wall time; `time.monotonic()` counts from boot. The
   difference is a huge negative number, so `gps_fix_age > LIMIT_MAX_MAP_DATA_AGE (10 s)`
   never fires and stale map data is **never** invalidated.
2. **Dead upcoming-limit anticipation — same mismatch.**
   `speed_limit_resolver.py:139` computes `distance_since_fix` from the same subtraction,
   producing an enormous positive `distance_to_speed_limit_ahead`, so the
   `distance_to_speed_limit_ahead <= adapt_distance` branch (`:150`) never triggers. The code
   even carries `# FIXME-SP: this is not working as expected` (`:145`). Result: no gradual
   adaptation toward a lower limit ahead.
3. **Stale limit latched forever + unconfirmed cap.**
   `update_speed_limit_states` (`speed_limit_resolver.py:77-82`) latches the last non-zero
   limit indefinitely; SLA outputs that latched value into the planner `min()` whenever it is
   merely *enabled* — including the unconfirmed 15 s `preActive` window after every engagement
   (`speed_limit_assist.py:128-136` with `ENABLED_STATES`). An old lower limit from a road you
   left can cap the car long after.

## 5. Planned work on this branch

1. Tests demonstrating bugs 1–3, then fixes, verified by those tests (and the existing
   `speed_limit` suites).
2. **Alpha long with any speed over the limit:** replace the pcm-long 120/130 exact-match with
   the press-again-to-confirm handshake already used by the non-PCM path; after confirmation,
   a one-shot, non-nagging prompt suggesting 130 for best experience; if the cluster sits
   below the desired speed, prompt to raise it to desired + offset.
3. **ICBM under alpha long:** let ICBM walk the physical cluster to the desired target plus a
   new user-configurable margin ("alpha long ICBM added threshold", default ~5 km/h) so the
   Ford PCM's near-set-speed accel gate never binds; the margin is configurable because the
   gate width is vehicle-dependent.
