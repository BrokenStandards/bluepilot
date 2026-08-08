"""BluePilot: per-frame model-decel gate for experimental longitudinal.

The e2e model's acceleration participates in the planner's min() ONLY while the model
expresses deceleration intent. Otherwise the MPC drives alone, so a model plateauing at
~0.0 accel below the set/limit target can never hold the car under speed — while the
model's early, smooth braking for lights/hazards is preserved (it engages the moment
intent appears, with zero mode-machine latency).

Shadow-replayed against a real drive: the accel clause engages on the exact first
decel-intent frame of both recorded stops (0.0 s lag; 3.0 s earlier than DEC's slowDown
detector at an empty red light, 13.4 s earlier than DEC's radar-lead rule behind a
stopped car), reproduces the recorded pure-experimental braking exactly, and restores
MPC acceleration in cruise and at launch.

Engaging is the safe direction (it only ADDS the model to the min; the MPC is never
removed), so engage is immediate. Release is the risky direction (hands authority to a
lead-blind cruise plan), so it requires EVERY intent signal clear, sustained — brief
positive excursions of the 0.3 s-smoothed model accel mid-stop cannot release because
the model's 10 s plan-end speed stays collapsed through a genuine stop.

The gate's PARTICIPATION is continuous, not binary (see weight) — but the weight is a
shaped ENVELOPE of the binary state machine above, never an independent signal. The
binary form alternated between "MPC accelerating toward the target" and "full e2e
deceleration" whenever the model hovered around a setpoint — log-measured at
10.9 flips/min on a real drive, with the release snap moving the plan accel up to
+0.73 m/s^2 within half a second. The weight smooths exactly those edges and nothing
else: it rises over RISE_TIME when the binary engage condition fires (shouldStop takes
it all immediately), HOLDS at its peak for as long as the machine is latched — inside
the hysteresis band a model still expressing decel intent keeps every bit of the
authority it had, exactly as the binary latch always guaranteed — and decays across the
release window only on frames where EVERY intent signal reads clear, reaching the MPC by
the time the machine deactivates. Below the engage setpoint the weight is zero, full
stop: an adversarial review of a band-position variant (weight tracking how far into the
hysteresis band the signals sat) showed why anything else is unsound — a model resting
just above the engage threshold indefinitely would phantom-brake the car from cruise to
a standstill, and a model easing mid-braking into the band would blend in enough of the
lead-blind accelerating cruise plan to command net acceleration against live decel
intent. The engage threshold exists to make sub-threshold intent a no-op; the envelope
keeps it that way.
"""

from openpilot.common.realtime import DT_MDL

ENGAGE_ACCEL = -0.2   # m/s^2 default: model demands real deceleration (outside the ~0.0 plateau noise band)
# The release threshold tracks the engage threshold with a fixed hysteresis band so the
# user-adjustable engage value (0.00 .. -5.00, BPModelDecelGateAccel) keeps a working band.
RELEASE_HYSTERESIS = 0.15  # m/s^2 (default engage -0.2 -> release -0.05)

# m/s below v_ego for the model's 10 s plan-end speed. The end speed collapses toward 0 well
# before instantaneous accel dips when approaching a red light — the EARLY engage signal.
# Log-measured: plan-end sags 0.6-2.3 m/s below v_ego in ordinary cruise, so the DEFAULT
# engage margin sits above that band; user-adjustable (BPModelDecelGateEndV, 0.00 .. -10.00,
# stored as a negative delta) since sensitivity is a per-model preference. The release
# margin re-arms at half the engage margin.
ENGAGE_END_V_MARGIN = 2.0

RELEASE_TIME = 0.5        # s of sustained all-clear before handing back to the MPC
RELEASE_TIME_LOW_SPEED = 1.5  # s below LOW_SPEED (creep zone: keep the model's launch authority)
LOW_SPEED = 3.0           # m/s

# How fast the continuous weight may traverse the whole band, per direction. The fall spans
# the release window (see weight), making the hand-back a ramp instead of a snap. The rise is
# fast but not instant: an unlimited rise re-creates the very jerk this exists to remove when
# the model's accel churns within the band (a one-frame 0.33 weight step against a 0.7 m/s^2
# e2e-to-MPC spread is a ~0.23 m/s^2 accel step, ~4.7 m/s^3). 0.15 s full-band keeps in-band
# churn under ~5 m/s^3 while delaying nothing that matters: deep engages from cruise are
# jerk-floored by the planner's 2.5 m/s^3 engage slew anyway (which takes ~0.3 s to traverse
# the same spread, i.e. the slew, not this rise, is what paces a hard engage), the min() with
# the MPC never waits on the blend for lead braking, and shouldStop bypasses the limit.
RISE_TIME = 0.15          # s for weight 0 -> 1
WEIGHT_EPS = 1e-6         # snap-to-zero: decay must terminate exactly, not asymptotically


class ModelDecelGate:
  def __init__(self, dt: float = DT_MDL, engage_accel: float = ENGAGE_ACCEL,
               end_v_margin: float = ENGAGE_END_V_MARGIN):
    self._dt = dt
    self._release_counter = 0
    self.active = False
    self.weight = 0.0  # continuous participation, 0 = MPC alone .. 1 = full min(e2e, mpc)
    self.set_engage_accel(engage_accel)
    self.set_end_v_margin(-end_v_margin)

  def set_engage_accel(self, engage_accel: float) -> None:
    # different models brake with different strength; the threshold is user-adjustable so
    # gentle model slowdowns (curves, crests, brake lights) can engage on soft-braking models
    self.engage_accel = min(0.0, max(-5.0, engage_accel))
    self.release_accel = self.engage_accel + RELEASE_HYSTERESIS

  def set_end_v_margin(self, end_v_delta: float) -> None:
    # param convention: negative delta (plan end speed this far BELOW v_ego engages);
    # stored internally as a positive margin. Release re-arms at half the engage margin.
    self.end_v_margin = min(10.0, max(0.0, -end_v_delta))
    self.release_end_v_margin = self.end_v_margin / 2.0

  def reset(self) -> None:
    """Forget everything. Called when the gate stops being consulted (leaving e2e mode,
    feature toggled off): letting weight/active freeze published minutes-stale state and,
    worse, resurrected stale participation into the plan on re-entry."""
    self.active = False
    self.weight = 0.0
    self._release_counter = 0

  def update(self, e2e_accel: float, e2e_should_stop: bool, model_end_v: float, v_ego: float) -> bool:
    decel_intent = (e2e_should_stop or
                    e2e_accel < self.engage_accel or
                    model_end_v < v_ego - self.end_v_margin)
    all_clear = (not e2e_should_stop and
                 e2e_accel > self.release_accel and
                 model_end_v > v_ego - self.release_end_v_margin)
    release_time = RELEASE_TIME_LOW_SPEED if v_ego < LOW_SPEED else RELEASE_TIME

    # ---- binary state machine (unchanged semantics; still the published state) ----
    if decel_intent:
      self.active = True
      self._release_counter = 0
    elif self.active:
      if all_clear:
        self._release_counter += 1
        if self._release_counter >= int(release_time / self._dt):
          self.active = False
          self._release_counter = 0
      else:
        # in the hysteresis band between engage and release thresholds: hold the gate
        self._release_counter = 0

    # ---- continuous participation: a shaped envelope of the machine above ----
    # Rise over RISE_TIME while the engage condition holds (shouldStop immediately - the
    # stop-hold latch takes everything); HOLD while latched in the hysteresis band, so a
    # model easing mid-braking keeps full authority exactly as the binary latch promised;
    # decay across the release window only on all-clear frames, so the hand-back to the
    # cruise plan is a ramp that completes as the machine deactivates. Note the decay
    # integrates CUMULATIVE clear time where the binary counter demands it consecutive:
    # band frames between clear runs hold the weight (never refill it), so flickering
    # intent releases no faster than sustained-clear intent, just not slower.
    if e2e_should_stop:
      self.weight = 1.0
    elif decel_intent:
      self.weight = min(1.0, self.weight + self._dt / RISE_TIME)
    elif self.active and not all_clear:
      pass  # latched in-band: hold
    else:
      self.weight = max(0.0, self.weight - self._dt / release_time)
      if self.weight < WEIGHT_EPS:
        self.weight = 0.0

    return self.active
