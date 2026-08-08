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

The gate's PARTICIPATION is continuous, not binary (see weight). The binary form
alternated between "MPC accelerating toward the target" and "full e2e deceleration"
whenever the model hovered around a setpoint — log-measured at 10.9 flips/min on a real
drive, with the release snap moving the plan accel up to +0.73 m/s^2 within half a
second and 15 ten-second windows containing four or more flips. weight ramps across the
SAME hysteresis bands the binary thresholds already define (so full-engage and
full-release happen at exactly the points they always did): as model intent deepens from
the release threshold toward the engage threshold, the planner's e2e branch blends in
proportionally, smoothly lowering the maximum acceleration until, at the setpoint, the
model has it all — the same shape the curve-speed profile uses to approach a curve.
Weight RISES instantly (engage stays the safe direction) and FALLS rate-limited over the
release window, so handing authority back to the cruise plan is a ramp, not a snap.
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

  def _target_weight(self, e2e_accel: float, e2e_should_stop: bool, model_end_v: float, v_ego: float) -> float:
    """How much of the e2e branch the intent signals ask for right now, 0..1.

    Each analog signal ramps linearly across its existing hysteresis band — from the
    release threshold (0, exactly where the binary gate finishes releasing) to the engage
    threshold (1, exactly where the binary gate engages) — and the signals combine by max,
    so any one deep signal keeps full authority regardless of the others (mid-stop, brief
    excursions of the smoothed model accel cannot lift the cap while the plan-end speed
    stays collapsed). shouldStop is binary and demands everything.
    """
    if e2e_should_stop:
      return 1.0

    w = 0.0
    band = self.release_accel - self.engage_accel  # = RELEASE_HYSTERESIS, always > 0
    w = max(w, min(1.0, (self.release_accel - e2e_accel) / band))

    if self.end_v_margin > 0.0:
      sag = v_ego - model_end_v
      v_band = self.end_v_margin - self.release_end_v_margin  # = margin/2, > 0 when enabled
      w = max(w, min(1.0, (sag - self.release_end_v_margin) / v_band))

    return max(0.0, w)

  def update(self, e2e_accel: float, e2e_should_stop: bool, model_end_v: float, v_ego: float) -> bool:
    # Continuous participation: both directions are rate-limited ramps (rise fast, fall over
    # the release window — see RISE_TIME/RELEASE_TIME), except shouldStop, which is the
    # stop-hold latch and takes everything immediately. The fall spans the same RELEASE_TIME
    # the binary release already waits, so by the time the state machine below deactivates,
    # the blend has already reached the MPC and deactivation changes nothing.
    target = self._target_weight(e2e_accel, e2e_should_stop, model_end_v, v_ego)
    if e2e_should_stop:
      self.weight = 1.0
    elif target >= self.weight:
      self.weight = min(target, self.weight + self._dt / RISE_TIME)
    else:
      release_time = RELEASE_TIME_LOW_SPEED if v_ego < LOW_SPEED else RELEASE_TIME
      self.weight = max(target, self.weight - self._dt / release_time)
    if self.weight < WEIGHT_EPS:
      self.weight = 0.0

    decel_intent = (e2e_should_stop or
                    e2e_accel < self.engage_accel or
                    model_end_v < v_ego - self.end_v_margin)

    if decel_intent:
      self.active = True
      self._release_counter = 0
      return self.active

    if self.active:
      all_clear = (not e2e_should_stop and
                   e2e_accel > self.release_accel and
                   model_end_v > v_ego - self.release_end_v_margin)
      if all_clear:
        self._release_counter += 1
        release_time = RELEASE_TIME_LOW_SPEED if v_ego < LOW_SPEED else RELEASE_TIME
        if self._release_counter >= int(release_time / self._dt):
          self.active = False
          self._release_counter = 0
      else:
        # in the hysteresis band between engage and release thresholds: hold the gate
        self._release_counter = 0

    return self.active
