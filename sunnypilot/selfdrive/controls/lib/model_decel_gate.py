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
"""

from openpilot.common.realtime import DT_MDL

ENGAGE_ACCEL = -0.2   # m/s^2: model demands real deceleration (outside the ~0.0 plateau noise band)
RELEASE_ACCEL = -0.05  # m/s^2: hysteresis upper bound for release

# m/s below v_ego for the model's 10 s plan-end speed. The end speed collapses toward 0 well
# before instantaneous accel dips when approaching a red light — the EARLY engage signal.
# Log-measured: plan-end sags 0.6-2.3 m/s below v_ego in ordinary cruise, so the engage
# margin sits above that band; release re-arms at half of it.
ENGAGE_END_V_MARGIN = 2.0
RELEASE_END_V_MARGIN = 1.0

RELEASE_TIME = 0.5        # s of sustained all-clear before handing back to the MPC
RELEASE_TIME_LOW_SPEED = 1.5  # s below LOW_SPEED (creep zone: keep the model's launch authority)
LOW_SPEED = 3.0           # m/s


class ModelDecelGate:
  def __init__(self, dt: float = DT_MDL):
    self._dt = dt
    self._release_counter = 0
    self.active = False

  def update(self, e2e_accel: float, e2e_should_stop: bool, model_end_v: float, v_ego: float) -> bool:
    decel_intent = (e2e_should_stop or
                    e2e_accel < ENGAGE_ACCEL or
                    model_end_v < v_ego - ENGAGE_END_V_MARGIN)

    if decel_intent:
      self.active = True
      self._release_counter = 0
      return self.active

    if self.active:
      all_clear = (not e2e_should_stop and
                   e2e_accel > RELEASE_ACCEL and
                   model_end_v > v_ego - RELEASE_END_V_MARGIN)
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
