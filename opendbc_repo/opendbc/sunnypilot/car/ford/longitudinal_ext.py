"""
BluePilot Ford longitudinal follow control extension.

Implements smoother highway following by classifying lead vehicle behavior
(gaining, pacing, trailing) and applying gas/accel limits per state. Also
adds split brake/precharge hysteresis for smoother deceleration.

Key features:
  - Speed deadband: BP long engages above 50 mph, disengages below 45 mph
  - Lead classification: gaining (closing in), pacing (matching), trailing (falling behind)
  - Gas limits per state: zero gas when gaining within 1.5s, capped gas when pacing
  - Rate-limited accel changes to avoid stomping the brakes
  - TTC-based emergency bypass for imminent collision scenarios
  - Mutual exclusion: brake_actuate forces gas to INACTIVE_GAS (legacy mode)

Coasting modes (FordPrefCoastingMode param, selectable in Longitudinal Tuning):
  - legacy (0): the original BluePilot behavior, byte-for-byte. Friction brakes
    (AccBrkDecel_B_Rq) engage once the request drops below -0.14 m/s^2.
  - extended (1): restores Ford's factory coast band. The ACCDATA propulsion
    channel (AccPrpl_A_Rq) is designed to carry deceleration requests down to
    MIN_GAS (-0.5 m/s^2) as a lift-throttle / engine-brake / regen request, so
    the friction-brake bit is only asserted below -0.45 m/s^2 -- everything
    shallower coasts on the propulsion channel, like stock Ford ACC. Extended
    mode also stops raising "no gas" (INACTIVE_GAS) requests into active
    hold-speed commands, drops the lead-loss accel pinning, and keeps negative
    propulsion flowing alongside the brake bits (both stock Ford ACC and
    upstream openpilot do this; the panda safety checks the bits and the accel
    value independently, see opendbc/safety/modes/ford.h).
"""

from collections import namedtuple

import numpy as np
from numpy import clip

from opendbc.car.ford.values import CarControllerParams


COASTING_MODE_LEGACY = 0
COASTING_MODE_EXTENDED = 1

# Lead classification for the longLeadState diagnostic (controllerStateBP)
LEAD_STATE_NONE = 0
LEAD_STATE_GAINING = 1
LEAD_STATE_PACING = 2
LEAD_STATE_TRAILING = 3


# Result namedtuple returned by LongitudinalExt.update()
LongitudinalResult = namedtuple('LongitudinalResult', [
  'accel',
  'gas',
  'brake_actuate',
  'precharge_actuate',
  'accel_pred_send',
  'stopping',
  'target_speed',
  'bp_long_used',
])


class LongitudinalExt:
  """
  BluePilot longitudinal follow control extension for Ford vehicles.

  Mixed into CarController via multiple inheritance. The stock carcontroller
  computes op_accel/op_gas using upstream logic, then calls
  LongitudinalExt.update() to apply BP follow control on top.

  The SubMaster (for radarState) is owned by LateralCurvExt and shared via self.sm
  since both classes are mixed into the same CarController instance.
  """

  def __init__(self, CP, CP_SP):
    # BP longitudinal state
    self._bp_long_active_last = False
    self.bp_gas_last = 0.0
    self.bp_accel_last = 0.0
    self.bpSpeedAllow = False

    # Thresholds
    self.MAX_URBAN_SPEED_MPH = 45.0
    self.following_accel_ROC = 0.002  # max accel change per scan in following mode

    # Brake hysteresis thresholds (legacy coasting mode)
    self.brake_actuate_target = -0.14   # engage brakes below this accel
    self.brake_actuate_release = -0.06  # release brakes above this accel
    self.precharge_actuate_target = -0.12
    self.precharge_actuate_release = -0.06
    self.op_brake_actuate_last = False

    # Extended coasting mode: friction brakes only below the propulsion channel's
    # usable floor (MIN_GAS = -0.5 m/s^2), so requests in (-0.45, 0) coast on
    # AccPrpl_A_Rq the way stock Ford ACC does. Wide release hysteresis avoids
    # bit chatter; precharge leads brake engagement to hide brake-lash latency
    # without holding pre-pressure through the whole coast band.
    self.ext_brake_actuate_target = -0.45
    self.ext_brake_actuate_release = -0.20
    self.ext_precharge_actuate_target = -0.35
    self.ext_precharge_actuate_release = -0.15
    # 0.02 m/s^2 per 20ms scan = 1.0 m/s^3: still smooths decel onset, but reaches
    # -1 m/s^2 in 1s instead of legacy's 10s (0.002/scan), so braking is not held
    # off long enough to overshoot the set speed first.
    self.ext_following_accel_ROC = 0.02
    self.ext_brake_actuate_last = False
    self.ext_precharge_actuate_last = False

    # Toggles (updated from Params each frame)
    self.disable_BP_long_UI = False
    self.disable_downhill_comp_UI = True
    self.coasting_mode = COASTING_MODE_LEGACY  # FordPrefCoastingMode: 0=legacy, 1=extended

    # Coasting diagnostics published via controllerStateBP (bp_card_publisher.py)
    self.longCoastingMode = COASTING_MODE_LEGACY
    self.longBpLongUsed = False
    self.longBrakeActuate = False
    self.longPrechargeActuate = False
    self.longLeadState = LEAD_STATE_NONE
    self.longOpAccel = 0.0
    self.longBpAccel = 0.0
    self.longOpGas = 0.0
    self.longBpGas = 0.0
    self.longAccelPitch = 0.0
    self.longTtcSec = 0.0
    self.longLeadTimeSec = 0.0
    self.longCoasting = False

  def update_long_params(self, params):
    """Read longitudinal-related Params from the UI. Called each frame."""
    self.disable_BP_long_UI = params.get_bool("disable_BP_long_UI")
    self.disable_downhill_comp_UI = params.get_bool("disable_downhill_comp_UI")
    self.coasting_mode = int(params.get("FordPrefCoastingMode", return_default=True) or COASTING_MODE_LEGACY)

  def update(self, CC, CS, op_accel, op_gas, accel_due_to_pitch, v_ego_mph, stopping, target_speed):
    """
    Apply BluePilot longitudinal follow control on top of stock op_accel/op_gas.

    Called at 50Hz from CarController.update() inside the ACC_CONTROL_STEP block,
    after stock creep compensation and rate limiting have been applied.

    Args:
      CC: CarControl with longActive
      CS: CarState with vEgo, gasPressed, brakePressed
      op_accel: Stock openpilot accel after creep comp + rate limit (m/s^2)
      op_gas: Stock openpilot gas value (m/s^2)
      accel_due_to_pitch: Pitch compensation value (m/s^2, may be clamped by downhill toggle)
      v_ego_mph: Current speed in mph
      stopping: True if in stopping state
      target_speed: Target cruise speed (km/h)

    Returns:
      LongitudinalResult namedtuple with final accel, gas, brake, precharge values.
    """
    extended = self.coasting_mode == COASTING_MODE_EXTENDED

    # Coasting-mode hysteresis thresholds. Legacy values keep the original
    # behavior exactly; extended widens the coast band (see class docstring).
    if extended:
      brake_target = self.ext_brake_actuate_target
      brake_release = self.ext_brake_actuate_release
      precharge_target = self.ext_precharge_actuate_target
      precharge_release = self.ext_precharge_actuate_release
    else:
      brake_target = self.brake_actuate_target
      brake_release = self.brake_actuate_release
      precharge_target = self.precharge_actuate_target
      precharge_release = self.precharge_actuate_release

    # Op brake actuate hysteresis
    accel_pitch_compensated = op_accel + accel_due_to_pitch
    op_brake_actuate = self.op_brake_actuate_last
    if accel_pitch_compensated > brake_release or not CC.longActive:
      op_brake_actuate = False
    elif accel_pitch_compensated < brake_target:
      op_brake_actuate = True

    # Speed deadband: engage above 50 mph, disallow below 45 mph
    bpSpeedTooSlow = v_ego_mph < self.MAX_URBAN_SPEED_MPH
    bpSpeedHighEnough = v_ego_mph > self.MAX_URBAN_SPEED_MPH + 5
    if bpSpeedHighEnough:
      self.bpSpeedAllow = True
    if bpSpeedTooSlow:
      self.bpSpeedAllow = False

    lead_state = LEAD_STATE_NONE
    ttc_sec = 120.0
    lead_time_sec = 999.0

    # BP longitudinal follow control
    if not self.disable_BP_long_UI:
      # Read lead vehicle data from radarState (SubMaster is on self via mixin)
      v_ego = max(CS.out.vEgo, 0.5)
      lead = None
      v_rel = 0.0
      v_lead = 0.0

      if self.sm.valid.get('radarState', False):
        rs = self.sm['radarState']
        lead = getattr(rs, 'leadOne', None)
        if lead is not None and getattr(lead, 'status', 0) != 1:
          lead = None
        if lead:
          d_rel = float(getattr(lead, 'dRel', 0))
          v_rel = float(getattr(lead, 'vRel', 0))
          v_lead = float(getattr(lead, 'vLead', 0))
          if d_rel > 0:
            lead_time_sec = d_rel / v_ego

      lead_time_sec = float(np.clip(lead_time_sec, 0.0, 999.0))
      v_lead_mph = v_lead * 2.23694

      # Time to collision
      ttc_sec = 120.0
      if lead:
        d_rel = float(getattr(lead, 'dRel', 0))
        v_rel = float(getattr(lead, 'vRel', 0))
        if d_rel > 0 and v_rel < 0:
          ttc_sec = d_rel / (-v_rel)
        else:
          ttc_sec = 60.0
      ttc_sec = float(np.clip(ttc_sec, 0.2, 120.0))

      # Classify lead state: gaining, pacing, or trailing
      gaining = False
      pacing = False
      trailing = False
      bp_brake_actuate = False
      bp_precharge_actuate = False

      if lead:
        if v_rel < -0.1:
          gaining = True
          lead_state = LEAD_STATE_GAINING
        elif v_rel > 0.1:
          trailing = True
          lead_state = LEAD_STATE_TRAILING
        else:
          pacing = True
          lead_state = LEAD_STATE_PACING

      if extended:
        # EXTENDED: gas limits are caps only. Legacy's floor of 0.0 turned a
        # "no gas" request (INACTIVE_GAS, -5.0) into an active 0.0 m/s^2
        # hold-speed command to the PCM -- the opposite of coasting. Here a
        # no-gas request stays no-gas, and negative propulsion passes through.
        gas_cap = None
        if gaining and lead_time_sec < 1.5:
          gas_cap = 0.0  # within 1.5s and gaining -- no gas
        elif pacing:
          # The pitch term can push the cap below the propulsion channel's
          # floor on steep downgrades; don't invert past MIN_GAS.
          gas_cap = max(0.2 + accel_due_to_pitch, CarControllerParams.MIN_GAS)

        if gas_cap is not None and op_gas != CarControllerParams.INACTIVE_GAS:
          bp_gas = min(op_gas, gas_cap)
        else:
          bp_gas = op_gas

        # No lead-loss pinning: legacy forced accel to exactly 0 with no lead,
        # which both held speed where the planner wanted decel and left
        # bp_accel_last seeded at 0 for the next lead's ROC ramp.
        bp_accel = op_accel

        # Rate limit downward accel changes while following (dampen initial
        # brake hit). Cruise decel is already shaped by the planner, so with
        # no lead the request passes through and the ROC seed stays fresh.
        # Skip rate limit if imminent collision risk.
        if lead is not None and ttc_sec > 8.0 and lead_time_sec > 0.5:
          bp_accel = clip(bp_accel, self.bp_accel_last - self.ext_following_accel_ROC, 999)
      else:
        # LEGACY: original gas/accel limits per state, byte-for-byte
        max_follow_gas = op_gas
        min_follow_gas = op_gas
        max_follow_accel = op_accel
        min_follow_accel = op_accel

        if gaining:
          if lead_time_sec < 1.5:
            max_follow_gas = 0.0  # within 1.5s and gaining — no gas
            min_follow_gas = 0.0
          else:
            max_follow_gas = op_gas
            min_follow_gas = op_gas
          max_follow_accel = op_accel
          min_follow_accel = op_accel

        if pacing:
          max_follow_gas = 0.2 + accel_due_to_pitch  # cap gas when pacing
          min_follow_gas = 0.0
          max_follow_accel = op_accel
          min_follow_accel = op_accel

        if trailing:
          max_follow_gas = op_gas
          min_follow_gas = op_gas
          max_follow_accel = op_accel
          min_follow_accel = op_accel

        if lead is None:
          max_follow_gas = op_gas
          min_follow_gas = op_gas
          max_follow_accel = 0
          min_follow_accel = 0

        # Apply BP gas and accel targets
        bp_gas = clip(op_gas, min_follow_gas, max_follow_gas)
        bp_accel = clip(op_accel, min_follow_accel, max_follow_accel)

        # Rate limit downward accel changes (dampen initial brake hit)
        # Skip rate limit if imminent collision risk
        if ttc_sec > 8.0 and lead_time_sec > 0.5:
          bp_accel = clip(bp_accel, self.bp_accel_last - self.following_accel_ROC, 999)

        # BP brake/precharge hysteresis (stateless: between target and release
        # the bits stay at their per-frame init of False)
        if bp_accel < brake_target:
          bp_brake_actuate = True
        if bp_accel > brake_release:
          bp_brake_actuate = False
        if bp_accel < precharge_target:
          bp_precharge_actuate = True
        if bp_accel > precharge_release:
          bp_precharge_actuate = False

      # Decide whether to apply BP long
      gasPressed = CS.out.gasPressed
      brakePressed = CS.out.brakePressed
      apply_bp_long = (not self.disable_BP_long_UI and self.bpSpeedAllow and
                       not gasPressed and not brakePressed and
                       (lead is None or v_lead_mph > 40.0))

      if apply_bp_long and CC.longActive:
        accel = bp_accel
        gas = bp_gas
        brake_actuate = bp_brake_actuate
        precharge_actuate = bp_precharge_actuate
      else:
        accel = op_accel
        gas = op_gas
        brake_actuate = op_brake_actuate
        precharge_actuate = op_brake_actuate

      self.bp_gas_last = bp_gas
      self.bp_accel_last = bp_accel
      bp_long_used = apply_bp_long
    else:
      # BP long disabled — pass through stock values
      accel = op_accel
      gas = op_gas
      brake_actuate = op_brake_actuate
      precharge_actuate = op_brake_actuate
      bp_long_used = False

    if extended:
      # EXTENDED: one persistent hysteresis pair, decided on the pitch-compensated
      # accel that is actually being sent (legacy's BP-path bits were stateless and
      # ignored pitch, and its op-path precharge shadowed the brake bit). The panda
      # checks the bits and the accel value independently (ford.h), so this is a
      # tuning change only.
      accel_comp = accel + accel_due_to_pitch
      ext_brake_actuate = self.ext_brake_actuate_last
      if accel_comp > brake_release or not CC.longActive:
        ext_brake_actuate = False
      elif accel_comp < brake_target:
        ext_brake_actuate = True
      ext_precharge_actuate = self.ext_precharge_actuate_last
      if accel_comp > precharge_release or not CC.longActive:
        ext_precharge_actuate = False
      elif accel_comp < precharge_target:
        ext_precharge_actuate = True
      # Coming to (and holding) a stop needs friction brakes regardless of how
      # shallow the request still is -- engine braking cannot hold the car.
      if stopping and CC.longActive:
        ext_brake_actuate = True
        ext_precharge_actuate = True
      self.ext_brake_actuate_last = ext_brake_actuate
      self.ext_precharge_actuate_last = ext_precharge_actuate
      brake_actuate = ext_brake_actuate
      precharge_actuate = ext_precharge_actuate
    else:
      # Keep the extended state pair tracking reality so a mid-drive mode switch
      # starts from a sane value.
      self.ext_brake_actuate_last = brake_actuate
      self.ext_precharge_actuate_last = precharge_actuate

    # Mutual exclusion: no positive gas while the friction brakes are requested.
    if brake_actuate:
      if extended:
        # Keep negative propulsion (engine braking / regen) flowing alongside the
        # brake request, as stock Ford ACC and upstream openpilot both do. Only a
        # positive (accelerate) request is contradictory.
        if gas != CarControllerParams.INACTIVE_GAS:
          gas = min(gas, 0.0)
      else:
        gas = CarControllerParams.INACTIVE_GAS

    # Clip to ford.h ACCDATA safety limits
    accel = float(clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    if gas != CarControllerParams.INACTIVE_GAS:
      gas = float(clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))
    accel_pred_send = CarControllerParams.INACTIVE_GAS

    self._bp_long_active_last = bp_long_used
    self.op_brake_actuate_last = op_brake_actuate

    # Coasting diagnostics for controllerStateBP (published by bp_card_publisher.py)
    self.longCoastingMode = int(self.coasting_mode)
    self.longBpLongUsed = bool(bp_long_used)
    self.longBrakeActuate = bool(brake_actuate)
    self.longPrechargeActuate = bool(precharge_actuate)
    self.longLeadState = int(lead_state)
    self.longOpAccel = float(op_accel)
    self.longBpAccel = float(accel)
    self.longOpGas = float(op_gas)
    self.longBpGas = float(gas)
    self.longAccelPitch = float(accel_due_to_pitch)
    self.longTtcSec = float(ttc_sec)
    self.longLeadTimeSec = float(lead_time_sec)
    # True while decel is being carried by the propulsion channel alone (the coast band)
    self.longCoasting = bool(CC.longActive and accel < 0.0 and not brake_actuate)

    return LongitudinalResult(
      accel=accel,
      gas=gas,
      brake_actuate=brake_actuate,
      precharge_actuate=precharge_actuate,
      accel_pred_send=accel_pred_send,
      stopping=stopping,
      target_speed=target_speed,
      bp_long_used=bp_long_used,
    )
