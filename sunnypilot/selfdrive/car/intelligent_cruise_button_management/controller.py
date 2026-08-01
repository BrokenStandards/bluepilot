"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car, custom
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4
V_CRUISE_UNSET = 255.  # m/s sentinel published by inactive SLA/SCC controllers
# BluePilot: how long an alpha-long target must hold steady before ICBM walks the cluster to it.
# Damps resolver source flaps (e.g. stale OSM vs TSR) that would otherwise stream SET+/SET-
# presses at the physical PCM continuously.
ALPHA_LONG_TARGET_DWELL = 2.0  # secs


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER

    # BluePilot: Track initial cruise speed when first enabled
    self.initial_cruise_speed_kph = 0
    self.cruise_enabled_prev = False

    # BluePilot: alpha-long ICBM — walk the physical cluster to (SLA target + margin) while
    # openpilot longitudinal drives, so the cluster ceiling (and the PCM's near-set-speed accel
    # gate) never limits acceleration to the desired speed. pcmCruiseSpeed stays True in this
    # mode; enable + margin are fed by selfdrived's params thread.
    self.alpha_long_mode_allowed = bool(CP.openpilotLongitudinalControl and CP_SP.intelligentCruiseButtonManagementAvailable)
    self.alpha_long_enabled = False
    self.alpha_long_offset = 5  # display units (km/h or mph)
    # frame-consistent snapshot of alpha_long_active: the params thread flips alpha_long_enabled
    # asynchronously, and gate/calc/state-machine must agree within one run() frame
    self._alpha_frame_active = False
    self._alpha_pending_target = 0
    self._alpha_pending_frames = 0
    self._alpha_adopted = False

  @property
  def alpha_long_active(self) -> bool:
    return self.alpha_long_mode_allowed and self.alpha_long_enabled

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def _update_alpha_long_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

    # The reference is SLA's limit target, never LP_SP.vTarget: vTarget is min()'d with the
    # cluster set speed itself, so a cluster walked toward it could only ratchet downward.
    # SCC curve targets are deliberately excluded — transient curve slowdowns should not churn
    # the physical cluster.
    assist = LP_SP.speedLimit.assist
    max_reasonable = 145 if self.is_metric else 90
    if assist.active and 0. < assist.vTarget < V_CRUISE_UNSET:
      raw_target = min(round(assist.vTarget * speed_conv) + self.alpha_long_offset, max_reasonable)
      # dwell: only adopt a changed target once it has held steady, so resolver source flaps
      # don't stream physical button presses at the PCM
      if raw_target != self._alpha_pending_target:
        self._alpha_pending_target = raw_target
        self._alpha_pending_frames = 0
      else:
        self._alpha_pending_frames += 1
      if self._alpha_pending_frames >= int(ALPHA_LONG_TARGET_DWELL / DT_CTRL):
        self.v_target = raw_target
        self._alpha_adopted = True
      elif not self._alpha_adopted:
        self.v_target = self.v_cruise_cluster
    else:
      # nothing to manage — leave the cluster where the driver put it
      self.v_target = self.v_cruise_cluster
      self._alpha_pending_target = 0
      self._alpha_pending_frames = 0
      self._alpha_adopted = False

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    # BluePilot: Capture initial cruise speed when cruise is first enabled
    # Use current speed (vEgo) as the initial setpoint if planner target is invalid/unreasonable
    cruise_enabled = CS.cruiseState.available and CS.cruiseState.enabled
    if cruise_enabled and not self.cruise_enabled_prev:
      # Cruise just enabled - capture current speed as initial setpoint
      current_speed_kph = CS.vEgo * CV.MS_TO_KPH
      self.initial_cruise_speed_kph = round(current_speed_kph)
    self.cruise_enabled_prev = cruise_enabled

    self.v_target_ms_last = apply_hysteresis(LP_SP.vTarget, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

    # BluePilot: If planner target is invalid/unreasonable and we have an initial cruise speed,
    # use the initial speed as the target (or cluster speed if it's been set)
    MAX_REASONABLE_TARGET = 145 if self.is_metric else 90
    if self.v_target >= MAX_REASONABLE_TARGET or self.v_target == 0:
      # Planner target is invalid - use initial cruise speed or cluster speed
      if self.initial_cruise_speed_kph > 0:
        self.v_target = self.initial_cruise_speed_kph
      elif self.v_cruise_cluster > 0:
        self.v_target = self.v_cruise_cluster

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster:
              # BluePilot: Prevent ICBM from increasing speed when cruise is first enabled
              # If cluster speed is 0 or very low, don't increase - wait for user to set initial speed
              # Also cap target to reasonable maximum (145 kph / 90 mph)
              # Don't increase if target exceeds initial cruise speed by more than 5 mph/kph
              MAX_REASONABLE_TARGET = 145 if self.is_metric else 90
              MAX_INITIAL_INCREASE = 5  # Allow small increases from initial speed

              # BluePilot: alpha-long targets are clamped to exactly MAX_REASONABLE_TARGET, so
              # only strictly-greater blocks there; stock mode keeps the original >= boundary
              at_max_cap = self.v_target > MAX_REASONABLE_TARGET if self._alpha_frame_active else \
                           self.v_target >= MAX_REASONABLE_TARGET
              if self.v_cruise_cluster == 0 or at_max_cap:
                # Don't increase - stay in preActive or go to holding
                self.state = State.holding
              elif not self._alpha_frame_active and self.initial_cruise_speed_kph > 0 and \
                   self.v_target > (self.initial_cruise_speed_kph + MAX_INITIAL_INCREASE):
                # Don't increase beyond initial cruise speed + small margin
                # This prevents ICBM from ramping up when cruise is first enabled.
                # BluePilot: not applied in alpha-long mode — there the target is the driver-confirmed
                # speed limit + margin, which may legitimately sit far above the engagement speed.
                self.state = State.holding
              else:
                self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    # BluePilot: Clear button timers when cruise is disabled to prevent stale presses
    # This ensures that when cruise is re-enabled, ICBM doesn't see stale button presses
    if not ready:
      for k in self.cruise_button_timers:
        self.cruise_button_timers[k] = 0
      # BluePilot: Reset initial cruise speed when cruise is disabled
      # This ensures we capture a fresh initial speed when cruise is re-enabled
      self.initial_cruise_speed_kph = 0

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    # BluePilot: pcmCruiseSpeed stays True under alpha long (flipping it would change engagement
    # and v_cruise semantics globally), so the alpha-long mode gets its own gate here. Snapshot
    # the mode once per frame — the params thread flips alpha_long_enabled asynchronously.
    self._alpha_frame_active = self.alpha_long_active

    if self.CP_SP.pcmCruiseSpeed and not self._alpha_frame_active:
      self.cruise_button = SendButtonState.none
      self.state = State.inactive
      return

    self.is_metric = is_metric

    if self._alpha_frame_active:
      self._update_alpha_long_calculations(CS, LP_SP)
    else:
      self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
