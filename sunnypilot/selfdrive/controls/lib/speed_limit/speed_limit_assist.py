"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from cereal import custom, car
from openpilot.common.params import Params
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_RECOMMENDED_SET_SPEED, CONFIRM_SPEED_THRESHOLD
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target, set_speed_limit_assist_availability

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source

ACTIVE_STATES = (SpeedLimitAssistState.active, SpeedLimitAssistState.adapting)
ENABLED_STATES = (SpeedLimitAssistState.preActive, SpeedLimitAssistState.pending, *ACTIVE_STATES)

DISABLED_GUARD_PERIOD = 0.5  # secs.
# secs. Time to wait after activation before considering temp deactivation signal.
PRE_ACTIVE_GUARD_PERIOD = {
  True: 15,
  False: 5,
}
SPEED_LIMIT_CHANGED_HOLD_PERIOD = 1  # secs. Time to wait after speed limit change before switching to preActive.

LIMIT_MIN_ACC = -1.5  # m/s^2 Maximum deceleration allowed for limit controllers to provide.
LIMIT_MAX_ACC = 1.0   # m/s^2 Maximum acceleration allowed for limit controllers to provide while active.
LIMIT_MIN_SPEED = 8.33  # m/s, Minimum speed limit to provide as solution on limit controllers.
LIMIT_SPEED_OFFSET_TH = -1.  # m/s Maximum offset between speed limit and current speed for adapting state.
V_CRUISE_UNSET = 255.

CRUISE_BUTTONS_PLUS = (ButtonType.accelCruise, ButtonType.resumeCruise)
CRUISE_BUTTONS_MINUS = (ButtonType.decelCruise, ButtonType.setCruise)
CRUISE_BUTTON_CONFIRM_HOLD = 0.5  # secs.

# BluePilot: pcm-long consent handshake tuning.
SET_SPEED_HINT_DELAY = 1.5  # secs. Delay after activation before the one-shot ceiling hint.


class SpeedLimitAssist:
  _speed_limit_final_last: float
  _distance: float
  v_ego: float
  a_ego: float
  v_offset: float

  def __init__(self, CP: car.CarParams, CP_SP: custom.CarParamsSP):
    self.params = Params()
    self.CP = CP
    self.CP_SP = CP_SP
    self.frame = -1
    self.long_engaged_timer = 0
    self.pre_active_timer = 0
    self.is_metric = self.params.get_bool("IsMetric")
    set_speed_limit_assist_availability(self.CP, self.CP_SP, self.params)
    self.enabled = self.params.get("SpeedLimitMode", return_default=True) == Mode.assist
    self.long_enabled = False
    self.long_enabled_prev = False
    self.is_enabled = False
    self.is_active = False
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.
    self.v_ego = 0.
    self.a_ego = 0.
    self.v_offset = 0.
    self.target_set_speed_conv = 0
    self.prev_target_set_speed_conv = 0
    self.v_cruise_cluster = 0.
    self.v_cruise_cluster_prev = 0.
    self.v_cruise_cluster_conv = 0
    self.prev_v_cruise_cluster_conv = 0
    self._has_speed_limit = False
    self._speed_limit = 0.
    self._speed_limit_final_last = 0.
    self.speed_limit_prev = 0.
    self.speed_limit_final_last_conv = 0
    self.prev_speed_limit_final_last_conv = 0
    self._distance = 0.
    self.state = SpeedLimitAssistState.disabled
    self._state_prev = SpeedLimitAssistState.disabled
    self.pcm_op_long = CP.openpilotLongitudinalControl and CP.pcmCruise

    self._plus_hold = 0.
    self._minus_hold = 0.
    self._last_carstate_ts = 0.

    # BluePilot: injectable clock so tests can drive the button-hold windows deterministically
    self._monotonic = time.monotonic

    # BluePilot: pcm-long "any speed over the limit" handshake state
    self._set_speed_hint_shown = False
    self._set_speed_hint_frames = -1
    self._raise_set_speed_prompted = False
    self.alpha_long_offset = int(self.params.get("AlphaLongIcbmOffset", return_default=True))
    self.suggested_set_speed_conv = 0
    # ICBM manages the cluster under alpha long: ceiling prompts are its job, not the driver's
    self.icbm_alpha_managing = self.pcm_op_long and self.CP_SP.intelligentCruiseButtonManagementAvailable and \
      self.params.get_bool("IntelligentCruiseButtonManagement")

    # BluePilot: last speed limit (converted/rounded) we notified the driver about while active.
    # Prevents re-alerting when map data flickers between "no limit" and the same limit, or when
    # the resolved source flaps between car and map with sub-rounding float differences.
    self._last_event_limit_conv = 0

    # TODO-SP: SLA's own output_a_target for planner
    # Solution functions mapped to respective states
    self.acceleration_solutions = {
      SpeedLimitAssistState.disabled: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.inactive: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.preActive: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.pending: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.adapting: self.get_adapting_state_target_acceleration,
      SpeedLimitAssistState.active: self.get_active_state_target_acceleration,
    }

  @property
  def speed_limit_changed(self) -> bool:
    # BluePilot: compare the held target in display units rather than the raw resolver value.
    # The resolver bridges short coverage gaps by holding speed_limit_final_last, so a raw
    # 0-crossing flicker on the same posted limit must not re-arm the confirm handshake or
    # drop an active cap; sub-rounding source flaps (car <-> map) are also ignored.
    return self._has_speed_limit and bool(self.speed_limit_final_last_conv != self.prev_speed_limit_final_last_conv)

  @property
  def v_cruise_cluster_changed(self) -> bool:
    return bool(self.v_cruise_cluster_conv != self.prev_v_cruise_cluster_conv)

  @property
  def target_set_speed_confirmed(self) -> bool:
    return bool(self.v_cruise_cluster_conv == self.target_set_speed_conv)

  @property
  def v_cruise_cluster_below_confirm_speed_threshold(self) -> bool:
    return bool(self.v_cruise_cluster_conv < CONFIRM_SPEED_THRESHOLD[self.is_metric])

  def update_active_event(self, events_sp: EventsSP) -> None:
    if self.v_cruise_cluster_below_confirm_speed_threshold:
      events_sp.add(EventNameSP.speedLimitChanged)
    else:
      events_sp.add(EventNameSP.speedLimitActive)
    self._last_event_limit_conv = self.speed_limit_final_last_conv

  def get_v_target_from_control(self) -> float:
    # BluePilot: pcm long now also requires is_active (was is_enabled) — no capping to a limit
    # the driver has not confirmed yet. Once active, limit changes apply directly through
    # speed_limit_final_last; the confirm handshake only gates initial activation.
    if self._has_speed_limit and self.is_active:
      return self._speed_limit_final_last

    # Fallback
    return V_CRUISE_UNSET

  # TODO-SP: SLA's own output_a_target for planner
  def get_a_target_from_control(self) -> float:
    return self.a_ego

  def update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.is_metric = self.params.get_bool("IsMetric")
      set_speed_limit_assist_availability(self.CP, self.CP_SP, self.params)
      self.enabled = self.params.get("SpeedLimitMode", return_default=True) == Mode.assist
      self.alpha_long_offset = int(self.params.get("AlphaLongIcbmOffset", return_default=True))
      self.icbm_alpha_managing = self.pcm_op_long and self.CP_SP.intelligentCruiseButtonManagementAvailable and \
        self.params.get_bool("IntelligentCruiseButtonManagement")

  def update_car_state(self, CS: car.CarState) -> None:
    now = self._monotonic()
    self._last_carstate_ts = now

    for b in CS.buttonEvents:
      if not b.pressed:
        if b.type in CRUISE_BUTTONS_PLUS:
          self._plus_hold = max(self._plus_hold, now + CRUISE_BUTTON_CONFIRM_HOLD)
        elif b.type in CRUISE_BUTTONS_MINUS:
          self._minus_hold = max(self._minus_hold, now + CRUISE_BUTTON_CONFIRM_HOLD)

  def _get_button_release(self, req_plus: bool, req_minus: bool) -> bool:
    now = self._monotonic()
    if req_plus and now <= self._plus_hold:
      self._plus_hold = 0.
      return True
    elif req_minus and now <= self._minus_hold:
      self._minus_hold = 0.
      return True

    # expired
    if now > self._plus_hold:
      self._plus_hold = 0.
    if now > self._minus_hold:
      self._minus_hold = 0.
    return False

  def update_calculations(self, v_cruise_cluster: float) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    self.v_cruise_cluster = v_cruise_cluster

    # Update current velocity offset (error)
    self.v_offset = self._speed_limit_final_last - self.v_ego

    self.speed_limit_final_last_conv = round(self._speed_limit_final_last * speed_conv)
    self.v_cruise_cluster_conv = round(self.v_cruise_cluster * speed_conv)

    # BluePilot: the target is the limit itself on both paths. Under pcm long the cluster is only
    # a ceiling — consent comes from a stalk press, not from dialing the cluster to 120/130.
    self.target_set_speed_conv = self.speed_limit_final_last_conv

    # Suggested cluster set speed while SLA drives to the limit: limit + margin, so the PCM's
    # near-set-speed accel gate never binds at the limit. 0 when there is nothing to suggest.
    if self.pcm_op_long and self._has_speed_limit and self.speed_limit_final_last_conv > 0:
      self.suggested_set_speed_conv = self.speed_limit_final_last_conv + self.alpha_long_offset
    else:
      self.suggested_set_speed_conv = 0

  @property
  def apply_confirm_speed_threshold(self) -> bool:
    # below CST: always require user confirmation
    if self.v_cruise_cluster_below_confirm_speed_threshold:
      return True

    # at/above CST:
    # - new speed limit >= CST: auto change
    # - new speed limit < CST: user confirmation required
    return bool(self.speed_limit_final_last_conv < CONFIRM_SPEED_THRESHOLD[self.is_metric])

  def get_current_acceleration_as_target(self) -> float:
    return self.a_ego

  def get_adapting_state_target_acceleration(self) -> float:
    if self._distance > 0:
      return (self._speed_limit_final_last ** 2 - self.v_ego ** 2) / (2. * self._distance)

    return self.v_offset / float(ModelConstants.T_IDXS[CONTROL_N])

  def get_active_state_target_acceleration(self) -> float:
    return self.v_offset / float(ModelConstants.T_IDXS[CONTROL_N])

  def _update_confirmed_state(self):
    if self._has_speed_limit:
      if self.v_offset < LIMIT_SPEED_OFFSET_TH:
        self.state = SpeedLimitAssistState.adapting
      else:
        self.state = SpeedLimitAssistState.active
    else:
      self.state = SpeedLimitAssistState.pending

  def _update_non_pcm_long_confirmed_state(self) -> bool:
    if self.target_set_speed_confirmed:
      return True

    if self.state != SpeedLimitAssistState.preActive:
      return False

    req_plus, req_minus = compare_cluster_target(self.v_cruise_cluster, self._speed_limit_final_last, self.is_metric)

    return self._get_button_release(req_plus, req_minus)

  def _pcm_long_consent(self) -> bool:
    # BluePilot: same press-again-to-confirm handshake the non-pcm/ICBM path uses, but
    # direction-agnostic — the cluster stays where the driver set it (any speed over the limit)
    # and only serves as a ceiling, so either SET+ or SET- signals consent. A cluster merely
    # equal to the limit is deliberately NOT consent: alpha-long ICBM parks the cluster at
    # limit + offset, so with the default +5 offset a bare equality check would silently
    # auto-confirm every standard +5 limit step.
    return self._get_button_release(True, True)

  def _enter_pre_active(self) -> None:
    self.state = SpeedLimitAssistState.preActive
    self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)
    # only presses made after the confirm prompt appears may count as consent — in particular
    # the press that engaged/resumed cruise must never be consumed as confirmation
    self._plus_hold = 0.
    self._minus_hold = 0.

  def update_state_machine_pcm_op_long(self):
    self.long_engaged_timer = max(0, self.long_engaged_timer - 1)
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # ACTIVE, ADAPTING, PENDING, PRE_ACTIVE, INACTIVE
    if self.state != SpeedLimitAssistState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = SpeedLimitAssistState.disabled

      else:
        # ACTIVE
        # BluePilot: cluster changes never deactivate under pcm long — the cluster is only a
        # ceiling (raising it is exactly what the prompts instruct; lowering it caps the car via
        # the planner min() with SLA still armed). Overrides are the accelerator (long override)
        # or turning Speed Limit Assist off. Losing the limit (coverage gap past the resolver
        # hold) drops to pending, which releases the cap audibly instead of silently.
        # Once active, limit changes auto-apply in BOTH directions with only the chime and the
        # sign-overlay flash — the confirm handshake exists solely for initial activation.
        if self.state == SpeedLimitAssistState.active:
          if not self._has_speed_limit:
            self.state = SpeedLimitAssistState.pending
          elif self.v_offset < LIMIT_SPEED_OFFSET_TH:
            self.state = SpeedLimitAssistState.adapting

        # ADAPTING
        elif self.state == SpeedLimitAssistState.adapting:
          if not self._has_speed_limit:
            self.state = SpeedLimitAssistState.pending
          elif self.v_offset >= LIMIT_SPEED_OFFSET_TH:
            self.state = SpeedLimitAssistState.active

        # PENDING
        elif self.state == SpeedLimitAssistState.pending:
          if self.speed_limit_changed:
            self._enter_pre_active()

        # PRE_ACTIVE
        # BluePilot: consent is a single fresh SET+/SET- press — the old exact 120/130 km/h
        # cluster match had no hardware basis and forced constant toggling between two set speeds.
        elif self.state == SpeedLimitAssistState.preActive:
          if not self._has_speed_limit:
            self.state = SpeedLimitAssistState.pending
          elif self._pcm_long_consent():
            self._update_confirmed_state()
          elif self.pre_active_timer <= 0:
            # Timeout - session ended
            self.state = SpeedLimitAssistState.inactive

        # INACTIVE
        # BluePilot: recoverable (was a dead end until re-engagement) — a new limit re-arms the
        # confirm handshake, mirroring the non-pcm state machine.
        elif self.state == SpeedLimitAssistState.inactive:
          if self.speed_limit_changed:
            self._enter_pre_active()

    # DISABLED
    elif self.state == SpeedLimitAssistState.disabled:
      if self.long_enabled and self.enabled:
        # start or reset preActive timer if initially enabled or manual set speed change detected
        if not self.long_enabled_prev or self.v_cruise_cluster_changed:
          self.long_engaged_timer = int(DISABLED_GUARD_PERIOD / DT_MDL)

        elif self.long_engaged_timer <= 0:
          if self._has_speed_limit:
            self._enter_pre_active()
          else:
            self.state = SpeedLimitAssistState.pending

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update_state_machine_non_pcm_long(self):
    self.long_engaged_timer = max(0, self.long_engaged_timer - 1)
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # ACTIVE, ADAPTING, PENDING, PRE_ACTIVE, INACTIVE
    if self.state != SpeedLimitAssistState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = SpeedLimitAssistState.disabled

      else:
        # ACTIVE
        if self.state == SpeedLimitAssistState.active:
          if self.v_cruise_cluster_changed:
            self.state = SpeedLimitAssistState.inactive

          # BluePilot: losing the limit (coverage gap past the resolver hold) must leave active,
          # otherwise the cap silently lifts while SLA still reports active
          elif not self._has_speed_limit:
            self.state = SpeedLimitAssistState.inactive

          elif self.speed_limit_changed and self.apply_confirm_speed_threshold:
            self.state = SpeedLimitAssistState.preActive
            self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)

        # PRE_ACTIVE
        elif self.state == SpeedLimitAssistState.preActive:
          if self._update_non_pcm_long_confirmed_state():
            self.state = SpeedLimitAssistState.active
          elif self.pre_active_timer <= 0:
            # Timeout - session ended
            self.state = SpeedLimitAssistState.inactive

        # INACTIVE
        elif self.state == SpeedLimitAssistState.inactive:
          if self.speed_limit_changed:
            self.state = SpeedLimitAssistState.preActive
            self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)
          elif self._update_non_pcm_long_confirmed_state():
            self.state = SpeedLimitAssistState.active

    # DISABLED
    elif self.state == SpeedLimitAssistState.disabled:
      if self.long_enabled and self.enabled:
        # start or reset preActive timer if initially enabled or manual set speed change detected
        if not self.long_enabled_prev or self.v_cruise_cluster_changed:
          self.long_engaged_timer = int(DISABLED_GUARD_PERIOD / DT_MDL)

        elif self.long_engaged_timer <= 0:
          if self._update_non_pcm_long_confirmed_state():
            self.state = SpeedLimitAssistState.active
          elif self._has_speed_limit:
            self.state = SpeedLimitAssistState.preActive
            self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)
          else:
            self.state = SpeedLimitAssistState.inactive

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def _update_pcm_long_prompts(self, events_sp: EventsSP) -> None:
    # BluePilot: one-shot, non-nagging prompts for the pcm-long "cluster is only a ceiling" flow.
    # The ceiling hint latches for the whole drive (process lifetime) — it must not repeat on
    # every re-engagement.
    if not self.is_active:
      self._set_speed_hint_frames = -1
      self._raise_set_speed_prompted = False
      return

    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    recommended_conv = round(PCM_LONG_RECOMMENDED_SET_SPEED[self.is_metric] * speed_conv)

    # When alpha-long ICBM manages the cluster, ceiling adjustments are its job — prompting the
    # driver to move the cluster would only fight the automation.
    if self.icbm_alpha_managing:
      return

    # Suggest a high ceiling once per drive, shortly after the first activation so it does not
    # fight the activation chime. If the driver ignores it, never repeat it.
    if self._state_prev not in ACTIVE_STATES and not self._set_speed_hint_shown \
       and self.v_cruise_cluster_conv < recommended_conv:
      self._set_speed_hint_frames = int(SET_SPEED_HINT_DELAY / DT_MDL)
    if self._set_speed_hint_frames > 0:
      self._set_speed_hint_frames -= 1
      if self._set_speed_hint_frames == 0 and not self._set_speed_hint_shown:
        events_sp.add(EventNameSP.speedLimitSetSpeedHint)
        self._set_speed_hint_shown = True

    # The cluster is a hard ceiling: below the limit, SLA cannot reach it. Prompt once per
    # below-limit episode to raise the set speed to limit + margin. Complying never deactivates
    # SLA — cluster changes are not overrides under pcm long.
    cluster_below_target = self._has_speed_limit and 0 < self.v_cruise_cluster_conv < self.speed_limit_final_last_conv
    if cluster_below_target and not self._raise_set_speed_prompted:
      events_sp.add(EventNameSP.speedLimitRaiseSetSpeed)
      self._raise_set_speed_prompted = True
    elif not cluster_below_target:
      self._raise_set_speed_prompted = False

  def update_events(self, events_sp: EventsSP) -> None:
    if self.state == SpeedLimitAssistState.preActive:
      events_sp.add(EventNameSP.speedLimitPreActive)

    if self.state == SpeedLimitAssistState.pending and self._state_prev != SpeedLimitAssistState.pending:
      events_sp.add(EventNameSP.speedLimitPending)

    if self.pcm_op_long:
      self._update_pcm_long_prompts(events_sp)

    if self.is_active:
      if self._state_prev not in ACTIVE_STATES:
        self.update_active_event(events_sp)

      # only notify if we acquire a valid speed limit
      # do not check has_speed_limit here
      elif self._speed_limit != self.speed_limit_prev:
        # BluePilot: only re-notify when the target actually changes in display units. Raw float
        # inequality re-fires on source flaps (car<->map) and on 0 -> limit flicker over OSM
        # coverage gaps, spamming a chime+alert every few seconds on the same posted limit.
        if self.speed_limit_final_last_conv != self._last_event_limit_conv:
          if self.speed_limit_prev <= 0:
            self.update_active_event(events_sp)
          elif self.speed_limit_prev > 0 and self._speed_limit > 0:
            self.update_active_event(events_sp)
    else:
      self._last_event_limit_conv = 0

  def update(self, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise_cluster: float, speed_limit: float,
             speed_limit_final_last: float, has_speed_limit: bool, distance: float, events_sp: EventsSP) -> None:
    self.long_enabled = long_enabled
    self.v_ego = v_ego
    self.a_ego = a_ego

    self._has_speed_limit = has_speed_limit
    self._speed_limit = speed_limit
    self._speed_limit_final_last = speed_limit_final_last
    self._distance = distance

    self.update_params()
    self.update_calculations(v_cruise_cluster)

    self._state_prev = self.state
    if self.pcm_op_long:
      self.is_enabled, self.is_active = self.update_state_machine_pcm_op_long()
    else:
      self.is_enabled, self.is_active = self.update_state_machine_non_pcm_long()

    self.update_events(events_sp)

    # Update change tracking variables
    self.speed_limit_prev = self._speed_limit
    self.v_cruise_cluster_prev = self.v_cruise_cluster
    self.long_enabled_prev = self.long_enabled
    self.prev_target_set_speed_conv = self.target_set_speed_conv
    self.prev_v_cruise_cluster_conv = self.v_cruise_cluster_conv
    self.prev_speed_limit_final_last_conv = self.speed_limit_final_last_conv

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
