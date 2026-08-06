"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import pytest

from cereal import car, custom
from opendbc.car.car_helpers import interfaces
from opendbc.car.rivian.values import CAR as RIVIAN
from opendbc.car.tesla.values import CAR as TESLA
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_RECOMMENDED_SET_SPEED
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist, \
  PRE_ACTIVE_GUARD_PERIOD, ACTIVE_STATES, SET_SPEED_HINT_DELAY
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

ALL_STATES = tuple(SpeedLimitAssistState.schema.enumerants.values())

SPEED_LIMITS = {
  'residential': 25 * CV.MPH_TO_MS,  # 25 mph
  'city': 35 * CV.MPH_TO_MS,         # 35 mph
  'highway': 65 * CV.MPH_TO_MS,      # 65 mph
  'freeway': 80 * CV.MPH_TO_MS,      # 80 mph
}

DEFAULT_CAR = TOYOTA.TOYOTA_RAV4_TSS2


@pytest.fixture
def car_name(request):
  return getattr(request, "param", DEFAULT_CAR)


@pytest.fixture(autouse=True)
def set_car_name_on_instance(request, car_name):
  instance = getattr(request, "instance", None)
  if instance:
    instance.car_name = car_name


class TestSpeedLimitAssist:

  def setup_method(self, method):
    self.params = Params()
    self.reset_custom_params()
    self.events_sp = EventsSP()
    CI = self._setup_platform(self.car_name)
    self.sla = SpeedLimitAssist(CI.CP, CI.CP_SP)
    # deterministic clock: the 0.5 s button-hold window must not race wall time on loaded
    # runners. Starts positive — expired holds are stored as 0.
    self.now = 1000.
    self.sla._monotonic = lambda: self.now
    self.sla.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.sla.pcm_op_long] / DT_MDL)
    # a comfortable ceiling well above every fixture limit (80 mph); no longer a required value
    self.pcm_long_max_set_speed = PCM_LONG_RECOMMENDED_SET_SPEED[self.sla.is_metric]
    self.speed_conv = CV.MS_TO_KPH if self.sla.is_metric else CV.MS_TO_MPH

  def press_cruise_button(self, button_type=None):
    """Simulate a driver stalk press+release reaching SLA via CS.buttonEvents."""
    button_type = button_type if button_type is not None else ButtonType.decelCruise
    cs = car.CarState.new_message()
    events = cs.init('buttonEvents', 2)
    events[0].type = button_type
    events[0].pressed = True
    events[1].type = button_type
    events[1].pressed = False
    self.sla.update_car_state(cs)

  def teardown_method(self, method):
    self.reset_state()

  def _setup_platform(self, car_name):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)
    CI.CP.openpilotLongitudinalControl = True  # always assume it's openpilot longitudinal
    sunnypilot_interfaces.setup_interfaces(CI, self.params)
    return CI

  def reset_custom_params(self):
    self.params.put("IsReleaseSpBranch", True, block=True)
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    self.params.put_bool("IsMetric", False, block=True)
    self.params.put("SpeedLimitOffsetType", 0, block=True)
    self.params.put("SpeedLimitValueOffset", 0, block=True)

  def reset_state(self):
    self.sla.state = SpeedLimitAssistState.disabled
    self.sla.frame = -1
    self.sla.last_op_engaged_frame = 0
    self.sla.op_engaged = False
    self.sla.op_engaged_prev = False
    self.sla._speed_limit = 0.
    self.sla.speed_limit_prev = 0.
    self.sla.last_valid_speed_limit_offsetted = 0.
    self.sla._distance = 0.
    self.events_sp.clear()

  def initialize_active_state(self, initialize_v_cruise, confirmed_limit=None):
    self.sla.state = SpeedLimitAssistState.active
    self.sla.v_cruise_cluster = initialize_v_cruise
    self.sla.v_cruise_cluster_prev = initialize_v_cruise
    self.sla.prev_v_cruise_cluster_conv = round(initialize_v_cruise * self.speed_conv)
    if confirmed_limit is not None:
      # seed the change-tracking state a genuinely confirmed session would have
      self.sla.speed_limit_prev = confirmed_limit
      self.sla._speed_limit = confirmed_limit
      self.sla._speed_limit_final_last = confirmed_limit
      self.sla.prev_speed_limit_final_last_conv = round(confirmed_limit * self.speed_conv)

  def test_initial_state(self):
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert not self.sla.is_enabled
    assert not self.sla.is_active
    assert V_CRUISE_UNSET == self.sla.get_v_target_from_control()

  @pytest.mark.parametrize("car_name", [RIVIAN.RIVIAN_R1, TESLA.TESLA_MODEL_Y], indirect=True)
  def test_disallowed_brands(self, car_name):
    """
      Speed Limit Assist is disabled for the following brands and conditions:
      - All Tesla and is a release branch;
      - All Rivian
    """
    assert not self.sla.enabled

    # stay disallowed even when the param may have changed from somewhere else
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    for _ in range(int(PARAMS_UPDATE_PERIOD / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'],
                      SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert not self.sla.enabled

  def test_disabled(self):
    self.params.put("SpeedLimitMode", int(Mode.off), block=True)
    for _ in range(int(10. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.disabled

  def test_transition_disabled_to_preactive(self):
    for _ in range(int(3. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive
    assert self.sla.is_enabled and not self.sla.is_active

  def test_transition_disabled_to_pending_no_speed_limit_not_max_initial_set_speed(self):
    for _ in range(int(3. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['highway'], 0, SPEED_LIMITS['city'], 0, 0, False, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.pending
    assert self.sla.is_enabled and not self.sla.is_active

  def test_preactive_to_active_with_button_confirmation(self):
    # Any set speed over the limit works — consent is a single stalk press, in either direction
    self.sla.state = SpeedLimitAssistState.preActive
    self.press_cruise_button()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active
    assert self.sla.is_enabled and self.sla.is_active
    assert self.sla.output_v_target == SPEED_LIMITS['highway']

  def test_preactive_to_active_with_plus_button(self):
    self.sla.state = SpeedLimitAssistState.preActive
    self.press_cruise_button(ButtonType.accelCruise)
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  def test_cluster_matching_limit_is_not_consent(self):
    # a cluster merely equal to the limit must NOT auto-confirm: alpha-long ICBM parks the
    # cluster at limit + offset, so equality would silently confirm every +offset limit step
    self.sla.state = SpeedLimitAssistState.preActive
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive

  def test_preactive_no_confirmation_without_button(self):
    # A high cluster alone is no longer consent: without a press, preActive persists
    self.sla.state = SpeedLimitAssistState.preActive
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive

  def test_any_set_speed_above_limit_activates(self):
    # Feature A: a modest ceiling (45 mph over a 35 mph limit) activates just like 80 mph did
    modest_cluster = 45 * CV.MPH_TO_MS
    self.sla.state = SpeedLimitAssistState.preActive
    self.press_cruise_button()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, modest_cluster, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active
    assert self.sla.output_v_target == SPEED_LIMITS['city']

  def test_preactive_timeout_to_inactive(self):
    self.sla.state = SpeedLimitAssistState.preActive
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)

    for _ in range(int(PRE_ACTIVE_GUARD_PERIOD[self.sla.pcm_op_long] / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.inactive

  def test_preactive_to_pending_no_speed_limit(self):
    self.sla.state = SpeedLimitAssistState.preActive
    self.sla.update(True, False, SPEED_LIMITS['highway'], 0, self.pcm_long_max_set_speed, 0, 0, False, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.pending
    assert self.sla.is_enabled and not self.sla.is_active

  def test_pending_to_preactive_when_speed_limit_available(self):
    # A newly acquired limit now always goes through the confirm handshake
    self.sla.state = SpeedLimitAssistState.pending
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    self.sla.update(True, False, SPEED_LIMITS['highway'], 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive

    self.press_cruise_button()
    self.sla.update(True, False, SPEED_LIMITS['highway'], 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  def test_pending_to_adapting_when_below_speed_limit(self):
    self.sla.state = SpeedLimitAssistState.pending
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    self.sla.update(True, False, SPEED_LIMITS['highway'] + 5, 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive

    self.press_cruise_button()
    self.sla.update(True, False, SPEED_LIMITS['highway'] + 5, 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting
    assert self.sla.is_enabled and self.sla.is_active

  def test_active_to_adapting_transition(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    self.sla.update(True, False, SPEED_LIMITS['highway'] + 2, 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting

  def test_adapting_to_active_transition(self):
    self.sla.state = SpeedLimitAssistState.adapting
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  def test_cluster_change_never_deactivates_pcm_long(self):
    # Under pcm long the cluster is only a ceiling: raising it (as the prompts instruct) or
    # lowering it (the planner min() then caps the car) must never deactivate SLA — even right
    # after a driver stalk press
    for different_cruise in (SPEED_LIMITS['highway'] + 5, SPEED_LIMITS['highway'] - 5):
      self.reset_state()
      self.initialize_active_state(SPEED_LIMITS['highway'], confirmed_limit=SPEED_LIMITS['city'])
      self.press_cruise_button()
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, different_cruise, SPEED_LIMITS['city'],
                      SPEED_LIMITS['city'], True, 0, self.events_sp)
      assert self.sla.state in ACTIVE_STATES

  def test_confirm_press_cluster_movement_does_not_deactivate(self):
    # The confirm press itself nudges the physical cluster a beat later; that movement must not
    # be read as an override
    self.sla.state = SpeedLimitAssistState.preActive
    self.press_cruise_button()
    cluster = self.pcm_long_max_set_speed
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, cluster, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

    # next frame: the press has now moved the cluster down one increment
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, cluster - 1 * CV.MPH_TO_MS, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES

  def test_active_limit_drop_auto_applies(self):
    # once active, limit changes apply directly — chime + sign flash only, no re-confirm prompt
    self.initialize_active_state(self.pcm_long_max_set_speed, confirmed_limit=SPEED_LIMITS['highway'])

    self.events_sp.clear()
    self.sla.update(True, False, SPEED_LIMITS['highway'], 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['residential'], SPEED_LIMITS['residential'], True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == SPEED_LIMITS['residential']
    # notified via the chime-only events, never the preActive text prompt
    assert EventNameSP.speedLimitPreActive not in self.events_sp.names
    assert (EventNameSP.speedLimitActive in self.events_sp.names or
            EventNameSP.speedLimitChanged in self.events_sp.names)

  def test_active_limit_rise_auto_applies(self):
    self.initialize_active_state(self.pcm_long_max_set_speed, confirmed_limit=SPEED_LIMITS['city'])

    self.events_sp.clear()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == SPEED_LIMITS['highway']
    assert EventNameSP.speedLimitPreActive not in self.events_sp.names

  def test_active_limit_loss_goes_pending(self):
    # coverage gap outlasting the resolver hold: the cap must release via pending (audible),
    # never silently while still reporting active
    self.initialize_active_state(self.pcm_long_max_set_speed, confirmed_limit=SPEED_LIMITS['highway'])
    self.sla.update(True, False, SPEED_LIMITS['highway'], 0, self.pcm_long_max_set_speed, 0, 0, False, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.pending
    assert not self.sla.is_active
    assert self.sla.output_v_target == V_CRUISE_UNSET

  def test_adapting_limit_loss_goes_pending(self):
    self.initialize_active_state(self.pcm_long_max_set_speed, confirmed_limit=SPEED_LIMITS['highway'])
    self.sla.state = SpeedLimitAssistState.adapting
    self.sla.update(True, False, SPEED_LIMITS['freeway'], 0, self.pcm_long_max_set_speed, 0, 0, False, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.pending

  def test_raw_limit_flicker_does_not_disturb_active(self):
    # one-frame raw dropout bridged by the resolver hold (final_last unchanged): SLA must keep
    # capping at the held limit instead of re-arming the handshake
    self.initialize_active_state(self.pcm_long_max_set_speed, confirmed_limit=SPEED_LIMITS['city'])
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0,
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == SPEED_LIMITS['city']

  def test_engagement_press_is_not_consent(self):
    # the SET press that engages cruise arms a 0.5 s hold; entering preActive clears it, so
    # SLA must wait for a separate confirm press instead of instantly self-confirming
    self.press_cruise_button()  # engagement press
    for _ in range(int(2. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                      SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive

  def test_button_hold_expires(self):
    # a press only counts within CRUISE_BUTTON_CONFIRM_HOLD of its release
    self.sla.state = SpeedLimitAssistState.preActive
    self.press_cruise_button()
    self.now += 0.6
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive

  def test_inactive_recovers_on_new_limit(self):
    # inactive is no longer a dead end: a new limit re-arms the confirm handshake
    self.sla.state = SpeedLimitAssistState.inactive
    self.sla.speed_limit_prev = SPEED_LIMITS['highway']
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive

  def _consent_to_active(self):
    """Confirm the handshake once: preActive + fresh press -> active with consent latched."""
    self.sla.state = SpeedLimitAssistState.preActive
    self.press_cruise_button()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  def test_consent_latched_through_limit_gap(self):
    # once the driver consents, a map-coverage gap must not demand another press: the
    # reacquired limit re-caps directly with the chime, never the preActive prompt
    self._consent_to_active()

    # coverage gap outlasting the resolver hold
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0, 0, False, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.pending

    self.events_sp.clear()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active
    assert self.sla.output_v_target == SPEED_LIMITS['highway']
    assert EventNameSP.speedLimitPreActive not in self.events_sp.names
    assert (EventNameSP.speedLimitActive in self.events_sp.names or
            EventNameSP.speedLimitChanged in self.events_sp.names)

  def test_consent_cleared_on_disengagement(self):
    # disengaging ends the consent session: the next engagement confirms afresh
    self._consent_to_active()

    self.sla.update(False, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.disabled

    # re-engage: after the engage guard, a limit re-arms the confirm handshake — active
    # only after a fresh press
    for _ in range(int(2. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                      SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive

    self.press_cruise_button()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  def test_consent_cleared_when_sla_disabled(self):
    # turning Speed Limit Assist off mid-drive also ends the consent session
    self._consent_to_active()
    self.sla.enabled = False
    self.sla.update_state_machine_pcm_op_long()
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert not self.sla._pcm_long_consented

  def test_raise_set_speed_prompt_once_per_episode(self):
    # cluster below the limit: prompt exactly once until the cluster recovers above the limit
    cluster = 30 * CV.MPH_TO_MS  # below the 35 mph city limit
    self.initialize_active_state(cluster, confirmed_limit=SPEED_LIMITS['city'])

    self.events_sp.clear()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, cluster, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert EventNameSP.speedLimitRaiseSetSpeed in self.events_sp.names

    self.events_sp.clear()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, cluster, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert EventNameSP.speedLimitRaiseSetSpeed not in self.events_sp.names

  def _drive_to_active_with_hint_window(self, modest_cluster):
    self.sla.state = SpeedLimitAssistState.preActive
    self.press_cruise_button()
    hint_seen = False
    for _ in range(int((SET_SPEED_HINT_DELAY + 1.) / DT_MDL)):
      self.events_sp.clear()
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, modest_cluster, SPEED_LIMITS['city'],
                      SPEED_LIMITS['city'], True, 0, self.events_sp)
      hint_seen = hint_seen or EventNameSP.speedLimitSetSpeedHint in self.events_sp.names
    return hint_seen

  def test_set_speed_hint_once_per_drive(self):
    # one-shot ceiling tip fires shortly after the FIRST activation of the drive when the
    # cluster is below the recommended value — never again, even across re-engagements
    modest_cluster = 45 * CV.MPH_TO_MS
    assert self._drive_to_active_with_hint_window(modest_cluster)
    assert self.sla.state in ACTIVE_STATES

    # disengage and re-engage: the tip must not repeat
    self.sla.update(False, False, SPEED_LIMITS['city'], 0, modest_cluster, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert not self._drive_to_active_with_hint_window(modest_cluster)

  def test_suggested_set_speed_published(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    limit_conv = round(SPEED_LIMITS['city'] * self.speed_conv)
    assert self.sla.suggested_set_speed_conv == limit_conv + self.sla.alpha_long_offset

  # TODO-SP: test lower CST cases
  def test_rapid_speed_limit_changes(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)
    speed_limits = [SPEED_LIMITS['highway'], SPEED_LIMITS['freeway']]

    for _, speed_limit in enumerate(speed_limits):
      self.sla.update(True, False, speed_limit, 0, self.pcm_long_max_set_speed, speed_limit, speed_limit, True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES

  def test_invalid_speed_limits_handling(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    invalid_limits = [-10, 0, 200 * CV.MPH_TO_MS]

    for invalid_limit in invalid_limits:
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, invalid_limit, SPEED_LIMITS['city'], True, 0, self.events_sp)
      assert isinstance(self.sla.output_v_target, (int, float))
      assert self.sla.output_v_target == V_CRUISE_UNSET or self.sla.output_v_target > 0

  def test_stale_data_handling(self):
    old_speed_limit = SPEED_LIMITS['city']
    self.initialize_active_state(self.pcm_long_max_set_speed, confirmed_limit=old_speed_limit)

    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0, old_speed_limit, True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == old_speed_limit

  def test_distance_based_adapting(self):
    self.sla.state = SpeedLimitAssistState.adapting
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    distance = 100.0
    current_speed = SPEED_LIMITS['freeway']
    target_speed = SPEED_LIMITS['highway']

    self.sla.update(True, False, current_speed, 0, self.pcm_long_max_set_speed, target_speed, target_speed, True, distance, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting
    assert self.sla.output_v_target == target_speed  # TODO-SP: assert expected accel, need to enable self.acceleration_solutions

  def test_long_disengaged_to_disabled(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    self.sla.update(False, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert self.sla.output_v_target == V_CRUISE_UNSET

  def test_maintain_states_with_no_changes(self):
    """Test that states are maintained when no significant changes occur"""
    test_states = [
      SpeedLimitAssistState.preActive,
      SpeedLimitAssistState.pending,
      SpeedLimitAssistState.active,
      SpeedLimitAssistState.adapting
    ]

    for state in test_states:
      self.sla.state = state
      self.sla.op_engaged = True

      initial_state = state

      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)

      assert self.sla.state in ALL_STATES  # Sanity check

      if initial_state == SpeedLimitAssistState.preActive:
        assert self.sla.state in [SpeedLimitAssistState.preActive, SpeedLimitAssistState.active]
      elif initial_state in ACTIVE_STATES:
        assert self.sla.state in ACTIVE_STATES

  def test_stale_limit_chimes_once_and_keeps_capping(self):
    # map data goes stale while SLA is capping: the cap must hold (releasing it snaps the car up
    # to the cruise set speed) and the driver gets exactly one downbeat chime
    self._consent_to_active()
    self.events_sp.clear()

    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0,
                    SPEED_LIMITS['city'], True, 0, self.events_sp, True)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == SPEED_LIMITS['city']  # still capped at the held limit
    assert EventNameSP.speedLimitLost in self.events_sp.names

    # every later stale frame is silent
    for _ in range(int(3. / DT_MDL)):
      self.events_sp.clear()
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0,
                      SPEED_LIMITS['city'], True, 0, self.events_sp, True)
      assert EventNameSP.speedLimitLost not in self.events_sp.names

  def test_stale_chime_rearms_after_data_returns(self):
    self._consent_to_active()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0,
                    SPEED_LIMITS['city'], True, 0, self.events_sp, True)

    # live data returns, then is lost again — the second loss chimes again
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp, False)
    self.events_sp.clear()
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0,
                    SPEED_LIMITS['city'], True, 0, self.events_sp, True)
    assert EventNameSP.speedLimitLost in self.events_sp.names

  def test_stale_limit_does_not_chime_when_not_capping(self):
    # SLA inactive (never confirmed): a stale limit it is not enforcing must stay silent
    self.sla.state = SpeedLimitAssistState.preActive
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0,
                    SPEED_LIMITS['city'], True, 0, self.events_sp, True)
    assert not self.sla.is_active
    assert EventNameSP.speedLimitLost not in self.events_sp.names
