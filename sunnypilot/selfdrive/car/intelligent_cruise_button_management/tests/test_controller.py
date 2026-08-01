"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  IntelligentCruiseButtonManagement, INACTIVE_TIMER, ALPHA_LONG_TARGET_DWELL)

ButtonType = car.CarState.ButtonEvent.Type
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

PRE_ACTIVE_FRAMES = int(INACTIVE_TIMER / DT_CTRL) + 2
DWELL_FRAMES = int(ALPHA_LONG_TARGET_DWELL / DT_CTRL) + 2
WALK_FRAMES = DWELL_FRAMES + PRE_ACTIVE_FRAMES


def make_cs(cluster_kph: float) -> car.CarState:
  cs = car.CarState.new_message()
  cs.cruiseState.available = True
  cs.cruiseState.enabled = True
  cs.cruiseState.speedCluster = cluster_kph * CV.KPH_TO_MS
  cs.vEgo = cluster_kph * CV.KPH_TO_MS
  return cs


def make_cc(enabled: bool = True) -> car.CarControl:
  cc = car.CarControl.new_message()
  cc.enabled = enabled
  return cc


def make_lp(sla_active: bool = False, sla_target_kph: float = 0.) -> custom.LongitudinalPlanSP:
  lp = custom.LongitudinalPlanSP.new_message()
  lp.speedLimit.assist.active = sla_active
  lp.speedLimit.assist.vTarget = sla_target_kph * CV.KPH_TO_MS if sla_active else 255.
  lp.vTarget = lp.speedLimit.assist.vTarget
  return lp


def make_icbm(op_long: bool = True, icbm_available: bool = True, pcm_cruise_speed: bool = True,
              alpha_enabled: bool = True) -> IntelligentCruiseButtonManagement:
  CP = structs.CarParams(openpilotLongitudinalControl=op_long)
  CP_SP = structs.CarParamsSP(pcmCruiseSpeed=pcm_cruise_speed,
                              intelligentCruiseButtonManagementAvailable=icbm_available)
  icbm = IntelligentCruiseButtonManagement(CP, CP_SP)
  icbm.alpha_long_enabled = alpha_enabled
  # CRUISE_BUTTON_TIMER is a module-level dict shared by reference; zero it for isolation
  for k in icbm.cruise_button_timers:
    icbm.cruise_button_timers[k] = 0
  return icbm


def drive(icbm, cs, cc, lp, frames: int) -> None:
  for _ in range(frames):
    icbm.run(cs, cc, lp, True)


class TestAlphaLongIcbm:

  def test_stock_gate_unchanged_without_alpha_mode(self):
    # pcmCruiseSpeed=True and alpha-long mode off: ICBM stays fully dormant (today's behavior)
    icbm = make_icbm(alpha_enabled=False)
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), PRE_ACTIVE_FRAMES)
    assert icbm.state == State.inactive
    assert icbm.cruise_button == SendButtonState.none

  def test_not_available_platform_stays_dormant(self):
    icbm = make_icbm(icbm_available=False)
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), PRE_ACTIVE_FRAMES)
    assert icbm.state == State.inactive

  def test_walks_cluster_up_to_limit_plus_margin(self):
    # SLA confirmed a 100 km/h limit; cluster at 80 must walk up toward 100 + 5
    icbm = make_icbm()
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), WALK_FRAMES)
    assert icbm.v_target == 105
    assert icbm.state == State.increasing
    assert icbm.cruise_button == SendButtonState.increase

  def test_walks_cluster_down_to_limit_plus_margin(self):
    # limit dropped to 50; cluster at 80 must walk down toward 55
    icbm = make_icbm()
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 50), WALK_FRAMES)
    assert icbm.v_target == 55
    assert icbm.state == State.decreasing
    assert icbm.cruise_button == SendButtonState.decrease

  def test_holds_at_limit_plus_margin(self):
    icbm = make_icbm()
    drive(icbm, make_cs(105), make_cc(), make_lp(True, 100), WALK_FRAMES)
    assert icbm.state == State.holding
    assert icbm.cruise_button == SendButtonState.none

  def test_holds_when_sla_inactive(self):
    # no active SLA target: leave the cluster wherever the driver put it
    icbm = make_icbm()
    drive(icbm, make_cs(80), make_cc(), make_lp(False), PRE_ACTIVE_FRAMES)
    assert icbm.v_target == icbm.v_cruise_cluster
    assert icbm.state == State.holding
    assert icbm.cruise_button == SendButtonState.none

  def test_configurable_offset(self):
    icbm = make_icbm()
    icbm.alpha_long_offset = 10
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), WALK_FRAMES)
    assert icbm.v_target == 110

  def test_engagement_speed_cap_not_applied_in_alpha_long_mode(self):
    # in stock mode ICBM refuses to walk more than +5 above the engagement speed; the alpha-long
    # target is the driver-confirmed limit + margin, so the cap must not block the walk-up
    icbm = make_icbm()
    icbm.initial_cruise_speed_kph = 50
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), WALK_FRAMES)
    assert icbm.state == State.increasing

  def test_driver_button_press_interrupts(self):
    icbm = make_icbm()
    cs = make_cs(80)
    drive(icbm, cs, make_cc(), make_lp(True, 100), WALK_FRAMES)
    assert icbm.state == State.increasing

    cs_pressed = make_cs(80)
    events = cs_pressed.init('buttonEvents', 1)
    events[0].type = ButtonType.decelCruise
    events[0].pressed = True
    drive(icbm, cs_pressed, make_cc(), make_lp(True, 100), 1)
    assert icbm.state == State.inactive
    assert icbm.cruise_button == SendButtonState.none

  def test_disengaged_goes_inactive(self):
    icbm = make_icbm()
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), WALK_FRAMES)
    assert icbm.state == State.increasing
    drive(icbm, make_cs(80), make_cc(enabled=False), make_lp(True, 100), 1)
    assert icbm.state == State.inactive

  def test_target_dwell_damps_flapping(self):
    # a changed target is only adopted once stable for ALPHA_LONG_TARGET_DWELL, so resolver
    # source flaps do not stream physical button presses at the PCM
    icbm = make_icbm()
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), WALK_FRAMES)
    assert icbm.v_target == 105

    # alternate targets faster than the dwell: the adopted target must not budge
    for _ in range(10):
      drive(icbm, make_cs(80), make_cc(), make_lp(True, 90), DWELL_FRAMES // 4)
      drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), DWELL_FRAMES // 4)
    assert icbm.v_target == 105

    # a stable new target is adopted after the dwell
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 90), DWELL_FRAMES)
    assert icbm.v_target == 95

  def test_before_dwell_holds_cluster(self):
    icbm = make_icbm()
    drive(icbm, make_cs(80), make_cc(), make_lp(True, 100), PRE_ACTIVE_FRAMES)
    # dwell not yet satisfied: no walking, target pinned to the cluster
    assert icbm.v_target == icbm.v_cruise_cluster
    assert icbm.state == State.holding

  def test_stock_mode_max_target_boundary_never_walks(self):
    # stock-mode regression guard: a target of exactly 145 km/h must never emit button presses
    # (the state legitimately oscillates preActive<->holding, but stays out of increasing)
    icbm = make_icbm(op_long=False, pcm_cruise_speed=False, alpha_enabled=False)
    cs = make_cs(140)
    cs.vEgo = 145 * CV.KPH_TO_MS  # engagement capture makes initial_cruise_speed_kph = 145
    for _ in range(PRE_ACTIVE_FRAMES):
      icbm.run(cs, make_cc(), make_lp(True, 150), True)
      assert icbm.state != State.increasing
      assert icbm.cruise_button == SendButtonState.none
    assert icbm.v_target == 145
