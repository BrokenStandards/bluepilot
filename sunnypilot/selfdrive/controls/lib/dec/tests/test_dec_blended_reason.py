"""BluePilot: tests for DEC's published blended reason + urgency."""

from cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController

BlendedReason = custom.LongitudinalPlanSP.DynamicExperimentalControl.BlendedReason


class MockParams:
  def get_bool(self, name):
    return True


class MockMpc:
  crash_cnt = 0


class MockCP:
  radarUnavailable = False


def make_dec() -> DynamicExperimentalController:
  return DynamicExperimentalController(MockCP(), MockMpc(), params=MockParams())


def run_mode(dec, frames=30):
  for _ in range(frames):
    dec._radar_mode()
    dec._mode_manager.update()


class TestDecBlendedReason:

  def test_no_reason_while_acc(self):
    dec = make_dec()
    run_mode(dec)
    assert dec.mode() == 'acc'
    assert dec.blended_reason() == BlendedReason.none

  def test_fcw_reason(self):
    dec = make_dec()
    dec._has_mpc_fcw = True
    run_mode(dec, frames=2)
    assert dec.mode() == 'blended'
    assert dec.blended_reason() == BlendedReason.fcw

  def test_slow_down_reason_and_urgency(self):
    dec = make_dec()
    dec._has_slow_down = True
    dec._urgency = 0.85  # emergency path: immediate blended
    run_mode(dec, frames=2)
    assert dec.mode() == 'blended'
    assert dec.blended_reason() == BlendedReason.slowDown
    assert dec.urgency() == 0.85

  def test_standstill_reason(self):
    dec = make_dec()
    dec._standstill_count = 10
    run_mode(dec, frames=30)
    assert dec.mode() == 'blended'
    assert dec.blended_reason() == BlendedReason.standstill

  def test_reason_clears_when_back_to_acc(self):
    dec = make_dec()
    dec._has_slow_down = True
    dec._urgency = 0.9
    run_mode(dec, frames=2)
    assert dec.blended_reason() == BlendedReason.slowDown

    # hazard passes: decisions request acc again; once the manager flips, reason reads none
    dec._has_slow_down = False
    dec._urgency = 0.0
    run_mode(dec, frames=60)
    assert dec.mode() == 'acc'
    assert dec.blended_reason() == BlendedReason.none

  def test_reason_latched_through_hysteresis(self):
    # while the manager still reports blended, the latched reason must remain visible
    dec = make_dec()
    dec._has_slow_down = True
    dec._urgency = 0.9
    run_mode(dec, frames=2)
    dec._has_slow_down = False
    dec._urgency = 0.0
    # a few frames of acc requests: hysteresis keeps blended briefly
    for _ in range(3):
      dec._radar_mode()
      dec._mode_manager.update()
      if dec.mode() == 'blended':
        assert dec.blended_reason() == BlendedReason.slowDown

  def test_lead_prefers_acc_over_slow_down_reasonless(self):
    # radar mode: a real lead forces acc; reason must read none even with stale latch
    dec = make_dec()
    dec._has_slow_down = True
    dec._urgency = 0.9
    run_mode(dec, frames=2)
    dec._has_slow_down = False
    dec._has_lead_filtered = True
    run_mode(dec, frames=60)
    assert dec.mode() == 'acc'
    assert dec.blended_reason() == BlendedReason.none
