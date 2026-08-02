"""BluePilot: tests for the per-frame model-decel gate."""

from openpilot.sunnypilot.selfdrive.controls.lib.model_decel_gate import (
  ModelDecelGate, ENGAGE_ACCEL, RELEASE_TIME, RELEASE_TIME_LOW_SPEED)

DT = 0.05
V_CRUISE = 15.0  # m/s


def run(gate, frames, accel, should_stop=False, end_v=None, v_ego=V_CRUISE):
  end_v = end_v if end_v is not None else v_ego
  active = gate.active
  for _ in range(frames):
    active = gate.update(accel, should_stop, end_v, v_ego)
  return active


class TestModelDecelGate:

  def test_cruise_plateau_never_engages(self):
    # the bug being fixed: model holds ~0.0 accel below target with a flat plan — MPC must drive
    gate = ModelDecelGate(DT)
    assert not run(gate, 200, accel=0.02, end_v=V_CRUISE - 0.4)

  def test_engages_immediately_on_decel_intent(self):
    gate = ModelDecelGate(DT)
    assert gate.update(ENGAGE_ACCEL - 0.05, False, V_CRUISE, V_CRUISE)

  def test_engages_on_collapsed_plan_end_speed(self):
    # red light far ahead: plan end speed collapses before instantaneous accel dips
    gate = ModelDecelGate(DT)
    assert gate.update(0.0, False, V_CRUISE - 3.0, V_CRUISE)

  def test_ordinary_plan_sag_does_not_engage(self):
    # log-measured: plan end sags 0.6-2.3 m/s below v_ego in ordinary cruise
    gate = ModelDecelGate(DT)
    assert not run(gate, 100, accel=0.0, end_v=V_CRUISE - 1.8)

  def test_engages_on_should_stop(self):
    gate = ModelDecelGate(DT)
    assert gate.update(0.0, True, 0.0, 0.2)

  def test_releases_after_sustained_clear(self):
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    assert gate.active
    frames = int(RELEASE_TIME / DT)
    assert run(gate, frames - 1, accel=0.1) is True
    assert run(gate, 1, accel=0.1) is False

  def test_midstop_accel_bounce_cannot_release(self):
    # smoothing passes brief positive model accel mid-stop; the collapsed plan end speed
    # must keep the gate held — releasing would surge toward the red light
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.2, end_v=1.0)
    assert run(gate, 100, accel=0.15, end_v=1.0) is True

  def test_hysteresis_band_holds_gate(self):
    # accel recovered into the band between release (-0.05) and engage (-0.2): hold
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    assert run(gate, 200, accel=-0.1) is True

  def test_clear_resets_release_counter(self):
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    run(gate, int(RELEASE_TIME / DT) - 2, accel=0.1)          # almost released
    run(gate, 1, accel=-0.5, end_v=V_CRUISE)                  # intent returns
    assert run(gate, int(RELEASE_TIME / DT) - 2, accel=0.1) is True  # counter restarted

  def test_low_speed_release_is_slower(self):
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-0.5, end_v=0.5, v_ego=2.0)
    frames_normal = int(RELEASE_TIME / DT)
    assert run(gate, frames_normal + 2, accel=0.3, end_v=2.0, v_ego=2.0) is True
    frames_rest = int(RELEASE_TIME_LOW_SPEED / DT) - frames_normal - 2
    assert run(gate, frames_rest + 1, accel=0.3, end_v=2.0, v_ego=2.0) is False

  def test_reengages_instantly_after_release(self):
    # green flips back to red: one frame of intent re-engages
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    run(gate, int(RELEASE_TIME / DT) + 1, accel=0.2)
    assert not gate.active
    assert gate.update(-0.5, False, V_CRUISE, V_CRUISE) is True

  def test_adjustable_engage_threshold(self):
    # sensitive threshold: any slight decel engages (soft-braking models, curves, crests)
    gate = ModelDecelGate(DT)
    gate.set_engage_accel(-0.0)
    assert gate.update(-0.05, False, V_CRUISE, V_CRUISE) is True

    # hard threshold: gentle decel no longer engages
    gate = ModelDecelGate(DT)
    gate.set_engage_accel(-2.0)
    assert not run(gate, 20, accel=-1.0)
    assert gate.update(-2.5, False, V_CRUISE, V_CRUISE) is True
    # release band follows the engage threshold
    assert run(gate, 200, accel=-1.9) is True   # inside hysteresis band: held
    assert run(gate, int(RELEASE_TIME / DT) + 1, accel=-1.5) is False  # above release: clears

  def test_engage_threshold_clamped(self):
    gate = ModelDecelGate(DT)
    gate.set_engage_accel(-99.0)
    assert gate.engage_accel == -5.0
    gate.set_engage_accel(1.0)
    assert gate.engage_accel == 0.0
