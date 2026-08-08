"""BluePilot: tests for the per-frame model-decel gate."""

from openpilot.sunnypilot.selfdrive.controls.lib.model_decel_gate import (
  ModelDecelGate, ENGAGE_ACCEL, RELEASE_TIME, RELEASE_TIME_LOW_SPEED, RISE_TIME)

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

  # ---- continuous weight (the ramp) -------------------------------------------------
  # The blend weight rises instantly with intent and falls rate-limited across the release
  # window, ramping the e2e branch in/out across the SAME hysteresis bands the binary
  # thresholds define. Log-measured motivation: the binary form flipped 10.9 times/min on a
  # real drive with +0.7 m/s^2 half-second swings at release.

  def test_weight_zero_in_cruise_plateau(self):
    gate = ModelDecelGate(DT)
    run(gate, 50, accel=0.02, end_v=V_CRUISE - 0.4)
    assert gate.weight == 0.0

  def test_weight_full_at_engage_setpoint(self):
    gate = ModelDecelGate(DT)
    run(gate, int(RISE_TIME / DT) + 1, accel=ENGAGE_ACCEL)
    assert gate.weight == 1.0

  def test_weight_ramps_across_accel_band(self):
    # halfway between release (-0.05) and engage (-0.2) => half participation
    gate = ModelDecelGate(DT)
    mid = (gate.release_accel + gate.engage_accel) / 2.0
    run(gate, int(RISE_TIME / DT) + 1, accel=mid)
    assert abs(gate.weight - 0.5) < 1e-9
    # and it pre-ramps BEFORE binary engage: gate not active, weight already partial
    assert not gate.active

  def test_weight_ramps_across_end_v_band(self):
    # halfway between release margin (1.0) and engage margin (2.0) => half participation
    gate = ModelDecelGate(DT)
    run(gate, int(RISE_TIME / DT) + 1, accel=0.0, end_v=V_CRUISE - 1.5)
    assert abs(gate.weight - 0.5) < 1e-9

  def test_weight_signals_combine_by_max(self):
    # a collapsed plan-end speed holds full weight even while the accel signal reads clear
    gate = ModelDecelGate(DT)
    run(gate, int(RISE_TIME / DT) + 1, accel=0.15, end_v=V_CRUISE - 3.0)
    assert gate.weight == 1.0

  def test_weight_rises_fast_but_rate_limited(self):
    # deep intent: full authority within RISE_TIME, but never a single-frame snap — a
    # one-frame weight step against a ~0.7 m/s^2 e2e-to-MPC spread is the in-band jerk this
    # ramp exists to remove
    gate = ModelDecelGate(DT)
    run(gate, 50, accel=0.1)
    assert gate.weight == 0.0
    gate.update(-1.0, False, 2.0, V_CRUISE)
    assert abs(gate.weight - DT / RISE_TIME) < 1e-9
    run(gate, int(RISE_TIME / DT), accel=-1.0, end_v=2.0)
    assert gate.weight == 1.0

  def test_should_stop_bypasses_the_rise_limit(self):
    # the stop-hold latch takes everything immediately
    gate = ModelDecelGate(DT)
    run(gate, 50, accel=0.1)
    gate.update(0.0, True, 0.0, 0.5)
    assert gate.weight == 1.0

  def test_weight_falls_over_release_window_not_instantly(self):
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    assert gate.weight == 1.0
    # one all-clear frame: barely moves
    gate.update(0.2, False, V_CRUISE, V_CRUISE)
    assert 1.0 - gate.weight <= DT / RELEASE_TIME + 1e-9
    # decays to zero across exactly the release window
    run(gate, int(RELEASE_TIME / DT) - 1, accel=0.2)
    assert gate.weight == 0.0

  def test_weight_decay_matches_binary_release_timing(self):
    # by the frame the binary machine deactivates, the blend has already reached the MPC:
    # deactivation is a no-op in the published accel
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    frames = int(RELEASE_TIME / DT)
    still_active = run(gate, frames - 1, accel=0.1)
    assert still_active and gate.weight <= DT / RELEASE_TIME + 1e-9
    assert run(gate, 1, accel=0.1) is False
    assert gate.weight == 0.0

  def test_weight_decay_slower_at_low_speed(self):
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-0.5, end_v=0.5, v_ego=2.0)
    gate.update(0.3, False, 2.0, 2.0)
    assert 1.0 - gate.weight <= DT / RELEASE_TIME_LOW_SPEED + 1e-9

  def test_midstop_bounce_holds_full_weight(self):
    # same guarantee as the binary hold: collapsed plan-end speed pins weight at 1 through
    # brief positive excursions of the smoothed model accel
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.2, end_v=1.0)
    run(gate, 100, accel=0.15, end_v=1.0)
    assert gate.weight == 1.0

  def test_boundary_hover_stays_smooth_not_bouncing(self):
    # THE bug: model accel oscillating around the engage setpoint. Binary active flaps with
    # it; the weight must stay continuously high with tiny per-frame movement.
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-0.5, end_v=2.0)
    weights = []
    for i in range(200):
      a = -0.2 + 0.1 * (1 if (i // 10) % 2 else -1)   # square wave -0.1 .. -0.3 around engage
      gate.update(a, False, V_CRUISE, V_CRUISE)
      weights.append(gate.weight)
    steps = [abs(b - a) for a, b in zip(weights, weights[1:], strict=False)]
    # the binary gate rode this hover as alternating full-release/full-engage output swings;
    # the weight instead stays continuously engaged between the band value and 1, moving no
    # faster than the rise rate per frame in either direction
    assert min(weights) > 0.3, "weight collapsed during a hover around the setpoint"
    assert max(steps) <= DT / RISE_TIME + 1e-9
    assert max(weights) == 1.0

  def test_weight_interrupted_decay_recovers_instantly(self):
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    run(gate, 5, accel=0.2)          # decaying
    assert gate.weight < 1.0
    run(gate, int(RISE_TIME / DT) + 1, accel=-0.5)
    assert gate.weight == 1.0        # intent returned: full authority within the rise window

  def test_weight_respects_adjusted_thresholds(self):
    gate = ModelDecelGate(DT)
    gate.set_engage_accel(-2.0)
    gate.update(-1.0, False, V_CRUISE, V_CRUISE)   # far above the band (release -1.85)
    assert gate.weight == 0.0
    mid = (gate.release_accel + gate.engage_accel) / 2.0
    run(gate, int(RISE_TIME / DT) + 1, accel=mid)
    assert abs(gate.weight - 0.5) < 1e-9

  def test_weight_end_v_disabled_margin_zero(self):
    # end-v feature disabled (margin 0): sag must contribute nothing, accel band still works
    gate = ModelDecelGate(DT)
    gate.set_end_v_margin(0.0)
    gate.update(0.1, False, 0.0, V_CRUISE)
    assert gate.weight == 0.0

  def test_adjustable_end_v_margin(self):
    # wide margin: ordinary plan sag must not engage; deep collapse must
    gate = ModelDecelGate(DT)
    gate.set_end_v_margin(-6.0)
    assert not run(gate, 20, accel=0.0, end_v=V_CRUISE - 5.0)
    assert gate.update(0.0, False, V_CRUISE - 7.0, V_CRUISE) is True

    # sensitive margin: small sag engages; release re-arms at half the margin
    gate = ModelDecelGate(DT)
    gate.set_end_v_margin(-0.5)
    assert gate.update(0.0, False, V_CRUISE - 0.8, V_CRUISE) is True
    assert run(gate, 200, accel=0.0, end_v=V_CRUISE - 0.4) is True  # inside hysteresis band
    assert run(gate, int(RELEASE_TIME / DT) + 1, accel=0.0, end_v=V_CRUISE - 0.1) is False

  def test_end_v_margin_clamped(self):
    gate = ModelDecelGate(DT)
    gate.set_end_v_margin(-99.0)
    assert gate.end_v_margin == 10.0
    gate.set_end_v_margin(2.0)  # positive input clamps to 0 (max sensitivity)
    assert gate.end_v_margin == 0.0
