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

  # ---- continuous weight (the envelope) ----------------------------------------------
  # The weight is a shaped envelope of the binary machine: rise over RISE_TIME on engage
  # (shouldStop instant), HOLD while latched in the hysteresis band, decay across the
  # release window on all-clear frames only, zero below the engage setpoint. An adversarial
  # review of the band-tracking alternative measured why each of these is load-bearing.

  def test_weight_zero_in_cruise_plateau(self):
    gate = ModelDecelGate(DT)
    run(gate, 50, accel=0.02, end_v=V_CRUISE - 0.4)
    assert gate.weight == 0.0

  def test_sub_engage_intent_never_participates(self):
    # THE critical from review: a model resting just above the engage threshold forever
    # (here -0.19 vs engage -0.2, sag inside the ordinary band) must contribute NOTHING -
    # a band-proportional weight phantom-braked a simulated cruise to a standstill in 106 s
    gate = ModelDecelGate(DT)
    run(gate, 2400, accel=-0.19, end_v=V_CRUISE - 1.5)
    assert gate.weight == 0.0
    assert not gate.active

  def test_weight_full_past_engage_setpoint(self):
    gate = ModelDecelGate(DT)
    run(gate, int(RISE_TIME / DT) + 1, accel=ENGAGE_ACCEL - 0.01)
    assert gate.weight == 1.0

  def test_weight_rises_fast_but_rate_limited(self):
    # deep intent: full authority within RISE_TIME, but never a single-frame snap
    gate = ModelDecelGate(DT)
    run(gate, 50, accel=0.1)
    assert gate.weight == 0.0
    gate.update(-1.0, False, 2.0, V_CRUISE)
    assert abs(gate.weight - DT / RISE_TIME) < 1e-9
    run(gate, int(RISE_TIME / DT), accel=-1.0, end_v=2.0)
    assert gate.weight == 1.0

  def test_should_stop_bypasses_the_rise_limit(self):
    gate = ModelDecelGate(DT)
    run(gate, 50, accel=0.1)
    gate.update(0.0, True, 0.0, 0.5)
    assert gate.weight == 1.0

  def test_collapsed_plan_end_alone_takes_full_weight(self):
    gate = ModelDecelGate(DT)
    run(gate, int(RISE_TIME / DT) + 1, accel=0.15, end_v=V_CRUISE - 3.0)
    assert gate.weight == 1.0

  def test_inband_easing_holds_full_weight(self):
    # THE major from review: engaged hard, model eases to -0.12 (still braking, inside the
    # band) with ordinary plan sag. A band-proportional weight blended 53% of an
    # accelerating cruise plan here - net acceleration against live decel intent. The
    # latch must hold everything.
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    run(gate, 400, accel=-0.12, end_v=V_CRUISE - 1.4)
    assert gate.active
    assert gate.weight == 1.0

  def test_deep_threshold_inband_easing_holds(self):
    # user-deepened engage -2.0: easing from -2.1 to -1.9 is still hard braking
    gate = ModelDecelGate(DT)
    gate.set_engage_accel(-2.0)
    run(gate, 10, accel=-2.1)
    run(gate, 400, accel=-1.9)
    assert gate.active and gate.weight == 1.0

  def test_midstop_bounce_holds_full_weight(self):
    # collapsed plan-end pins the envelope through brief positive accel excursions
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.2, end_v=1.0)
    run(gate, 100, accel=0.15, end_v=1.0)
    assert gate.weight == 1.0

  def test_low_speed_waver_cannot_surge(self):
    # the review's low-stop scenario: v 2.4 m/s, model excursions to +0.1 with plan-end
    # ~1.4 m/s. all_clear needs end_v above v - margin/2 = 1.4, so the latch holds and the
    # weight must not decay toward the accelerating cruise plan mid-stop.
    gate = ModelDecelGate(DT)
    run(gate, 20, accel=-1.0, end_v=0.5, v_ego=2.4)
    run(gate, 40, accel=0.1, end_v=1.4, v_ego=2.4)
    assert gate.active
    assert gate.weight == 1.0

  def test_weight_falls_over_release_window_not_instantly(self):
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    assert gate.weight == 1.0
    gate.update(0.2, False, V_CRUISE, V_CRUISE)
    assert 1.0 - gate.weight <= DT / RELEASE_TIME + 1e-9
    run(gate, int(RELEASE_TIME / DT) - 1, accel=0.2)
    assert gate.weight == 0.0

  def test_weight_decay_matches_binary_release_timing(self):
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

  def test_band_frames_pause_decay_but_never_refill(self):
    # flickering intent: clear frames decay, band frames hold - cumulative clear time
    # releases, and the weight never climbs without a genuine engage condition
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    run(gate, 4, accel=0.2)               # clear: decays 4 steps
    w = gate.weight
    run(gate, 40, accel=-0.1)             # band: hold exactly
    assert gate.weight == w
    run(gate, 2, accel=0.2)               # clear again: resumes
    assert gate.weight < w

  def test_weight_interrupted_decay_recovers_on_engage(self):
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    run(gate, 5, accel=0.2)
    assert gate.weight < 1.0
    run(gate, int(RISE_TIME / DT) + 1, accel=-0.5)
    assert gate.weight == 1.0

  # ---- pre-threshold ramp (the acceleration cap) --------------------------------------

  def test_pre_cap_none_above_the_band(self):
    gate = ModelDecelGate(DT)
    assert gate.pre_engage_cap(0.0, 0.8) is None
    assert gate.pre_engage_cap(ENGAGE_ACCEL + 0.06, 0.8) is None

  def test_pre_cap_tapers_to_the_model_at_the_setpoint(self):
    # halfway into the band the cap is halfway between unrestricted and the model;
    # at the setpoint it would meet the model (floored at 0 - braking is the gate's job)
    gate = ModelDecelGate(DT)
    mid = ENGAGE_ACCEL + 0.025
    cap = gate.pre_engage_cap(mid, 0.8)
    assert cap is not None and abs(cap - (0.5 * mid + 0.5 * 0.8)) < 1e-9

  def test_pre_cap_never_commands_braking_by_default(self):
    # the default floor: a model resting deep in the band must not creep the car down
    # (unfloored sim worst case: 10.4 m/s -> 0 in under two minutes at band 0.10)
    gate = ModelDecelGate(DT)
    for e2e in (-0.16, -0.18, -0.19, -0.199):
      cap = gate.pre_engage_cap(e2e, 0.8)
      assert cap is not None and cap >= 0.0, f"cap {cap} at e2e {e2e} commands braking pre-engagement"

  def test_pre_cap_unfloored_follows_the_blend_below_zero(self):
    # on-road test option: with the floor off the cap IS the taper blend, so the model's
    # gentle proto-braking acts before the gate engages
    gate = ModelDecelGate(DT)
    gate.set_pre_floor(False)
    for e2e in (-0.18, -0.19, -0.199):
      f = min(1.0, ((gate.engage_accel + gate.pre_band) - e2e) / gate.pre_band)
      expect = f * e2e + (1.0 - f) * 0.8
      cap = gate.pre_engage_cap(e2e, 0.8)
      assert cap is not None and abs(cap - expect) < 1e-9
    assert gate.pre_engage_cap(-0.199, 0.8) < 0.0  # deep in the band it may brake gently
    # outside the band the ramp still does not exist, floored or not
    assert gate.pre_engage_cap(0.0, 0.8) is None
    gate.set_pre_floor(True)
    assert gate.pre_engage_cap(-0.199, 0.8) >= 0.0  # toggling back restores the floor

  def test_pre_cap_disabled_at_zero_band(self):
    gate = ModelDecelGate(DT)
    gate.set_pre_band(0.0)
    assert gate.pre_engage_cap(ENGAGE_ACCEL - 0.01, 0.8) is None

  def test_pre_cap_follows_adjusted_engage_threshold(self):
    gate = ModelDecelGate(DT)
    gate.set_engage_accel(-2.0)
    assert gate.pre_engage_cap(-1.9, 0.8) is None      # above -2.0 + 0.05
    assert gate.pre_engage_cap(-1.97, 0.8) is not None  # inside the band

  def test_reset_clears_everything(self):
    # leaving e2e mode / toggling the feature: stale weight must not resurrect on re-entry
    gate = ModelDecelGate(DT)
    run(gate, 10, accel=-1.0, end_v=2.0)
    assert gate.active and gate.weight == 1.0
    gate.reset()
    assert not gate.active and gate.weight == 0.0 and gate._release_counter == 0

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
