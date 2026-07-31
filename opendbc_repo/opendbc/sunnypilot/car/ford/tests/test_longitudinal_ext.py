"""
Tests for the BluePilot Ford longitudinal follow control extension (longitudinal_ext.py).

Pins the legacy coasting mode's behavior (including the quirks extended mode fixes) and
verifies extended mode restores the stock Ford coast band: decel requests shallower than
the extended brake threshold ride the propulsion channel (AccPrpl_A_Rq) with the friction
brake bits off.
"""

import unittest
from dataclasses import dataclass
from types import SimpleNamespace

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.longitudinal_ext import (
  LongitudinalExt,
  COASTING_MODE_LEGACY,
  COASTING_MODE_EXTENDED,
  PARAM_REFRESH_FRAMES,
)

INACTIVE_GAS = CarControllerParams.INACTIVE_GAS  # -5.0
MIN_GAS = CarControllerParams.MIN_GAS            # -0.5

HIGHWAY_MPH = 60.0  # above the 50 mph BP-long engage threshold
V_EGO_HIGHWAY = 27.0  # m/s, ~60 mph


@dataclass
class _CSOut:
  vEgo: float = V_EGO_HIGHWAY
  gasPressed: bool = False
  brakePressed: bool = False


class _CS:
  def __init__(self, **kwargs):
    self.out = _CSOut(**kwargs)


@dataclass
class _CC:
  longActive: bool = True


class _FakeSubMaster:
  """Serves radarState with an optional leadOne; mirrors what LongitudinalExt reads."""

  def __init__(self, lead=None, radar_valid=True):
    self.valid = {'radarState': radar_valid}
    self._radar_state = SimpleNamespace(leadOne=lead)

  def __getitem__(self, key):
    if key == 'radarState':
      return self._radar_state
    raise KeyError(key)


def _lead(d_rel=54.0, v_rel=0.0, v_lead=V_EGO_HIGHWAY, status=1):
  return SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel, vLead=v_lead)


def _make_ext(mode=COASTING_MODE_LEGACY, lead=None, radar_valid=True):
  ext = LongitudinalExt(None, None)
  ext.coasting_mode = mode
  ext.sm = _FakeSubMaster(lead=lead, radar_valid=radar_valid)
  return ext


def _update(ext, op_accel, op_gas=None, pitch=0.0, long_active=True, v_ego_mph=HIGHWAY_MPH,
            stopping=False, gas_pressed=False, brake_pressed=False):
  if op_gas is None:
    op_gas = op_accel if op_accel >= MIN_GAS else INACTIVE_GAS
  cc = _CC(longActive=long_active)
  cs = _CS(gasPressed=gas_pressed, brakePressed=brake_pressed)
  return ext.update(cc, cs, op_accel, op_gas, pitch, v_ego_mph, stopping, 135.0)


def _prime_speed_allow(ext):
  """One benign update above 50 mph latches bpSpeedAllow."""
  _update(ext, 0.0, 0.0)
  assert ext.bpSpeedAllow


class TestLegacyModeUnchanged(unittest.TestCase):
  """Legacy mode must keep the original behavior exactly, quirks included."""

  def test_legacy_constants(self):
    ext = _make_ext()
    self.assertEqual(ext.brake_actuate_target, -0.14)
    self.assertEqual(ext.brake_actuate_release, -0.06)
    self.assertEqual(ext.precharge_actuate_target, -0.12)
    self.assertEqual(ext.precharge_actuate_release, -0.06)
    self.assertEqual(ext.following_accel_ROC, 0.002)
    self.assertEqual(ext.coasting_mode, COASTING_MODE_LEGACY)

  def test_legacy_op_path_brake_hysteresis(self):
    # disable_BP_long_UI=True exercises the pure op path
    ext = _make_ext()
    ext.disable_BP_long_UI = True
    # shallow decel already asserts the friction brakes
    res = _update(ext, -0.2)
    self.assertTrue(res.brake_actuate)
    self.assertTrue(res.precharge_actuate)  # legacy op path ties precharge to brake
    self.assertEqual(res.gas, INACTIVE_GAS)  # legacy mutual exclusion
    # inside the hysteresis band the bit holds
    res = _update(ext, -0.10)
    self.assertTrue(res.brake_actuate)
    # above release it lets go
    res = _update(ext, -0.02)
    self.assertFalse(res.brake_actuate)

  def test_legacy_pacing_raises_inactive_gas_to_zero(self):
    # The legacy floor of 0.0 turns a "no gas" request into an active 0.0 m/s^2 hold-speed
    # command. Pinned here as legacy behavior; extended mode fixes it.
    ext = _make_ext(lead=_lead(v_rel=0.0))
    _prime_speed_allow(ext)
    res = _update(ext, -0.6, op_gas=INACTIVE_GAS)
    self.assertTrue(res.bp_long_used)
    self.assertEqual(res.gas, 0.0)

  def test_legacy_no_lead_pins_accel_to_zero(self):
    ext = _make_ext(lead=None)
    _prime_speed_allow(ext)
    res = _update(ext, -1.0, op_gas=INACTIVE_GAS)
    self.assertTrue(res.bp_long_used)
    self.assertEqual(res.accel, 0.0)  # pinned: holds speed where the planner wanted decel

  def test_legacy_roc_limits_decel_onset(self):
    # bp_accel_last seeds at 0.0; a -1.0 request only moves 0.002 per scan (0.1 m/s^3)
    ext = _make_ext(lead=_lead(d_rel=54.0, v_rel=0.0))
    _prime_speed_allow(ext)
    ext.bp_accel_last = 0.0
    res = _update(ext, -1.0, op_gas=INACTIVE_GAS)
    self.assertAlmostEqual(res.accel, -0.002, places=6)

  def test_legacy_low_speed_falls_back_to_op(self):
    ext = _make_ext(lead=None)
    res = _update(ext, -0.5, op_gas=INACTIVE_GAS, v_ego_mph=30.0)
    self.assertFalse(res.bp_long_used)
    self.assertEqual(res.accel, -0.5)

  def test_legacy_emergency_ttc_bypasses_roc(self):
    # cut-in: d_rel=20, v_rel=-5 -> ttc=4s; the following ROC must not hold off braking
    ext = _make_ext(mode=COASTING_MODE_LEGACY, lead=_lead(d_rel=20.0, v_rel=-5.0, v_lead=22.0))
    _prime_speed_allow(ext)
    ext.bp_accel_last = 0.0
    res = _update(ext, -3.5, op_gas=INACTIVE_GAS)
    self.assertTrue(res.bp_long_used)
    self.assertEqual(res.accel, -3.5)

  def test_pedal_press_falls_back_to_op(self):
    for kwargs in ({'gas_pressed': True}, {'brake_pressed': True}):
      ext = _make_ext(lead=None)
      _prime_speed_allow(ext)
      res = _update(ext, -1.0, op_gas=INACTIVE_GAS, **kwargs)
      self.assertFalse(res.bp_long_used)
      self.assertEqual(res.accel, -1.0)  # op passthrough, not the legacy 0.0 no-lead pin

  def test_status_zero_lead_treated_as_no_lead(self):
    # Real cereal radarState always populates leadOne; status=0 is the live "no lead" shape
    ext = _make_ext(lead=_lead(status=0, d_rel=0.0, v_rel=0.0, v_lead=0.0))
    _prime_speed_allow(ext)
    res = _update(ext, -1.0, op_gas=INACTIVE_GAS)
    self.assertTrue(res.bp_long_used)
    self.assertEqual(res.accel, 0.0)  # legacy no-lead pin, same as leadOne=None

  def test_radar_invalid_treated_as_no_lead(self):
    ext = _make_ext(lead=_lead(), radar_valid=False)
    _prime_speed_allow(ext)
    res = _update(ext, -1.0, op_gas=INACTIVE_GAS)
    self.assertTrue(res.bp_long_used)
    self.assertEqual(res.accel, 0.0)  # legacy no-lead pin

  def test_speed_deadband_hysteresis(self):
    # primed above 50 mph, then dropping into the 45-50 deadband holds BP long
    ext = _make_ext(lead=None)
    _prime_speed_allow(ext)
    res = _update(ext, 0.0, 0.0, v_ego_mph=47.0)
    self.assertTrue(res.bp_long_used)
    # a fresh instance starting inside the deadband never engages
    ext2 = _make_ext(lead=None)
    res2 = _update(ext2, 0.0, 0.0, v_ego_mph=47.0)
    self.assertFalse(res2.bp_long_used)
    # dropping below 45 mph unlatches
    res3 = _update(ext, 0.0, 0.0, v_ego_mph=44.0)
    self.assertFalse(res3.bp_long_used)

  def test_gaining_far_gas_passthrough(self):
    # gaining but beyond 1.5s gap: positive gas must NOT be capped (both modes)
    for mode in (COASTING_MODE_LEGACY, COASTING_MODE_EXTENDED):
      ext = _make_ext(mode=mode, lead=_lead(d_rel=108.0, v_rel=-0.2, v_lead=26.8))
      _prime_speed_allow(ext)
      res = _update(ext, 0.5, op_gas=0.5)
      self.assertTrue(res.bp_long_used)
      self.assertEqual(res.gas, 0.5)

  def test_slow_lead_disengages_bp_long(self):
    # approaching a lead below 40 mph at highway speed hands control back to stock
    for mode in (COASTING_MODE_LEGACY, COASTING_MODE_EXTENDED):
      ext = _make_ext(mode=mode, lead=_lead(d_rel=110.0, v_rel=-12.0, v_lead=15.0))  # 33.5 mph, ttc ~9.2s
      _prime_speed_allow(ext)
      ext.bp_accel_last = 0.0
      res = _update(ext, -1.0, op_gas=INACTIVE_GAS)
      self.assertFalse(res.bp_long_used)
      self.assertEqual(res.accel, -1.0)  # no ROC throttling of decel onset


class TestExtendedCoastBand(unittest.TestCase):
  """Extended mode: friction brakes only beyond the propulsion channel's floor."""

  def test_extended_shallow_decel_coasts(self):
    # -0.3 m/s^2 is inside Ford's factory coast band: no brake bits, negative
    # propulsion passes through on the gas channel.
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    res = _update(ext, -0.3, op_gas=-0.3)
    self.assertFalse(res.brake_actuate)
    self.assertFalse(res.precharge_actuate)
    self.assertEqual(res.gas, -0.3)
    self.assertEqual(res.accel, -0.3)

  def test_extended_deep_decel_brakes(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    res = _update(ext, -0.6)
    self.assertTrue(res.brake_actuate)
    self.assertTrue(res.precharge_actuate)

  def test_extended_brake_hysteresis_band(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    _update(ext, -0.6)  # engage
    res = _update(ext, -0.3)  # inside band: hold
    self.assertTrue(res.brake_actuate)
    res = _update(ext, -0.18)  # above -0.20 release
    self.assertFalse(res.brake_actuate)

  def test_extended_precharge_leads_brake(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    res = _update(ext, -0.40)  # below precharge target (-0.35), above brake target (-0.45)
    self.assertTrue(res.precharge_actuate)
    self.assertFalse(res.brake_actuate)

  def test_extended_pitch_compensation_in_bit_decision(self):
    # -0.2 request on a downgrade contributing -0.3: compensated -0.5 crosses the threshold
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    res = _update(ext, -0.2, pitch=-0.3)
    self.assertTrue(res.brake_actuate)

  def test_extended_pacing_preserves_inactive_gas(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=_lead(v_rel=0.0))
    _prime_speed_allow(ext)
    res = _update(ext, -0.6, op_gas=INACTIVE_GAS)
    self.assertTrue(res.bp_long_used)
    self.assertEqual(res.gas, INACTIVE_GAS)  # no-gas request stays no-gas

  def test_extended_pacing_caps_positive_gas(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=_lead(v_rel=0.0))
    _prime_speed_allow(ext)
    res = _update(ext, 0.5, op_gas=0.5)
    self.assertAlmostEqual(res.gas, 0.2, places=6)

  def test_extended_pacing_cap_floored_at_min_gas(self):
    # steep downgrade: 0.2 + pitch would fall below the propulsion floor; don't invert.
    # cap = max(0.2 + (-0.9), MIN_GAS) = MIN_GAS, so gas is exactly MIN_GAS -- and the
    # floor must come from the cap itself, not be rescued by the final safety clip
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=_lead(v_rel=0.0))
    _prime_speed_allow(ext)
    res = _update(ext, -0.3, op_gas=-0.3, pitch=-0.9)
    self.assertEqual(res.gas, MIN_GAS)
    self.assertEqual(ext.bp_gas_last, MIN_GAS)

  def test_extended_gaining_close_no_gas(self):
    # closing within 1.5s: positive gas capped to 0
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=_lead(d_rel=27.0, v_rel=-0.5, v_lead=26.5))
    _prime_speed_allow(ext)
    res = _update(ext, 0.4, op_gas=0.4)
    self.assertEqual(res.gas, 0.0)

  def test_extended_no_lead_passthrough(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=None)
    _prime_speed_allow(ext)
    res = _update(ext, -1.0, op_gas=INACTIVE_GAS)
    self.assertTrue(res.bp_long_used)
    self.assertEqual(res.accel, -1.0)  # no legacy 0-pin
    self.assertTrue(res.brake_actuate)

  def test_extended_roc_faster_and_following_only(self):
    # with a lead: decel onset limited at ext rate (0.02/scan) from the fresh seed
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=_lead(d_rel=54.0, v_rel=0.0))
    _prime_speed_allow(ext)
    ext.bp_accel_last = 0.0
    res = _update(ext, -1.0, op_gas=INACTIVE_GAS)
    self.assertAlmostEqual(res.accel, -0.02, places=6)

  def test_extended_roc_seed_stays_fresh_without_lead(self):
    # cruise decel passes through, keeping bp_accel_last synced so a new lead's
    # ramp starts from reality instead of legacy's pinned 0
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=None)
    _prime_speed_allow(ext)
    _update(ext, -0.8, op_gas=INACTIVE_GAS)
    self.assertAlmostEqual(ext.bp_accel_last, -0.8, places=6)

  def test_extended_stopping_forces_brakes(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    res = _update(ext, -0.1, stopping=True)  # shallow request, but stopping
    self.assertTrue(res.brake_actuate)
    self.assertTrue(res.precharge_actuate)

  def test_extended_no_positive_gas_while_braking(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    # engage brakes deep, then a shallow frame inside the hysteresis hold with positive op_gas
    _update(ext, -0.6)
    res = _update(ext, -0.25, op_gas=0.1)
    self.assertTrue(res.brake_actuate)  # held by hysteresis
    # capped to an active 0.0 lift-throttle request, NOT the INACTIVE_GAS sentinel
    self.assertEqual(res.gas, 0.0)

  def test_extended_negative_gas_flows_alongside_brakes(self):
    # unlike legacy, braking does not force the propulsion channel inactive
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    _update(ext, -0.6)
    res = _update(ext, -0.3, op_gas=-0.3)
    self.assertTrue(res.brake_actuate)
    self.assertEqual(res.gas, -0.3)

  def test_extended_uphill_no_dead_zone(self):
    # ~2 deg uphill (pitch +0.34): a raw -0.6 request is below the propulsion floor
    # (gas INACTIVE) while comp = -0.26 is still above brake_target. Without the
    # propulsion-floor override, NOTHING would actuate. The brakes must take over.
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    res = _update(ext, -0.6, op_gas=INACTIVE_GAS, pitch=0.34)
    self.assertTrue(res.brake_actuate)
    self.assertTrue(res.precharge_actuate)
    # legacy at the same inputs also brakes (comp -0.26 < -0.14)
    ext_legacy = _make_ext(mode=COASTING_MODE_LEGACY)
    ext_legacy.disable_BP_long_UI = True
    res_legacy = _update(ext_legacy, -0.6, op_gas=INACTIVE_GAS, pitch=0.34)
    self.assertTrue(res_legacy.brake_actuate)

  def test_extended_uphill_floor_holds_across_frames(self):
    # steady sub-MIN_GAS request on a grade must not be released every other frame
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    for _ in range(5):
      res = _update(ext, -0.55, op_gas=INACTIVE_GAS, pitch=0.34)
      self.assertTrue(res.brake_actuate)

  def test_extended_roc_no_positive_pin(self):
    # accelerating behind a departing lead (seed +2.0) that then brakes: the ROC floor
    # starts at min(seed, 0), so the first frame is already <= 0, not +1.98
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=_lead(d_rel=54.0, v_rel=0.0))
    _prime_speed_allow(ext)
    ext.bp_accel_last = 2.0
    res = _update(ext, -1.0, op_gas=INACTIVE_GAS)
    self.assertAlmostEqual(res.accel, -0.02, places=6)

  def test_extended_emergency_ttc_bypasses_roc(self):
    # cut-in: d_rel=20, v_rel=-5 -> ttc=4s; the following ROC must not hold off braking
    ext = _make_ext(mode=COASTING_MODE_EXTENDED, lead=_lead(d_rel=20.0, v_rel=-5.0, v_lead=22.0))
    _prime_speed_allow(ext)
    ext.bp_accel_last = 0.0
    res = _update(ext, -3.5, op_gas=INACTIVE_GAS)
    self.assertTrue(res.bp_long_used)
    self.assertEqual(res.accel, -3.5)

  def test_mode_flip_mid_brake_keeps_bits(self):
    # Braking in legacy at -0.30 asserts the bits and syncs ext_*_last; a mid-drive
    # flip to extended lands inside the extended hysteresis hold band (-0.45..-0.20),
    # so the friction brakes must stay applied on the flip frame instead of dropping
    # to a stale-False seed.
    ext = _make_ext(mode=COASTING_MODE_LEGACY)
    ext.disable_BP_long_UI = True
    res = _update(ext, -0.30)
    self.assertTrue(res.brake_actuate)
    ext.coasting_mode = COASTING_MODE_EXTENDED  # what update_long_params does next frame
    res = _update(ext, -0.30, op_gas=-0.30)
    self.assertTrue(res.brake_actuate)
    self.assertTrue(res.precharge_actuate)

  def test_extended_not_long_active_releases_bits(self):
    ext = _make_ext(mode=COASTING_MODE_EXTENDED)
    ext.disable_BP_long_UI = True
    _update(ext, -0.6)
    res = _update(ext, -0.6, long_active=False)
    self.assertFalse(res.brake_actuate)
    self.assertFalse(res.precharge_actuate)


class TestSafetyEnvelope(unittest.TestCase):
  """Outputs must stay inside the ford.h ACCDATA limits in both modes."""

  def test_gas_clip_floor(self):
    for mode in (COASTING_MODE_LEGACY, COASTING_MODE_EXTENDED):
      ext = _make_ext(mode=mode)
      ext.disable_BP_long_UI = True
      res = _update(ext, 0.0, op_gas=-0.6)  # below the floor: must clip to exactly MIN_GAS
      self.assertEqual(res.gas, MIN_GAS)
      res = _update(ext, 3.0, op_gas=3.0)
      self.assertLessEqual(res.gas, CarControllerParams.ACCEL_MAX)

  def test_accel_clip(self):
    for mode in (COASTING_MODE_LEGACY, COASTING_MODE_EXTENDED):
      ext = _make_ext(mode=mode)
      ext.disable_BP_long_UI = True
      res = _update(ext, -5.0, op_gas=INACTIVE_GAS)
      self.assertGreaterEqual(res.accel, CarControllerParams.ACCEL_MIN)
      res = _update(ext, 5.0, op_gas=2.0)
      self.assertLessEqual(res.accel, CarControllerParams.ACCEL_MAX)

  def test_accel_pred_always_inactive(self):
    for mode in (COASTING_MODE_LEGACY, COASTING_MODE_EXTENDED):
      ext = _make_ext(mode=mode)
      res = _update(ext, -0.3)
      self.assertEqual(res.accel_pred_send, INACTIVE_GAS)


class _CountingParams:
  """Counts param reads so the refresh throttle can be pinned."""

  def __init__(self, values=None):
    self.values = values or {}
    self.reads = 0

  def get_bool(self, key):
    self.reads += 1
    return bool(self.values.get(key, False))

  def get(self, key, return_default=False):
    self.reads += 1
    return self.values.get(key, 0)


class TestParamRefreshThrottle(unittest.TestCase):
  """Params are files on disk; update_long_params runs at 100Hz so reads are throttled."""

  def test_reads_on_first_call_then_throttles(self):
    ext = _make_ext()
    p = _CountingParams({"FordPrefCoastingMode": 1})
    ext.update_long_params(p)
    # applied immediately on the first frame -- no window of stale defaults
    self.assertEqual(ext.coasting_mode, COASTING_MODE_EXTENDED)
    first = p.reads
    self.assertGreater(first, 0)
    # the next PARAM_REFRESH_FRAMES-1 calls must not touch the disk
    for _ in range(PARAM_REFRESH_FRAMES - 1):
      ext.update_long_params(p)
    self.assertEqual(p.reads, first)
    # ...and then exactly one more refresh
    ext.update_long_params(p)
    self.assertEqual(p.reads, 2 * first)

  def test_refreshed_value_is_picked_up(self):
    ext = _make_ext()
    p = _CountingParams({"FordPrefCoastingMode": 0})
    ext.update_long_params(p)
    self.assertEqual(ext.coasting_mode, COASTING_MODE_LEGACY)
    p.values["FordPrefCoastingMode"] = 1
    for _ in range(PARAM_REFRESH_FRAMES):
      ext.update_long_params(p)
    self.assertEqual(ext.coasting_mode, COASTING_MODE_EXTENDED)

  def test_unrecognized_mode_falls_back_to_legacy(self):
    for bad in (-1, 2, 99):
      ext = _make_ext()
      ext.update_long_params(_CountingParams({"FordPrefCoastingMode": bad}))
      self.assertEqual(ext.coasting_mode, COASTING_MODE_LEGACY)


if __name__ == '__main__':
  unittest.main()
