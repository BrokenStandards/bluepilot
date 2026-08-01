"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import log, custom
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import classify_primary_limiter

PrimaryLimiter = custom.LongitudinalPlanSP.PrimaryLimiter
SPSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
StockSource = log.LongitudinalPlan.LongitudinalPlanSource


class TestClassifyPrimaryLimiter:

  def test_stopped_wins_over_everything(self):
    assert classify_primary_limiter(True, True, True, StockSource.e2e, SPSource.speedLimitAssist) == PrimaryLimiter.stopped

  def test_force_decel(self):
    assert classify_primary_limiter(False, True, False, StockSource.cruise, SPSource.cruise) == PrimaryLimiter.forceDecel

  def test_accel_clip_binds_before_source_attribution(self):
    # final accel = clip(min(model, mpc)): when the clip reduced the request, the clip is the limiter
    assert classify_primary_limiter(False, False, True, StockSource.cruise, SPSource.speedLimitAssist) == PrimaryLimiter.accelClip

  def test_model_when_e2e_won_the_min(self):
    assert classify_primary_limiter(False, False, False, StockSource.e2e, SPSource.cruise) == PrimaryLimiter.model

  def test_lead(self):
    for lead in (StockSource.lead0, StockSource.lead1, StockSource.lead2):
      assert classify_primary_limiter(False, False, False, lead, SPSource.cruise) == PrimaryLimiter.lead

  def test_sp_source_refines_cruise(self):
    # the mpc cruise obstacle embodies the SP min(), so the SP source names the true limiter
    cases = {
      SPSource.cruise: PrimaryLimiter.cruise,
      SPSource.sccVision: PrimaryLimiter.sccVision,
      SPSource.sccMap: PrimaryLimiter.sccMap,
      SPSource.speedLimitAssist: PrimaryLimiter.speedLimitAssist,
    }
    for sp_source, expected in cases.items():
      assert classify_primary_limiter(False, False, False, StockSource.cruise, sp_source) == expected
