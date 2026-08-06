"""BluePilot: tests for the signed contribution window behind the controller icons."""

from openpilot.selfdrive.ui.bp.lib.limiter_window import (
  BRAKE_CONTROLLER, ContributionWindow, PrimaryLimiter, SAMPLE_DT, icon_key,
  ICON_BRAKE, ICON_CRUISE, ICON_CURVE, ICON_LEAD, ICON_MODEL, ICON_NONE, ICON_STOP)

CRUISE = PrimaryLimiter.cruise
CLIP = PrimaryLimiter.accelClip
VISION = PrimaryLimiter.sccVision
MAP = PrimaryLimiter.sccMap
SLA = PrimaryLimiter.speedLimitAssist
LEAD = PrimaryLimiter.lead
MODEL = PrimaryLimiter.model
STOPPED = PrimaryLimiter.stopped
FORCE_DECEL = PrimaryLimiter.forceDecel


def key_driving(c):
  return icon_key(c, model_stopping=False)


def key_stopping(c):
  return icon_key(c, model_stopping=True)


def fill(win, start, seconds, controller, accel=0.0):
  t = start
  end = start + seconds
  while t < end:
    win.add(t, controller, accel)
    t += SAMPLE_DT
  return end


def glyphs(groups):
  return [glyph for glyph, _ in groups]


class TestIconKey:
  """The glyph mapping is the dedup key: controllers sharing a glyph are one icon."""

  def test_cruise_and_accel_clip_share_the_gauge(self):
    assert icon_key(CRUISE) == icon_key(CLIP) == ICON_CRUISE

  def test_vision_and_map_share_the_curve_triangle(self):
    assert icon_key(VISION) == icon_key(MAP) == ICON_CURVE

  def test_brake_and_force_decel_share_the_brake_text(self):
    assert icon_key(BRAKE_CONTROLLER) == icon_key(FORCE_DECEL) == ICON_BRAKE

  def test_model_shares_the_octagon_only_while_stopping(self):
    assert icon_key(MODEL, model_stopping=True) == icon_key(STOPPED) == ICON_STOP
    assert icon_key(MODEL, model_stopping=False) == ICON_MODEL

  def test_lead_is_its_own_glyph(self):
    assert icon_key(LEAD) == ICON_LEAD

  def test_speed_limit_assist_and_none_draw_nothing(self):
    # the speed limit sign itself is SLA's indication
    assert icon_key(SLA) == ICON_NONE
    assert icon_key(PrimaryLimiter.none) == ICON_NONE


class TestContributionWindow:

  def test_empty(self):
    win = ContributionWindow()
    assert win.ranked_groups(10.0, key_driving) == []

  def test_gross_ranking(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, MODEL, accel=-2.0)   # gross 2.0
    t = fill(win, t, 2.0, CRUISE, accel=0.5)     # gross 1.0
    t = fill(win, t, 0.5, LEAD, accel=-0.5)      # gross 0.25
    assert glyphs(win.ranked_groups(t, key_driving)) == [ICON_MODEL, ICON_CRUISE, ICON_LEAD]

  def test_shared_glyph_appears_once(self):
    # the bug this fixes: cruise and accel-clip are distinct controllers drawing one gauge —
    # keyed by controller they took two slots, keyed by glyph they are a single entry
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, CRUISE, accel=0.4)
    t = fill(win, t, 1.0, CLIP, accel=0.6)
    groups = win.ranked_groups(t, key_driving)
    assert glyphs(groups) == [ICON_CRUISE]

  def test_shared_glyph_sums_contributions(self):
    # merged icons carry the group's combined delta-v, so the color/offset stay truthful
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, VISION, accel=-1.0)
    t = fill(win, t, 1.0, MAP, accel=-0.5)
    (glyph, net), = win.ranked_groups(t, key_driving)
    assert glyph == ICON_CURVE
    assert abs(net - (-1.5)) < 0.11

  def test_merged_group_outranks_a_larger_single_controller(self):
    # two curve controllers at 0.6 each beat a lead at 1.0 once merged
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, LEAD, accel=-1.0)     # gross 1.0
    t = fill(win, t, 1.0, VISION, accel=-0.6)     # gross 0.6
    t = fill(win, t, 1.0, MAP, accel=-0.6)        # gross 0.6, merged 1.2
    assert glyphs(win.ranked_groups(t, key_driving)) == [ICON_CURVE, ICON_LEAD]

  def test_model_merges_into_the_stop_octagon_while_stopping(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, MODEL, accel=-1.0)
    t = fill(win, t, 1.0, STOPPED, accel=-0.5)
    assert glyphs(win.ranked_groups(t, key_driving)) == [ICON_MODEL, ICON_STOP]
    assert glyphs(win.ranked_groups(t, key_stopping)) == [ICON_STOP]

  def test_speed_limit_assist_is_dropped(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 2.0, SLA, accel=-1.0)
    t = fill(win, t, 0.5, LEAD, accel=-0.2)
    assert glyphs(win.ranked_groups(t, key_driving)) == [ICON_LEAD]

  def test_net_signed_delta_v(self):
    # 1 s at -2.0 then 1 s at +1.0: net = -1.0 m/s, gross keeps it top-ranked
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, MODEL, accel=-2.0)
    t = fill(win, t, 1.0, MODEL, accel=1.0)
    (glyph, net), = win.ranked_groups(t, key_driving)
    assert glyph == ICON_MODEL
    assert abs(net - (-1.0)) < 0.11

  def test_accel_then_equal_decel_nets_to_neutral(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, MODEL, accel=1.0)
    t = fill(win, t, 1.0, MODEL, accel=-1.0)
    (_, net), = win.ranked_groups(t, key_driving)
    assert abs(net) < 0.11

  def test_window_expiry(self):
    win = ContributionWindow(window_s=4.0)
    t = fill(win, 0.0, 0.5, LEAD, accel=-2.0)
    t = fill(win, t, 5.0, CRUISE, accel=0.2)
    assert glyphs(win.ranked_groups(t, key_driving)) == [ICON_CRUISE]

  def test_set_window_shrinks_history(self):
    win = ContributionWindow(window_s=5.0)
    t = fill(win, 0.0, 1.0, LEAD, accel=-1.0)
    t = fill(win, t, 1.0, CRUISE, accel=0.5)
    win.set_window(0.5)
    assert glyphs(win.ranked_groups(t, key_driving)) == [ICON_CRUISE]

  def test_zero_window_keeps_nothing(self):
    win = ContributionWindow(window_s=0.0)
    t = fill(win, 0.0, 1.0, MODEL, accel=-1.0)
    assert win.ranked_groups(t + SAMPLE_DT, key_driving) == []

  def test_zero_accel_samples_do_not_rank(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 2.0, CRUISE, accel=0.0)
    assert win.ranked_groups(t, key_driving) == []

  def test_clear(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, LEAD, accel=-1.0)
    win.clear()
    assert win.ranked_groups(t, key_driving) == []
