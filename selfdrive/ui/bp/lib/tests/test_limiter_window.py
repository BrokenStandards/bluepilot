"""BluePilot: tests for the longitudinal HUD's rolling limiter-contribution window."""

from openpilot.selfdrive.ui.bp.lib.limiter_window import LimiterWindow

# stand-in ordinals matching cereal PrimaryLimiter usage
CRUISE, CURVE, SLA, LEAD, MODEL = 1, 2, 4, 5, 6

DT = 0.05  # 20 Hz plan rate


def fill(win, start, seconds, limiter, a_target=0.0):
  t = start
  end = start + seconds
  while t < end:
    win.add(t, limiter, a_target, DT)
    t += DT
  return end


class TestLimiterWindow:

  def test_empty(self):
    assert LimiterWindow().top(10.0) == []

  def test_single_limiter(self):
    win = LimiterWindow()
    t = fill(win, 0.0, 1.0, SLA)
    assert win.top(t) == [SLA]

  def test_dwell_ranking(self):
    win = LimiterWindow()
    t = fill(win, 0.0, 2.0, CRUISE)
    t = fill(win, t, 0.5, LEAD)
    assert win.top(t) == [CRUISE, LEAD]

  def test_braking_outweighs_dwell(self):
    # a hard 0.5 s brake demand must outrank 2 s of idle capping
    win = LimiterWindow()
    t = fill(win, 0.0, 2.0, CRUISE, a_target=0.0)
    t = fill(win, t, 0.5, LEAD, a_target=-2.5)
    assert win.top(t) == [LEAD, CRUISE]

  def test_positive_accel_does_not_boost(self):
    win = LimiterWindow()
    t = fill(win, 0.0, 1.0, CRUISE, a_target=2.0)
    t = fill(win, t, 1.5, CURVE, a_target=0.0)
    assert win.top(t) == [CURVE, CRUISE]

  def test_nominal_limiter_pinned_last(self):
    # SLA dominates dwell in steady state but must sit at the bottom so transients surface
    win = LimiterWindow()
    t = fill(win, 0.0, 3.0, SLA)
    t = fill(win, t, 0.3, LEAD, a_target=-1.0)
    t = fill(win, t, 0.2, CURVE, a_target=-0.5)
    assert win.top(t, bottom_limiter=SLA) == [LEAD, CURVE, SLA]

  def test_nominal_limiter_alone_still_shows(self):
    win = LimiterWindow()
    t = fill(win, 0.0, 2.0, SLA)
    assert win.top(t, bottom_limiter=SLA) == [SLA]

  def test_max_three_tags(self):
    win = LimiterWindow()
    t = 0.0
    for i, limiter in enumerate((CRUISE, CURVE, LEAD, MODEL)):
      t = fill(win, t, 1.0 - i * 0.2, limiter)  # descending dwell
    assert win.top(t) == [CRUISE, CURVE, LEAD]

  def test_window_expiry(self):
    # a transient braking event stays visible for the window, then ages out
    win = LimiterWindow()
    t = fill(win, 0.0, 0.5, LEAD, a_target=-2.0)
    t = fill(win, t, 1.0, SLA)
    assert win.top(t, bottom_limiter=SLA)[0] == LEAD

    t = fill(win, t, 4.0, SLA)  # keep driving; lead sample ages past the 4 s window
    assert win.top(t, bottom_limiter=SLA) == [SLA]

  def test_clear(self):
    win = LimiterWindow()
    t = fill(win, 0.0, 1.0, LEAD)
    win.clear()
    assert win.top(t) == []
