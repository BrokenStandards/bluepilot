"""BluePilot: tests for the signed contribution window behind the controller icons."""

from openpilot.selfdrive.ui.bp.lib.limiter_window import ContributionWindow, SAMPLE_DT

CRUISE, CURVE, SLA, LEAD, MODEL = 1, 2, 4, 5, 6


def fill(win, start, seconds, controller, accel=0.0):
  t = start
  end = start + seconds
  while t < end:
    win.add(t, controller, accel)
    t += SAMPLE_DT
  return end


class TestContributionWindow:

  def test_empty(self):
    win = ContributionWindow()
    assert win.contributors(10.0) == []
    assert win.net(MODEL) == 0.0

  def test_gross_ranking(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, MODEL, accel=-2.0)   # gross 2.0
    t = fill(win, t, 2.0, CRUISE, accel=0.5)     # gross 1.0
    t = fill(win, t, 0.5, LEAD, accel=-0.5)      # gross 0.25
    assert win.contributors(t, n=3) == [MODEL, CRUISE, LEAD]

  def test_exclude(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, MODEL, accel=-2.0)
    t = fill(win, t, 1.0, CRUISE, accel=0.5)
    assert win.contributors(t, exclude=(MODEL,)) == [CRUISE]

  def test_net_signed_delta_v(self):
    # 1 s at -2.0 then 1 s at +1.0: net = -1.0 m/s, gross keeps it top-ranked
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, MODEL, accel=-2.0)
    t = fill(win, t, 1.0, MODEL, accel=1.0)
    assert abs(win.net(MODEL) - (-1.0)) < 0.11
    assert win.contributors(t) == [MODEL]

  def test_accel_then_equal_decel_nets_to_neutral(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 1.0, MODEL, accel=1.0)
    t = fill(win, t, 1.0, MODEL, accel=-1.0)
    assert abs(win.net(MODEL)) < 0.11

  def test_window_expiry(self):
    win = ContributionWindow(window_s=4.0)
    t = fill(win, 0.0, 0.5, LEAD, accel=-2.0)
    t = fill(win, t, 5.0, CRUISE, accel=0.2)
    assert win.contributors(t) == [CRUISE]
    assert win.net(LEAD) == 0.0

  def test_set_window_shrinks_history(self):
    win = ContributionWindow(window_s=5.0)
    t = fill(win, 0.0, 1.0, LEAD, accel=-1.0)
    t = fill(win, t, 1.0, CRUISE, accel=0.5)
    win.set_window(0.5)
    assert win.contributors(t) == [CRUISE]

  def test_zero_window_keeps_nothing(self):
    win = ContributionWindow(window_s=0.0)
    t = fill(win, 0.0, 1.0, MODEL, accel=-1.0)
    win.prune(t + SAMPLE_DT)
    assert win.contributors(t + SAMPLE_DT) == []

  def test_zero_accel_samples_do_not_rank(self):
    win = ContributionWindow()
    t = fill(win, 0.0, 2.0, SLA, accel=0.0)
    assert win.contributors(t) == []
