"""BluePilot: rolling signed-contribution window for the longitudinal controller icons.

Aggregates the planner's per-frame controller (primaryLimiter, plus the UI-side BRAKE
pseudo-controller) with its signed accel over the last few seconds, so the bottom-center
target cluster can show which controllers drove speed changes: gross contribution (sum |a|*dt) ranks the
top contributors, net contribution (sum a*dt, i.e. the delta-v in m/s attributable to the
controller) drives icon color saturation (green accel / red decel, grey at neutral) and
vertical offset. Pure logic — no UI imports — unit-testable headless.
"""

from collections import deque

DEFAULT_WINDOW_S = 4.0
SAMPLE_DT = 0.05  # 20 Hz plan cadence


class ContributionWindow:
  def __init__(self, window_s: float = DEFAULT_WINDOW_S):
    self._window_s = window_s
    self._samples: deque[tuple[float, int, float]] = deque()  # (ts, controller, accel m/s^2)

  def set_window(self, window_s: float) -> None:
    self._window_s = max(0.0, window_s)

  @property
  def window_s(self) -> float:
    return self._window_s

  def clear(self) -> None:
    self._samples.clear()

  def add(self, ts: float, controller: int, accel: float) -> None:
    self._samples.append((ts, controller, accel))
    self.prune(ts)

  def prune(self, now: float) -> None:
    while self._samples and now - self._samples[0][0] > self._window_s:
      self._samples.popleft()

  def _sums(self) -> dict[int, tuple[float, float]]:
    sums: dict[int, tuple[float, float]] = {}
    for _, controller, accel in self._samples:
      gross, net = sums.get(controller, (0.0, 0.0))
      sums[controller] = (gross + abs(accel) * SAMPLE_DT, net + accel * SAMPLE_DT)
    return sums

  def net(self, controller: int) -> float:
    """Signed delta-v (m/s) attributable to the controller within the window."""
    return self._sums().get(controller, (0.0, 0.0))[1]

  def contributors(self, now: float, exclude: tuple[int, ...] = (), n: int = 2) -> list[int]:
    """Top-n controllers by gross contribution, excluding `exclude`."""
    self.prune(now)
    sums = self._sums()
    ranked = sorted((c for c in sums if c not in exclude), key=lambda c: sums[c][0], reverse=True)
    return [c for c in ranked if sums[c][0] > 0.0][:n]
