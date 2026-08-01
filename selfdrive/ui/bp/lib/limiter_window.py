"""BluePilot: rolling contribution window for the longitudinal target HUD.

Aggregates the planner's per-frame primaryLimiter over the last few seconds so transient
events (a brief lead detection, a curve slowdown, a misread stop) stay readable after the
fact, and ranks concurrent limiters by how much they actually contributed to slowing or
capping the car. Pure logic — no UI imports — so it stays unit-testable headless.
"""

from collections import deque

WINDOW_S = 4.0  # s of history shown; long enough to read a tag after a quick braking event
# One second of mere presence scores 1.0; each m/s^2 of demanded braking during a frame
# multiplies that frame's contribution, so a hard 0.5 s brake outranks seconds of idling.
BRAKE_WEIGHT_GAIN = 2.0
MAX_TAGS = 3


class LimiterWindow:
  def __init__(self, window_s: float = WINDOW_S):
    self._window_s = window_s
    self._samples: deque[tuple[float, int, float]] = deque()  # (ts, limiter, weight)

  def clear(self) -> None:
    self._samples.clear()

  def add(self, ts: float, limiter: int, a_target: float, dt: float) -> None:
    weight = dt * (1.0 + BRAKE_WEIGHT_GAIN * max(0.0, -a_target))
    self._samples.append((ts, limiter, weight))
    self._prune(ts)

  def _prune(self, now: float) -> None:
    while self._samples and now - self._samples[0][0] > self._window_s:
      self._samples.popleft()

  def top(self, now: float, bottom_limiter: int | None = None, max_tags: int = MAX_TAGS) -> list[int]:
    """Up to max_tags limiters from the window, highest contribution first.

    bottom_limiter (the nominal limiter, e.g. the speed limit) always sorts last when
    present, so transient culprits surface first regardless of its steady-state dwell."""
    self._prune(now)

    scores: dict[int, float] = {}
    for _, limiter, weight in self._samples:
      scores[limiter] = scores.get(limiter, 0.0) + weight

    ranked = sorted(scores, key=lambda k: scores[k], reverse=True)[:max_tags]
    if bottom_limiter is not None and bottom_limiter in ranked:
      ranked = [x for x in ranked if x != bottom_limiter] + [bottom_limiter]
    return ranked
