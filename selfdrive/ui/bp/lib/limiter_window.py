"""BluePilot: rolling signed-contribution window for the longitudinal controller icons.

Aggregates the planner's per-frame controller (primaryLimiter, plus the UI-side BRAKE
pseudo-controller) with its signed accel over the last few seconds, so the bottom-center
target cluster can show which controllers drove speed changes.

Controllers are grouped by the glyph they draw, because several distinct controllers
share one icon (vision and map curve control both draw the curve triangle; cruise and
accel-clip both draw the gauge) — without grouping the same icon can occupy two slots.
Summed gross contribution (sum |a|*dt) ranks the groups; summed net (sum a*dt, the
delta-v in m/s attributable to the group) drives icon color saturation (green accel /
red decel, grey at neutral) and vertical offset. Pure logic — no UI imports —
unit-testable headless.
"""

from collections import deque
from collections.abc import Callable

from cereal import custom

PrimaryLimiter = custom.LongitudinalPlanSP.PrimaryLimiter

DEFAULT_WINDOW_S = 4.0
SAMPLE_DT = 0.05  # 20 Hz plan cadence

# UI-side pseudo-controller for driver braking (outside the capnp enum range)
BRAKE_CONTROLLER = 1000

# Glyph identities. Controllers sharing one are one icon in the row.
ICON_BRAKE = 'brake'    # BRAKE text
ICON_STOP = 'stop'      # stop octagon
ICON_MODEL = 'model'    # experimental flask
ICON_CURVE = 'curve'    # curve warning triangle
ICON_LEAD = 'lead'      # following-distance circle
ICON_CRUISE = 'cruise'  # dashboard cruise gauge
ICON_NONE = ''          # draws nothing


def icon_key(controller: int, model_stopping: bool = False) -> str:
  """The glyph a controller draws, '' when it draws none.

  Speed Limit Assist deliberately has no glyph — the speed limit sign is its indication.
  The model's glyph depends on intent: it shares the octagon while planning a stop,
  otherwise it is the experimental flask.
  """
  if controller in (BRAKE_CONTROLLER, PrimaryLimiter.forceDecel):
    return ICON_BRAKE
  if controller == PrimaryLimiter.stopped:
    return ICON_STOP
  if controller == PrimaryLimiter.model:
    return ICON_STOP if model_stopping else ICON_MODEL
  if controller in (PrimaryLimiter.sccVision, PrimaryLimiter.sccMap):
    return ICON_CURVE
  if controller == PrimaryLimiter.lead:
    return ICON_LEAD
  if controller in (PrimaryLimiter.cruise, PrimaryLimiter.accelClip):
    return ICON_CRUISE
  return ICON_NONE


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

  def ranked_groups(self, now: float, key: Callable[[int], str]) -> list[tuple[str, float]]:
    """[(glyph, net delta-v)] ranked by summed gross contribution, biggest first.

    Controllers whose key is '' (no glyph) and groups with no contribution are dropped.
    One entry per glyph, so the caller can never draw the same icon twice.
    """
    self.prune(now)

    groups: dict[str, list[float]] = {}  # glyph -> [gross, net]
    for controller, (gross, net) in self._sums().items():
      glyph = key(controller)
      if not glyph:
        continue
      group = groups.setdefault(glyph, [0.0, 0.0])
      group[0] += gross
      group[1] += net

    ranked = sorted(groups.items(), key=lambda kv: kv[1][0], reverse=True)
    return [(glyph, net) for glyph, (gross, net) in ranked if gross > 0.0]
