"""BluePilot MICI: persistent speed limit / curve speed sign overlay.

Replaces the old "Auto adjusting to speed limit" text banner. The chime still comes from
selfdrived (the speedLimit* alerts are now AlertSize.none with sound kept); this widget
flashes the sign with an underglow for the alert's duration, keyed off
selfdriveState.alertType so the flash stays in sync with the ding.

Shows:
- the resolved speed limit as a MUTCD (imperial) or Vienna (metric) sign, persistent while
  a limit is known when BPSpeedLimitSignOverlay is enabled, transient (flash-only) otherwise
- a yellow curve-warning diamond with the targeted curve speed while Smart Cruise Control
  (vision or map) is actively slowing for a curve
"""

import math
import time

import pyray as rl

from cereal import custom
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

AssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState
MapState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState

SLA_ACTIVE_STATES = (AssistState.active, AssistState.adapting)
SCC_VISION_ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)

# selfdriveState.alertType is "<EventName>/<eventType>"; these are the chime-only speed
# limit adjust alerts whose visual is this widget's flash.
SLA_FLASH_EVENTS = ('speedLimitActive', 'speedLimitChanged', 'speedLimitPending')

FLASH_DURATION = 5.0  # s, matches the old text alert / chime alert duration
PARAM_REFRESH_FRAMES = 60
V_TARGET_UNSET = 200.0  # m/s; scc vTarget publishes V_CRUISE_UNSET (255) when inactive

MARGIN_TOP = 10
MARGIN_RIGHT = 14

MUTCD_W = 60
MUTCD_H = 78
VIENNA_RADIUS = 37
DIAMOND_RADIUS = 42

WHITE = rl.WHITE
BLACK = rl.BLACK
RED = rl.Color(235, 32, 32, 255)
GREY = rl.Color(145, 155, 149, 255)
MUTCD_YELLOW = rl.Color(255, 205, 0, 255)
GLOW_LIMIT = rl.Color(255, 255, 255, 255)
GLOW_CURVE = rl.Color(255, 190, 0, 255)


class MiciSpeedLimitSign(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._frame = 0
    self._persistent = self._params.get_bool("BPSpeedLimitSignOverlay")

    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)

    self._alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    self._speed_limit = 0.0
    self._speed_limit_valid = False
    self._has_limit = False
    self._speed_limit_final_last = 0.0
    self._speed = 0.0

    self._curve_active = False
    self._curve_active_prev = False
    self._curve_speed = 0.0

    self._flash_until = 0.0
    self._alert_flash = False

  @property
  def _speed_conv(self) -> float:
    return CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH

  def _update_state(self) -> None:
    self._frame += 1
    if self._frame % PARAM_REFRESH_FRAMES == 0:
      self._persistent = self._params.get_bool("BPSpeedLimitSignOverlay")

    sm = ui_state.sm
    lp_sp = sm['longitudinalPlanSP']
    resolver = lp_sp.speedLimit.resolver
    scc = lp_sp.smartCruiseControl

    self._speed_limit = resolver.speedLimitLast * self._speed_conv
    self._speed_limit_valid = resolver.speedLimitValid
    self._has_limit = resolver.speedLimitValid or resolver.speedLimitLastValid
    self._speed_limit_final_last = resolver.speedLimitFinalLast * self._speed_conv

    car_state = sm['carState']
    v_ego = car_state.vEgoCluster if car_state.vEgoCluster != 0.0 else car_state.vEgo
    self._speed = max(0.0, v_ego * self._speed_conv)

    # curve slowdown: targeted speed decided from vision/map data
    vision_active = scc.vision.state in SCC_VISION_ACTIVE_STATES
    map_active = scc.map.state == MapState.turning
    targets = []
    if vision_active and scc.vision.vTarget < V_TARGET_UNSET:
      targets.append(scc.vision.vTarget)
    if map_active and scc.map.vTarget < V_TARGET_UNSET:
      targets.append(scc.map.vTarget)

    self._curve_active = bool(targets)
    if self._curve_active:
      self._curve_speed = min(targets) * self._speed_conv

    now = time.monotonic()

    # flash on curve control activation, for as long as the old text alert would have shown
    if self._curve_active and not self._curve_active_prev:
      self._flash_until = now + FLASH_DURATION
    self._curve_active_prev = self._curve_active

    # flash in sync with the speed limit adjust chime: the chime-only alert stays current
    # for its full duration, so alertType holds the event name for the whole flash window
    alert_event = sm['selfdriveState'].alertType.split('/')[0]
    self._alert_flash = alert_event in SLA_FLASH_EVENTS

  @property
  def _flashing(self) -> bool:
    return self._alert_flash or time.monotonic() < self._flash_until

  def _render(self, rect: rl.Rectangle) -> None:
    show_limit = self._has_limit and self._speed_limit > 0
    visible = ui_state.started and (self._curve_active or show_limit) and (self._persistent or self._flashing)

    # keep clear of visible (non-chime) alerts, which own the top of the screen
    if ui_state.sm['selfdriveState'].alertSize != 0:
      visible = False

    alpha = self._alpha_filter.update(1.0 if visible else 0.0)
    if alpha < 1e-2:
      return

    center_x = rect.x + rect.width - MARGIN_RIGHT - max(MUTCD_W, DIAMOND_RADIUS * 2) / 2
    if self._flashing:
      self._draw_underglow(center_x, alpha)

    if self._curve_active:
      self._draw_curve_sign(center_x, alpha)
    elif show_limit:
      if ui_state.is_metric:
        self._draw_vienna(center_x, alpha)
      else:
        self._draw_mutcd(center_x, alpha)

  def _sign_center_y(self) -> float:
    return self._rect.y + MARGIN_TOP + (DIAMOND_RADIUS * 2 if self._curve_active else MUTCD_H) / 2

  def _draw_underglow(self, center_x: float, alpha: float) -> None:
    # soft pulsing glow behind the sign for the duration the old text alert would have shown
    pulse = 0.45 + 0.55 * abs(math.sin(time.monotonic() * math.pi * 1.5))
    glow = GLOW_CURVE if self._curve_active else GLOW_LIMIT
    center_y = self._sign_center_y()
    radius = (DIAMOND_RADIUS * 2 if self._curve_active else MUTCD_H) * 0.85
    color = rl.Color(glow.r, glow.g, glow.b, int(200 * pulse * alpha))
    rl.draw_circle_gradient(rl.Vector2(center_x, center_y), radius, color, rl.BLANK)

  def _limit_text_color(self, alpha: float) -> rl.Color:
    is_overspeed = self._has_limit and round(self._speed_limit_final_last) < round(self._speed)
    if is_overspeed:
      color = RED
    elif not self._speed_limit_valid:
      color = GREY
    else:
      color = BLACK
    return rl.color_alpha(color, alpha)

  def _draw_text_centered(self, font, text: str, size: int, cx: float, cy: float, color: rl.Color) -> None:
    sz = measure_text_cached(font, text, size)
    rl.draw_text_ex(font, text, rl.Vector2(cx - sz.x / 2, cy - sz.y / 2), size, 0, color)

  def _draw_mutcd(self, center_x: float, alpha: float) -> None:
    x = center_x - MUTCD_W / 2
    y = self._rect.y + MARGIN_TOP
    sign_rect = rl.Rectangle(x, y, MUTCD_W, MUTCD_H)

    white = rl.color_alpha(WHITE, alpha)
    black = rl.color_alpha(BLACK, alpha)

    rl.draw_rectangle_rounded(sign_rect, 0.25, 8, white)
    inner = rl.Rectangle(x + 4, y + 4, MUTCD_W - 8, MUTCD_H - 8)
    rl.draw_rectangle_rounded_lines_ex(inner, 0.25, 8, 2, black)

    self._draw_text_centered(self._font_semi_bold, "SPEED", 13, center_x, y + 14, black)
    self._draw_text_centered(self._font_semi_bold, "LIMIT", 13, center_x, y + 27, black)
    self._draw_text_centered(self._font_bold, str(round(self._speed_limit)), 36, center_x, y + 53,
                             self._limit_text_color(alpha))

  def _draw_vienna(self, center_x: float, alpha: float) -> None:
    radius = VIENNA_RADIUS
    center = rl.Vector2(center_x, self._rect.y + MARGIN_TOP + radius)

    rl.draw_circle_v(center, radius, rl.color_alpha(WHITE, alpha))
    rl.draw_ring(center, radius * 0.72, radius, 0, 360, 36, rl.color_alpha(RED, alpha))

    val = str(round(self._speed_limit))
    font_size = 26 if len(val) >= 3 else 32
    self._draw_text_centered(self._font_bold, val, font_size, center.x, center.y, self._limit_text_color(alpha))

  def _draw_curve_sign(self, center_x: float, alpha: float) -> None:
    radius = DIAMOND_RADIUS
    center = rl.Vector2(center_x, self._rect.y + MARGIN_TOP + radius)

    black = rl.color_alpha(BLACK, alpha)
    yellow = rl.color_alpha(MUTCD_YELLOW, alpha)

    # diamond: outer black border, inner yellow face
    for r, color in ((radius, black), (radius - 4, yellow)):
      pts = [
        rl.Vector2(center.x, center.y - r),
        rl.Vector2(center.x - r, center.y),
        rl.Vector2(center.x, center.y + r),
        rl.Vector2(center.x + r, center.y),
      ]
      rl.draw_triangle_fan(pts, len(pts), color)

    # curve arrow glyph: arc bending right with an arrowhead
    arc_center = rl.Vector2(center.x - 8, center.y - 8)
    rl.draw_ring(arc_center, 10, 15, 0, 90, 16, black)
    head = [
      rl.Vector2(center.x + 3, center.y - 26),
      rl.Vector2(center.x + 13, center.y - 16),
      rl.Vector2(center.x - 2, center.y - 14),
    ]
    rl.draw_triangle_fan(head, len(head), black)

    # targeted curve speed from vision/map data
    self._draw_text_centered(self._font_bold, str(max(0, round(self._curve_speed))), 26, center.x, center.y + 14, black)
