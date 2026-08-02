"""BluePilot MICI: speed limit sign overlay + longitudinal controller icons.

The sign (MUTCD imperial / Vienna circle metric) is the single home for the longitudinal
target: when a controller is actively limiting, its target speed is shown in the sign
(curve targets included — the old yellow curve diamond is gone), falling back to the
resolved speed limit otherwise. It flashes with an underglow in sync with the chime-only
speedLimit* alerts (visual replacement for the old text banners).

Above the sign, up to three road-sign icons name the longitudinal controllers (toggle:
BPLongitudinalTargetHUD). Leftmost (slightly larger) is the current controller; the two
slots right of it are the largest contributors over a rolling window (BPLimiterWindow,
0-5 s) ranked by gross |accel| contribution. Icon language:
  curve triangle = vision/map curve control     octagon = stopped / model stop intent
  following-distance circle = lead              gauge = cruise set speed / accel schedule
  experimental flask = e2e model                BRAKE = manual braking
  (no icon for Speed Limit Assist — the sign itself is its indication)
Each icon is tinted by its net delta-v over the window — green accelerating, red
decelerating, fading to grey at neutral — and floats above/below the row centerline in
proportion (accel up, decel down).
"""

import math
import time

import pyray as rl

from cereal import custom
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.limiter_window import ContributionWindow
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

AssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState
MapState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState
PrimaryLimiter = custom.LongitudinalPlanSP.PrimaryLimiter

SLA_ACTIVE_STATES = (AssistState.active, AssistState.adapting)
SCC_VISION_ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)

# selfdriveState.alertType is "<EventName>/<eventType>"; these are the chime-only speed
# limit adjust alerts whose visual is this widget's flash.
SLA_FLASH_EVENTS = ('speedLimitActive', 'speedLimitChanged', 'speedLimitPending')

FLASH_DURATION = 5.0  # s, matches the old text alert / chime alert duration
PARAM_REFRESH_FRAMES = 60
V_TARGET_UNSET = 200.0  # m/s; inactive controllers publish V_CRUISE_UNSET (255)

# UI-side pseudo-controller for driver braking (outside the capnp enum range)
BRAKE_CONTROLLER = 1000

MARGIN_TOP = 10
MARGIN_RIGHT = 14

MUTCD_W = 60
MUTCD_H = 78
VIENNA_RADIUS = 37

ICON_ROW_H = 36
ICON_SLOT_W = 30
ICON_R = 11              # base icon half-size
CURRENT_SCALE = 1.2      # leftmost/current slot is slightly larger
CONTRIB_FULL_DV = 1.5    # m/s net delta-v for full color saturation
CONTRIB_PX_PER_DV = 6.0  # vertical px per m/s net delta-v (accel raises, decel lowers)
CONTRIB_MAX_OFFSET = 9.0

WHITE = rl.WHITE
BLACK = rl.BLACK
RED = rl.Color(235, 32, 32, 255)
GREY = rl.Color(145, 155, 149, 255)
GLOW_LIMIT = rl.Color(255, 255, 255, 255)
NEUTRAL_GREY = 168
CONTRIB_GREEN = rl.Color(50, 220, 110, 255)
CONTRIB_RED = rl.Color(240, 60, 50, 255)


class MiciSpeedLimitSign(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._frame = 0
    self._persistent = self._params.get_bool("BPSpeedLimitSignOverlay")
    self._icons_enabled = self._params.get_bool("BPLongitudinalTargetHUD")

    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)

    self._experimental_tex = gui_app.texture('icons_mici/experimental_mode.png', 2 * ICON_R + 6, 2 * ICON_R + 6)

    self._alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    self._speed_limit = 0.0
    self._speed_limit_valid = False
    self._has_limit = False
    self._speed_limit_final_last = 0.0
    self._speed = 0.0

    self._curve_active = False
    self._curve_active_prev = False

    self._flash_until = 0.0
    self._alert_flash = False

    # controller telemetry
    self._window = ContributionWindow(self._read_window_param())
    self._engaged = False
    self._current_controller: int | None = None
    self._model_stopping = False
    self._target_value: float | None = None  # display units
    self._slots: list[tuple[int, float] | None] = [None, None, None]  # (controller, net_dv)

  def _read_window_param(self) -> float:
    try:
      val = float(self._params.get("BPLimiterWindow", return_default=True))
    except (TypeError, ValueError):
      val = 4.0
    return min(max(val, 0.0), 5.0)

  @property
  def _speed_conv(self) -> float:
    return CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH

  def _limiter_target_ms(self, sm, lp_sp, lp) -> float | None:
    """Target speed (m/s) of the limiter binding right now; None when there is none."""
    limiter = lp_sp.primaryLimiter.raw
    if limiter == PrimaryLimiter.stopped:
      return 0.0
    if limiter == PrimaryLimiter.speedLimitAssist:
      v = lp_sp.speedLimit.assist.vTarget
    elif limiter == PrimaryLimiter.sccVision:
      v = lp_sp.smartCruiseControl.vision.vTarget
    elif limiter == PrimaryLimiter.sccMap:
      v = lp_sp.smartCruiseControl.map.vTarget
    elif limiter == PrimaryLimiter.lead:
      lead = sm['radarState'].leadOne
      v = max(0.0, sm['carState'].vEgo + lead.vRel) if lead.status else 0.0
    elif limiter == PrimaryLimiter.model:
      speeds = lp.speeds
      v = speeds[len(speeds) - 1] if len(speeds) else 0.0
    else:  # cruise / accelClip / forceDecel
      v = lp_sp.vTarget
    if not 0.0 < v < V_TARGET_UNSET:
      return None
    return v

  def _update_state(self) -> None:
    self._frame += 1
    if self._frame % PARAM_REFRESH_FRAMES == 0:
      self._persistent = self._params.get_bool("BPSpeedLimitSignOverlay")
      self._icons_enabled = self._params.get_bool("BPLongitudinalTargetHUD")
      self._window.set_window(self._read_window_param())

    sm = ui_state.sm
    lp_sp = sm['longitudinalPlanSP']
    lp = sm['longitudinalPlan']
    resolver = lp_sp.speedLimit.resolver
    scc = lp_sp.smartCruiseControl

    self._speed_limit = resolver.speedLimitLast * self._speed_conv
    self._speed_limit_valid = resolver.speedLimitValid
    self._has_limit = resolver.speedLimitValid or resolver.speedLimitLastValid
    self._speed_limit_final_last = resolver.speedLimitFinalLast * self._speed_conv

    car_state = sm['carState']
    v_ego = car_state.vEgoCluster if car_state.vEgoCluster != 0.0 else car_state.vEgo
    self._speed = max(0.0, v_ego * self._speed_conv)

    # curve activation still drives the flash; the target itself now lives in the sign
    vision_active = scc.vision.state in SCC_VISION_ACTIVE_STATES
    map_active = scc.map.state == MapState.turning
    self._curve_active = vision_active or map_active

    now = time.monotonic()
    if self._curve_active and not self._curve_active_prev:
      self._flash_until = now + FLASH_DURATION
    self._curve_active_prev = self._curve_active

    alert_event = sm['selfdriveState'].alertType.split('/')[0]
    self._alert_flash = alert_event in SLA_FLASH_EVENTS

    # controller telemetry
    self._engaged = sm['carControl'].enabled
    if sm.recv_frame['longitudinalPlanSP'] < ui_state.started_frame:
      self._window.clear()
      self._current_controller = None
      self._target_value = None
      self._slots = [None, None, None]
      return

    self._model_stopping = bool(lp.shouldStop)
    target_ms = self._limiter_target_ms(sm, lp_sp, lp) if self._engaged else None
    self._target_value = target_ms * self._speed_conv if target_ms is not None else None

    if sm.updated['longitudinalPlanSP']:
      if self._engaged:
        controller = lp_sp.primaryLimiter.raw
        if controller != PrimaryLimiter.none:
          self._window.add(now, controller, lp.aTarget)
          self._current_controller = controller
      else:
        self._current_controller = None
      if car_state.brakePressed:
        self._window.add(now, BRAKE_CONTROLLER, min(car_state.aEgo, 0.0))
        self._current_controller = BRAKE_CONTROLLER
    self._window.prune(now)

    # slot 0: current controller; slots 1-2: top contributors in the window, excluding
    # slot 0 and SLA (the sign itself is SLA's indication). Schema-side enum attrs compare
    # equal to the raw ints stored in the window.
    slots: list[tuple[int, float] | None] = [None, None, None]
    hidden = (PrimaryLimiter.speedLimitAssist,)
    current = self._current_controller
    if current is not None and current not in hidden:
      slots[0] = (current, self._window.net(current))
    exclude = hidden + ((current,) if current is not None else ())
    for i, controller in enumerate(self._window.contributors(now, exclude=exclude, n=2)):
      slots[i + 1] = (controller, self._window.net(controller))
    self._slots = slots

  @property
  def _flashing(self) -> bool:
    return self._alert_flash or time.monotonic() < self._flash_until

  def _render(self, rect: rl.Rectangle) -> None:
    show_limit = self._has_limit and self._speed_limit > 0
    show_target = self._icons_enabled and self._target_value is not None
    visible = ui_state.started and (show_limit or show_target) and (self._persistent or self._flashing)

    # keep clear of visible (non-chime) alerts, which own the top of the screen
    if ui_state.sm['selfdriveState'].alertSize != 0:
      visible = False

    alpha = self._alpha_filter.update(1.0 if visible else 0.0)
    if alpha < 1e-2:
      return

    center_x = rect.x + rect.width - MARGIN_RIGHT - max(MUTCD_W, VIENNA_RADIUS * 2) / 2

    if self._flashing:
      self._draw_underglow(center_x, alpha)

    value = self._target_value if show_target else self._speed_limit
    if ui_state.is_metric:
      self._draw_vienna(center_x, value, show_target, alpha)
    else:
      self._draw_mutcd(center_x, value, show_target, alpha)

    if self._icons_enabled:
      self._draw_icon_row(center_x, alpha)

  def _row_h(self) -> float:
    return ICON_ROW_H if self._icons_enabled else 0

  def _sign_top(self) -> float:
    return self._rect.y + MARGIN_TOP + self._row_h()

  def _sign_center_y(self) -> float:
    return self._sign_top() + (VIENNA_RADIUS if ui_state.is_metric else MUTCD_H / 2)

  def _draw_underglow(self, center_x: float, alpha: float) -> None:
    pulse = 0.45 + 0.55 * abs(math.sin(time.monotonic() * math.pi * 1.5))
    color = rl.Color(GLOW_LIMIT.r, GLOW_LIMIT.g, GLOW_LIMIT.b, int(200 * pulse * alpha))
    rl.draw_circle_gradient(rl.Vector2(center_x, self._sign_center_y()), MUTCD_H * 0.85, color, rl.BLANK)

  def _value_text_color(self, showing_target: bool, alpha: float) -> rl.Color:
    is_overspeed = self._has_limit and round(self._speed_limit_final_last) < round(self._speed)
    if not showing_target and is_overspeed:
      color = RED
    elif not showing_target and not self._speed_limit_valid:
      color = GREY
    else:
      color = BLACK
    return rl.color_alpha(color, alpha)

  def _draw_text_centered(self, font, text: str, size: int, cx: float, cy: float, color: rl.Color) -> None:
    sz = measure_text_cached(font, text, size)
    rl.draw_text_ex(font, text, rl.Vector2(cx - sz.x / 2, cy - sz.y / 2), size, 0, color)

  def _draw_mutcd(self, center_x: float, value: float, showing_target: bool, alpha: float) -> None:
    x = center_x - MUTCD_W / 2
    y = self._sign_top()
    sign_rect = rl.Rectangle(x, y, MUTCD_W, MUTCD_H)

    white = rl.color_alpha(WHITE, alpha)
    black = rl.color_alpha(BLACK, alpha)

    rl.draw_rectangle_rounded(sign_rect, 0.25, 8, white)
    inner = rl.Rectangle(x + 4, y + 4, MUTCD_W - 8, MUTCD_H - 8)
    rl.draw_rectangle_rounded_lines_ex(inner, 0.25, 8, 2, black)

    self._draw_text_centered(self._font_semi_bold, "SPEED", 13, center_x, y + 14, black)
    self._draw_text_centered(self._font_semi_bold, "LIMIT", 13, center_x, y + 27, black)
    self._draw_text_centered(self._font_bold, str(round(value)), 36, center_x, y + 53,
                             self._value_text_color(showing_target, alpha))

  def _draw_vienna(self, center_x: float, value: float, showing_target: bool, alpha: float) -> None:
    radius = VIENNA_RADIUS
    center = rl.Vector2(center_x, self._sign_top() + radius)

    rl.draw_circle_v(center, radius, rl.color_alpha(WHITE, alpha))
    rl.draw_ring(center, radius * 0.72, radius, 0, 360, 36, rl.color_alpha(RED, alpha))

    val = str(round(value))
    font_size = 26 if len(val) >= 3 else 32
    self._draw_text_centered(self._font_bold, val, font_size, center.x, center.y,
                             self._value_text_color(showing_target, alpha))

  # ---- controller icon row ----

  def _contrib_style(self, net_dv: float, alpha: float) -> tuple[rl.Color, float]:
    """Tint + vertical offset from the controller's net delta-v over the window."""
    sat = min(abs(net_dv) / CONTRIB_FULL_DV, 1.0)
    base = CONTRIB_GREEN if net_dv > 0 else CONTRIB_RED
    r = int(NEUTRAL_GREY + (base.r - NEUTRAL_GREY) * sat)
    g = int(NEUTRAL_GREY + (base.g - NEUTRAL_GREY) * sat)
    b = int(NEUTRAL_GREY + (base.b - NEUTRAL_GREY) * sat)
    # accel raises the icon, decel lowers it
    offset = -max(-CONTRIB_MAX_OFFSET, min(CONTRIB_MAX_OFFSET, net_dv * CONTRIB_PX_PER_DV))
    return rl.Color(r, g, b, int(255 * alpha)), offset

  def _draw_icon_row(self, sign_center_x: float, alpha: float) -> None:
    right_edge = sign_center_x + max(MUTCD_W, VIENNA_RADIUS * 2) / 2
    row_cy = self._rect.y + MARGIN_TOP + ICON_ROW_H / 2

    for i, slot in enumerate(self._slots):
      if slot is None:
        continue
      controller, net_dv = slot
      tint, dy = self._contrib_style(net_dv, alpha)
      scale = CURRENT_SCALE if i == 0 else 1.0
      cx = right_edge - (3 - i) * ICON_SLOT_W + ICON_SLOT_W / 2
      cy = row_cy + dy
      self._draw_controller_icon(controller, cx, cy, ICON_R * scale, tint, alpha)

  def _draw_controller_icon(self, controller: int, cx: float, cy: float, r: float,
                            tint: rl.Color, alpha: float) -> None:
    if controller == BRAKE_CONTROLLER or controller == PrimaryLimiter.forceDecel:
      self._draw_text_centered(self._font_bold, "BRAKE", int(r), cx, cy, tint)
    elif controller == PrimaryLimiter.stopped:
      self._draw_octagon(cx, cy, r, tint, alpha)
    elif controller == PrimaryLimiter.model:
      if self._model_stopping:
        self._draw_octagon(cx, cy, r, tint, alpha)
      else:
        scale = (2 * r + 6) / self._experimental_tex.width
        pos = rl.Vector2(cx - self._experimental_tex.width * scale / 2, cy - self._experimental_tex.height * scale / 2)
        rl.draw_texture_ex(self._experimental_tex, pos, 0.0, scale, tint)
    elif controller in (PrimaryLimiter.sccVision, PrimaryLimiter.sccMap):
      self._draw_curve_triangle(cx, cy, r, tint)
    elif controller == PrimaryLimiter.lead:
      self._draw_following_distance(cx, cy, r, tint)
    elif controller in (PrimaryLimiter.cruise, PrimaryLimiter.accelClip):
      self._draw_cruise_gauge(cx, cy, r, tint)

  def _draw_octagon(self, cx: float, cy: float, r: float, tint: rl.Color, alpha: float) -> None:
    center = rl.Vector2(cx, cy)
    rl.draw_poly(center, 8, r, 22.5, tint)
    rl.draw_poly_lines_ex(center, 8, r, 22.5, 2, rl.color_alpha(WHITE, alpha))

  def _draw_curve_triangle(self, cx: float, cy: float, r: float, tint: rl.Color) -> None:
    # warning triangle with a bend arrow
    top = rl.Vector2(cx, cy - r)
    left = rl.Vector2(cx - r, cy + r * 0.85)
    right = rl.Vector2(cx + r, cy + r * 0.85)
    for a, b in ((top, left), (left, right), (right, top)):
      rl.draw_line_ex(a, b, 2.5, tint)
    rl.draw_ring(rl.Vector2(cx - r * 0.25, cy + r * 0.55), r * 0.35, r * 0.55, 270, 360, 10, tint)

  def _draw_following_distance(self, cx: float, cy: float, r: float, tint: rl.Color) -> None:
    # two cars in a circle: keep-your-distance
    rl.draw_ring(rl.Vector2(cx, cy), r - 2, r, 0, 360, 24, tint)
    car_w, car_h = r * 0.55, r * 0.4
    for dx in (-r * 0.45, r * 0.45):
      rl.draw_rectangle_rounded(rl.Rectangle(cx + dx - car_w / 2, cy - car_h / 2, car_w, car_h), 0.5, 4, tint)

  def _draw_cruise_gauge(self, cx: float, cy: float, r: float, tint: rl.Color) -> None:
    # dashboard cruise-control symbol: open speedometer arc with a needle
    center = rl.Vector2(cx, cy)
    rl.draw_ring(center, r - 2.5, r, 135, 405, 20, tint)
    needle_angle = math.radians(-45)
    tip = rl.Vector2(cx + r * 0.75 * math.cos(needle_angle), cy + r * 0.75 * math.sin(needle_angle))
    rl.draw_line_ex(center, tip, 2.5, tint)
