"""BluePilot MICI: speed limit sign + longitudinal target cluster.

The sign (MUTCD imperial / Vienna circle metric) shows the resolved speed limit on the
left edge, between the driver-monitoring face and the steering wheel. It flashes with an
underglow in sync with the chime-only speedLimit* alerts (visual replacement for the old
text banners).

The longitudinal target lives at the bottom center, over the steering torque curve
(toggle: BPLongitudinalTargetHUD): the current limiter's target speed in large white
text, with up to three road-sign icons above it naming the longitudinal controllers.
Leftmost is the current controller; the two slots right of it are the largest
contributors over a rolling window (BPLimiterWindow, 0-5 s) ranked by gross |accel|
contribution. The row is keyed by glyph rather than by controller, so an icon shared by
several controllers can never occupy two slots. All icons are the same 48px size,
matching the experimental flask on the home screen. The sign yields to the transient
set-speed readout that owns the top-left corner (same protocol as the DM face). Icon
language:
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
from openpilot.selfdrive.ui.bp.lib.limiter_window import (
  BRAKE_CONTROLLER, ContributionWindow, icon_key,
  ICON_BRAKE, ICON_CRUISE, ICON_CURVE, ICON_FORCE_DECEL, ICON_LEAD, ICON_MODEL, ICON_STOP)
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

# Sign column: centered on the DM face (16 + 60/2) and steering wheel (21 + 50/2)
# column, vertically between the face (bottom y=70) and the powerflow arc around the
# wheel (top y≈156).
SIGN_CX = 46
SIGN_CY = 113

MUTCD_W = 60
MUTCD_H = 78
VIENNA_RADIUS = 37

# Underglow radius: the sign sits ~4px from the DM face above and the powerflow arc
# below, so the flash must decay before it reaches them (they are ~43px from center)
GLOW_RADIUS = 50

# Bottom-center target cluster, aligned with the torque bar's arc center
TORQUE_CENTER_OFFSET_X = 8   # torque bar draws 8px right of the camera-feed center
TARGET_FONT_SIZE = 54
TARGET_BOTTOM_MARGIN = 16
TARGET_ICON_GAP = 6
TARGET_SHADOW_DEPTH = 3      # black rim so white digits survive the white torque-bar fill

ICON_ROW_H = 60
ICON_SLOT_W = 56
ICON_R = 24              # icon half-size: every icon is 2r = 48px (home-screen flask size)
BRAKE_FONT_SIZE = 20     # widest label that fits the slot; "BRAKE" is text by design

# TEMPORARY (diagnostic): vision and map curve control share the curve triangle, so a small
# eye / map-pin badge above it names which source is actually contributing. Remove once the
# split-vs-merge question is settled. Hand-drawn: there is no map/pin asset anywhere in
# selfdrive/assets, and the eye assets are unfetched LFS pointers that already mean
# "driver monitoring" elsewhere in MICI.
CURVE_BADGE_R = 7
CURVE_BADGE_GAP = 3
CONTRIB_FULL_DV = 1.5    # m/s net delta-v for full color saturation
CONTRIB_PX_PER_DV = 8.0  # vertical px per m/s net delta-v (accel raises, decel lowers)
CONTRIB_MAX_OFFSET = 12.0

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

    self._experimental_tex = gui_app.texture('icons_mici/experimental_mode.png', 2 * ICON_R, 2 * ICON_R)

    self._alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._target_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

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
    # (glyph, net_dv, member controllers)
    self._slots: list[tuple[str, float, frozenset[int]] | None] = [None, None, None]

    # the transient set-speed readout owns the top-left corner; the sign yields to it
    self._top_icons_active = False

  def set_top_icons_active(self, active: bool) -> None:
    self._top_icons_active = active

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

    # everything below reads longitudinalPlanSP; at the start of a new drive the last
    # message SubMaster holds is the previous drive's — show nothing until fresh data
    if sm.recv_frame['longitudinalPlanSP'] < ui_state.started_frame:
      self._speed_limit = 0.0
      self._speed_limit_valid = False
      self._has_limit = False
      self._flash_until = 0.0
      self._alert_flash = False
      self._curve_active_prev = False
      self._window.clear()
      self._current_controller = None
      self._target_value = None
      self._slots = [None, None, None]
      return

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

    # curve activation still drives the flash; the curve target shows at bottom center
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

    # The row is keyed by glyph, not by controller: several controllers share one icon
    # (vision/map curve, cruise/accel-clip, stopped/model-while-stopping, brake/force-decel),
    # so keying by controller could put the same icon in two slots. Slot 0 is the current
    # controller's glyph; slots 1-2 are the largest other glyph groups in the window. SLA
    # has no glyph — the speed limit sign is its indication.
    groups = self._window.ranked_groups(now, self._icon_key)
    current = self._current_controller
    current_glyph = self._icon_key(current) if current is not None else ''

    slots: list[tuple[str, float, frozenset[int]] | None] = [None, None, None]
    if current_glyph:
      slots[0] = next((g for g in groups if g[0] == current_glyph),
                      (current_glyph, 0.0, frozenset({current})))
    for i, group in enumerate([g for g in groups if g[0] != current_glyph][:2]):
      slots[i + 1] = group
    self._slots = slots

  def _icon_key(self, controller: int) -> str:
    return icon_key(controller, self._model_stopping)

  @property
  def _flashing(self) -> bool:
    return self._alert_flash or time.monotonic() < self._flash_until

  def _render(self, rect: rl.Rectangle) -> None:
    # keep clear of visible (non-chime) alerts, which own the top of the screen
    alert_showing = ui_state.sm['selfdriveState'].alertSize != 0

    # the sign yields to the transient set-speed readout, like the DM face does
    show_limit = (ui_state.started and not alert_showing and not self._top_icons_active and
                  self._has_limit and self._speed_limit > 0 and (self._persistent or self._flashing))
    show_target = (ui_state.started and not alert_showing and self._icons_enabled and
                   self._target_value is not None)

    alpha = self._alpha_filter.update(1.0 if show_limit else 0.0)
    if alpha >= 1e-2:
      cx = rect.x + SIGN_CX
      cy = rect.y + SIGN_CY
      if self._flashing:
        self._draw_underglow(cx, cy, alpha)
      if ui_state.is_metric:
        self._draw_vienna(cx, cy, alpha)
      else:
        self._draw_mutcd(cx, cy, alpha)

    target_alpha = self._target_alpha_filter.update(1.0 if show_target else 0.0)
    if target_alpha >= 1e-2:
      self._draw_target_cluster(rect, target_alpha)

  def _draw_underglow(self, cx: float, cy: float, alpha: float) -> None:
    pulse = 0.45 + 0.55 * abs(math.sin(time.monotonic() * math.pi * 1.5))
    color = rl.Color(GLOW_LIMIT.r, GLOW_LIMIT.g, GLOW_LIMIT.b, int(200 * pulse * alpha))
    rl.draw_circle_gradient(rl.Vector2(cx, cy), GLOW_RADIUS, color, rl.BLANK)

  def _value_text_color(self, alpha: float) -> rl.Color:
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

  def _draw_mutcd(self, cx: float, cy: float, alpha: float) -> None:
    x = cx - MUTCD_W / 2
    y = cy - MUTCD_H / 2
    sign_rect = rl.Rectangle(x, y, MUTCD_W, MUTCD_H)

    white = rl.color_alpha(WHITE, alpha)
    black = rl.color_alpha(BLACK, alpha)

    rl.draw_rectangle_rounded(sign_rect, 0.25, 8, white)
    inner = rl.Rectangle(x + 4, y + 4, MUTCD_W - 8, MUTCD_H - 8)
    rl.draw_rectangle_rounded_lines_ex(inner, 0.25, 8, 2, black)

    self._draw_text_centered(self._font_semi_bold, "SPEED", 13, cx, y + 14, black)
    self._draw_text_centered(self._font_semi_bold, "LIMIT", 13, cx, y + 27, black)
    self._draw_text_centered(self._font_bold, str(round(self._speed_limit)), 36, cx, y + 53,
                             self._value_text_color(alpha))

  def _draw_vienna(self, cx: float, cy: float, alpha: float) -> None:
    radius = VIENNA_RADIUS
    center = rl.Vector2(cx, cy)

    rl.draw_circle_v(center, radius, rl.color_alpha(WHITE, alpha))
    rl.draw_ring(center, radius * 0.72, radius, 0, 360, 36, rl.color_alpha(RED, alpha))

    val = str(round(self._speed_limit))
    font_size = 26 if len(val) >= 3 else 32
    self._draw_text_centered(self._font_bold, val, font_size, center.x, center.y,
                             self._value_text_color(alpha))

  # ---- bottom-center target cluster (target speed + controller icon row) ----

  def _draw_target_cluster(self, rect: rl.Rectangle, alpha: float) -> None:
    cx = rect.x + rect.width / 2 + TORQUE_CENTER_OFFSET_X

    text_top = rect.y + rect.height - TARGET_BOTTOM_MARGIN - TARGET_FONT_SIZE
    if self._target_value is not None:
      text = str(round(self._target_value))
      text_cy = text_top + TARGET_FONT_SIZE / 2
      # black rim (complication shadow idiom, all four diagonals): the torque-bar fill
      # under this text is white at 0.9 alpha, so a bare white glyph would vanish in it
      shadow = rl.Color(0, 0, 0, int(180 * alpha))
      for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
        self._draw_text_centered(self._font_bold, text, TARGET_FONT_SIZE,
                                 cx + dx * TARGET_SHADOW_DEPTH, text_cy + dy * TARGET_SHADOW_DEPTH, shadow)
      self._draw_text_centered(self._font_bold, text, TARGET_FONT_SIZE, cx, text_cy,
                               rl.color_alpha(WHITE, alpha))

    row_cy = text_top - TARGET_ICON_GAP - ICON_ROW_H / 2
    self._draw_icon_row(cx, row_cy, alpha)

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

  def _draw_icon_row(self, cx: float, row_cy: float, alpha: float) -> None:
    row_left = cx - 1.5 * ICON_SLOT_W

    for i, slot in enumerate(self._slots):
      if slot is None:
        continue
      glyph, net_dv, members = slot
      tint, dy = self._contrib_style(net_dv, alpha)
      icon_cx = row_left + (i + 0.5) * ICON_SLOT_W
      self._draw_icon(glyph, icon_cx, row_cy + dy, ICON_R, tint, alpha, members)

  def _draw_icon(self, glyph: str, cx: float, cy: float, r: float,
                 tint: rl.Color, alpha: float, members: frozenset[int] = frozenset()) -> None:
    if glyph == ICON_BRAKE:
      self._draw_text_centered(self._font_bold, "BRAKE", BRAKE_FONT_SIZE, cx, cy, tint)
    elif glyph == ICON_FORCE_DECEL:
      self._draw_force_decel(cx, cy, r, tint)
    elif glyph == ICON_STOP:
      self._draw_octagon(cx, cy, r, tint, alpha)
    elif glyph == ICON_MODEL:
      scale = (2 * r) / self._experimental_tex.width
      pos = rl.Vector2(cx - self._experimental_tex.width * scale / 2, cy - self._experimental_tex.height * scale / 2)
      rl.draw_texture_ex(self._experimental_tex, pos, 0.0, scale, tint)
    elif glyph == ICON_CURVE:
      self._draw_curve_triangle(cx, cy, r, tint)
      self._draw_curve_source_badges(cx, cy, r, tint, members)
    elif glyph == ICON_LEAD:
      self._draw_following_distance(cx, cy, r, tint)
    elif glyph == ICON_CRUISE:
      self._draw_cruise_gauge(cx, cy, r, tint)

  def _draw_octagon(self, cx: float, cy: float, r: float, tint: rl.Color, alpha: float) -> None:
    center = rl.Vector2(cx, cy)
    rl.draw_poly(center, 8, r, 22.5, tint)
    rl.draw_poly_lines_ex(center, 8, r, 22.5, 3, rl.color_alpha(WHITE, alpha))

  def _draw_curve_triangle(self, cx: float, cy: float, r: float, tint: rl.Color) -> None:
    # warning triangle with a bend arrow
    top = rl.Vector2(cx, cy - r)
    left = rl.Vector2(cx - r, cy + r * 0.85)
    right = rl.Vector2(cx + r, cy + r * 0.85)
    for a, b in ((top, left), (left, right), (right, top)):
      rl.draw_line_ex(a, b, 4, tint)
    rl.draw_ring(rl.Vector2(cx - r * 0.25, cy + r * 0.55), r * 0.35, r * 0.55, 270, 360, 10, tint)

  def _draw_force_decel(self, cx: float, cy: float, r: float, tint: rl.Color) -> None:
    # (!) — the system forcing the car down (driver monitoring timeout or a fault
    # soft-disabling), deliberately distinct from the driver's own BRAKE
    center = rl.Vector2(cx, cy)
    rl.draw_ring(center, r - 3.5, r, 0, 360, 24, tint)
    bar_w, bar_top, bar_bot = 3.5, cy - r * 0.45, cy + r * 0.18
    rl.draw_rectangle_rounded(rl.Rectangle(cx - bar_w / 2, bar_top, bar_w, bar_bot - bar_top), 0.5, 4, tint)
    rl.draw_circle(int(cx), int(cy + r * 0.42), bar_w / 2, tint)

  def _draw_curve_source_badges(self, cx: float, cy: float, r: float, tint: rl.Color,
                                members: frozenset[int]) -> None:
    """TEMPORARY: name which curve controller is behind the shared triangle."""
    badges = [glyph for controller, glyph in
              ((PrimaryLimiter.sccVision, 'eye'), (PrimaryLimiter.sccMap, 'map'))
              if controller in members]
    if not badges:
      return

    span = len(badges) * 2 * CURVE_BADGE_R + (len(badges) - 1) * CURVE_BADGE_GAP
    bx = cx - span / 2 + CURVE_BADGE_R
    by = cy - r - CURVE_BADGE_R - 1
    for badge in badges:
      if badge == 'eye':
        rl.draw_ellipse_lines(int(bx), int(by), CURVE_BADGE_R, CURVE_BADGE_R * 0.62, tint)
        rl.draw_circle(int(bx), int(by), CURVE_BADGE_R * 0.3, tint)
      else:
        head_r = CURVE_BADGE_R * 0.62
        head_cy = by - CURVE_BADGE_R * 0.28
        rl.draw_circle(int(bx), int(head_cy), head_r, tint)
        rl.draw_poly(rl.Vector2(bx, head_cy + head_r * 0.75), 3, head_r * 0.95, 90, tint)
      bx += 2 * CURVE_BADGE_R + CURVE_BADGE_GAP

  def _draw_following_distance(self, cx: float, cy: float, r: float, tint: rl.Color) -> None:
    # two cars in a circle: keep-your-distance
    rl.draw_ring(rl.Vector2(cx, cy), r - 3.5, r, 0, 360, 24, tint)
    car_w, car_h = r * 0.55, r * 0.4
    for dx in (-r * 0.45, r * 0.45):
      rl.draw_rectangle_rounded(rl.Rectangle(cx + dx - car_w / 2, cy - car_h / 2, car_w, car_h), 0.5, 4, tint)

  def _draw_cruise_gauge(self, cx: float, cy: float, r: float, tint: rl.Color) -> None:
    # dashboard cruise-control symbol: open speedometer arc with a needle
    center = rl.Vector2(cx, cy)
    rl.draw_ring(center, r - 4, r, 135, 405, 20, tint)
    needle_angle = math.radians(-45)
    tip = rl.Vector2(cx + r * 0.75 * math.cos(needle_angle), cy + r * 0.75 * math.sin(needle_angle))
    rl.draw_line_ex(center, tip, 4, tint)
