"""BluePilot MICI: longitudinal target debug HUD.

Compact two-line readout above the steering-wheel HUD element:
  line 1: the plan's target speed (display units) + a tag naming the primary limiter
  line 2: planned accel (longitudinalPlan.aTarget, pre-PID) -> applied accel
          (carControl.actuators.accel, post Ford set-speed clamp and PID)

The limiter tag comes from longitudinalPlanSP.primaryLimiter, classified in the planner
where the accel-clip comparison is observable: CRUISE (cluster set speed), SLA (speed
limit), CURVE/MAP (smart cruise curve targets), LEAD, MODEL (e2e/blended accel won the
min), CLIP (comfort/turn/coast accel schedule bound), STOP, FDEC (forced decel).

Toggle: Longitudinal > Longitudinal Target HUD (BPLongitudinalTargetHUD).
"""

import pyray as rl

from cereal import custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

PrimaryLimiter = custom.LongitudinalPlanSP.PrimaryLimiter

PARAM_REFRESH_FRAMES = 60
V_TARGET_UNSET = 200.0  # m/s; inactive controllers publish V_CRUISE_UNSET (255)

# clear of the steering wheel zone: wheel (50px) + bottom margin (14) + powerflow arc (20)
MARGIN_LEFT = 12
MARGIN_BOTTOM = 96

WHITE = rl.Color(255, 255, 255, 235)
DIM = rl.Color(200, 205, 202, 210)
GREEN = rl.Color(60, 220, 120, 255)
BLUE = rl.Color(80, 170, 255, 255)
ORANGE = rl.Color(255, 160, 40, 255)
YELLOW = rl.Color(255, 205, 0, 255)
RED = rl.Color(235, 64, 52, 255)

# (tag, color) per limiter — short tags fit the 536px-wide MICI screen
LIMITER_STYLE = {
  PrimaryLimiter.none: ("--", DIM),
  PrimaryLimiter.cruise: ("CRUISE", WHITE),
  PrimaryLimiter.sccVision: ("CURVE", BLUE),
  PrimaryLimiter.sccMap: ("MAP", BLUE),
  PrimaryLimiter.speedLimitAssist: ("SLA", GREEN),
  PrimaryLimiter.lead: ("LEAD", ORANGE),
  PrimaryLimiter.model: ("MODEL", YELLOW),
  PrimaryLimiter.accelClip: ("CLIP", YELLOW),
  PrimaryLimiter.stopped: ("STOP", RED),
  PrimaryLimiter.forceDecel: ("FDEC", RED),
}


class MiciLongTargetHud(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._frame = 0
    self._enabled = self._params.get_bool("BPLongitudinalTargetHUD")

    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_medium = gui_app.font(FontWeight.MEDIUM)

    self._target_speed_str = "--"
    self._limiter_tag = "--"
    self._limiter_color = DIM
    self._accel_str = ""
    self._engaged = False

  def _update_state(self) -> None:
    self._frame += 1
    if self._frame % PARAM_REFRESH_FRAMES == 0:
      self._enabled = self._params.get_bool("BPLongitudinalTargetHUD")
    if not self._enabled:
      return

    sm = ui_state.sm
    if sm.recv_frame['longitudinalPlanSP'] < ui_state.started_frame:
      self._target_speed_str = "--"
      self._limiter_tag, self._limiter_color = LIMITER_STYLE[PrimaryLimiter.none]
      self._accel_str = ""
      return

    lp_sp = sm['longitudinalPlanSP']
    lp = sm['longitudinalPlan']
    self._engaged = sm['carControl'].enabled

    v_target = lp_sp.vTarget
    if 0. < v_target < V_TARGET_UNSET:
      speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
      self._target_speed_str = str(round(v_target * speed_conv))
    else:
      self._target_speed_str = "--"

    # .raw: reader-side capnp enums don't hash-match the schema-side keys (see cruise_ext)
    self._limiter_tag, self._limiter_color = LIMITER_STYLE.get(lp_sp.primaryLimiter.raw,
                                                               LIMITER_STYLE[PrimaryLimiter.none])

    applied = sm['carControl'].actuators.accel
    self._accel_str = f"a {lp.aTarget:+.2f} → {applied:+.2f}"

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._enabled:
      return

    x = rect.x + MARGIN_LEFT
    base_y = rect.y + rect.height - MARGIN_BOTTOM

    line1_size = 24
    line2_size = 17
    unit = "km/h" if ui_state.is_metric else "mph"

    color = WHITE if self._engaged else DIM

    # line 1: target speed + unit + limiter tag
    speed_txt = self._target_speed_str
    speed_w = measure_text_cached(self._font_semi_bold, speed_txt, line1_size).x
    line1_y = base_y - line1_size - line2_size - 4
    rl.draw_text_ex(self._font_semi_bold, speed_txt, rl.Vector2(int(x), int(line1_y)), line1_size, 0, color)

    unit_w = measure_text_cached(self._font_medium, unit, line2_size).x
    rl.draw_text_ex(self._font_medium, unit, rl.Vector2(int(x + speed_w + 5), int(line1_y + line1_size - line2_size - 1)),
                    line2_size, 0, DIM)

    rl.draw_text_ex(self._font_semi_bold, self._limiter_tag,
                    rl.Vector2(int(x + speed_w + 5 + unit_w + 10), int(line1_y)), line1_size, 0, self._limiter_color)

    # line 2: planned -> applied accel
    if self._accel_str:
      rl.draw_text_ex(self._font_medium, self._accel_str, rl.Vector2(int(x), int(base_y - line2_size)),
                      line2_size, 0, DIM)
