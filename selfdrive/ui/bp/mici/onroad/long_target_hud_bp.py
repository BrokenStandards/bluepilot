"""BluePilot MICI: longitudinal target debug HUD.

Compact two-line readout above the steering-wheel HUD element:
  line 1: the plan's target speed (display units) + up to three tags naming the limiters
          that contributed over the last few seconds, biggest contributor first
  line 2: planned accel (longitudinalPlan.aTarget, pre-PID) -> applied accel
          (carControl.actuators.accel, post Ford set-speed clamp and PID)

Per-frame limiters come from longitudinalPlanSP.primaryLimiter (classified in the planner
where the accel-clip comparison is observable). A rolling LimiterWindow aggregates them so
a quick braking event stays readable after the fact, ranked by contribution (dwell time
weighted by braking demand). The speed limit tag is pinned to the end of the list — it is
the nominal limiter, so transient culprits (lead flicker, curve, model stop) surface first.

Tags: CRUISE (cluster set speed), SLA (speed limit), CURVE/MAP (smart cruise curve
targets), LEAD, MODEL (e2e/blended accel won the min), CLIP (comfort/turn/coast accel
schedule bound), STOP, FDEC (forced decel).

Toggle: Longitudinal > Longitudinal Target HUD (BPLongitudinalTargetHUD).
"""

import time

import pyray as rl

from cereal import custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.limiter_window import LimiterWindow
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

PrimaryLimiter = custom.LongitudinalPlanSP.PrimaryLimiter

PARAM_REFRESH_FRAMES = 60
V_TARGET_UNSET = 200.0  # m/s; inactive controllers publish V_CRUISE_UNSET (255)
V_CRUISE_UNSET_KPH = 255.0
PLAN_DT = 0.05  # 20 Hz longitudinalPlanSP cadence

# pseudo-limiter for driver braking (UI-side only, outside the capnp enum range)
BRAKE_LIMITER = 1000

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
MAGENTA = rl.Color(240, 100, 220, 255)

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
  BRAKE_LIMITER: ("BRAKE", MAGENTA),
}


class MiciLongTargetHud(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._frame = 0
    self._enabled = self._params.get_bool("BPLongitudinalTargetHUD")

    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_medium = gui_app.font(FontWeight.MEDIUM)

    self._window = LimiterWindow()
    self._target_speed_str = "--"
    self._tags: list[tuple[str, rl.Color]] = []
    self._accel_str = ""
    self._engaged = False

  def _limiter_target_speed_str(self, sm, lp_sp, lp) -> str:
    """Target speed of the limiter that is binding RIGHT NOW, in display units.

    The SP min() (lp_sp.vTarget) does not reflect lead/model/mpc constraints, so each
    limiter contributes its own target instead of always echoing the SLA/cruise value."""
    # .raw: reader-side capnp enums don't hash-match the schema-side keys (see cruise_ext)
    limiter = lp_sp.primaryLimiter.raw
    if limiter == PrimaryLimiter.stopped:
      return "0"
    if limiter == PrimaryLimiter.speedLimitAssist:
      v = lp_sp.speedLimit.assist.vTarget
    elif limiter == PrimaryLimiter.sccVision:
      v = lp_sp.smartCruiseControl.vision.vTarget
    elif limiter == PrimaryLimiter.sccMap:
      v = lp_sp.smartCruiseControl.map.vTarget
    elif limiter == PrimaryLimiter.lead:
      lead = sm['radarState'].leadOne
      v = max(0., sm['carState'].vEgo + lead.vRel) if lead.status else 0.
    elif limiter == PrimaryLimiter.model:
      # the model outputs accel, not a set speed; the plan's end-of-horizon speed is the
      # closest thing to where it is steering the car
      speeds = lp.speeds
      v = speeds[len(speeds) - 1] if len(speeds) else 0.
    else:  # cruise / accelClip / forceDecel
      v = lp_sp.vTarget

    if not 0. < v < V_TARGET_UNSET:
      return "--"
    speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    return str(round(v * speed_conv))

  def _update_state(self) -> None:
    self._frame += 1
    if self._frame % PARAM_REFRESH_FRAMES == 0:
      self._enabled = self._params.get_bool("BPLongitudinalTargetHUD")
    if not self._enabled:
      return

    sm = ui_state.sm
    if sm.recv_frame['longitudinalPlanSP'] < ui_state.started_frame:
      self._window.clear()
      self._target_speed_str = "--"
      self._tags = []
      self._accel_str = ""
      return

    lp_sp = sm['longitudinalPlanSP']
    lp = sm['longitudinalPlan']
    cs = sm['carState']
    self._engaged = sm['carControl'].enabled

    # target speed only means something while openpilot longitudinal is engaged — before
    # engagement the plan idles at V_CRUISE_MAX and churns limiters
    self._target_speed_str = self._limiter_target_speed_str(sm, lp_sp, lp) if self._engaged else "--"

    now = time.monotonic()
    if sm.updated['longitudinalPlanSP']:
      if self._engaged:
        limiter = lp_sp.primaryLimiter.raw
        if limiter != PrimaryLimiter.none:
          self._window.add(now, limiter, lp.aTarget, PLAN_DT)
      # manual braking disengages openpilot long, so it never appears as a plan limiter —
      # track it as its own cause
      if cs.brakePressed:
        self._window.add(now, BRAKE_LIMITER, min(cs.aEgo, 0.), PLAN_DT)

    ranked = self._window.top(now, bottom_limiter=PrimaryLimiter.speedLimitAssist)
    self._tags = [LIMITER_STYLE.get(limiter, LIMITER_STYLE[PrimaryLimiter.none]) for limiter in ranked]

    applied = sm['carControl'].actuators.accel
    self._accel_str = f"a {lp.aTarget:+.2f} → {applied:+.2f}"

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._enabled:
      return

    x = rect.x + MARGIN_LEFT
    base_y = rect.y + rect.height - MARGIN_BOTTOM

    line1_size = 26
    line2_size = 22
    unit = "km/h" if ui_state.is_metric else "mph"

    color = WHITE if self._engaged else DIM

    # line 1: target speed + unit + limiter tags (primary big, contributors smaller/dimmer)
    speed_txt = self._target_speed_str
    speed_w = measure_text_cached(self._font_semi_bold, speed_txt, line1_size).x
    line1_y = base_y - line1_size - line2_size - 4
    rl.draw_text_ex(self._font_semi_bold, speed_txt, rl.Vector2(int(x), int(line1_y)), line1_size, 0, color)

    unit_w = measure_text_cached(self._font_medium, unit, line2_size).x
    rl.draw_text_ex(self._font_medium, unit, rl.Vector2(int(x + speed_w + 5), int(line1_y + line1_size - line2_size - 1)),
                    line2_size, 0, DIM)

    tag_x = x + speed_w + 5 + unit_w + 10
    tags = self._tags if self._tags else [LIMITER_STYLE[PrimaryLimiter.none]]
    for i, (tag, tag_color) in enumerate(tags):
      size = line1_size if i == 0 else line2_size
      y = line1_y if i == 0 else line1_y + line1_size - line2_size - 1
      draw_color = tag_color if i == 0 else rl.Color(tag_color.r, tag_color.g, tag_color.b, 170)
      rl.draw_text_ex(self._font_semi_bold, tag, rl.Vector2(int(tag_x), int(y)), size, 0, draw_color)
      tag_x += measure_text_cached(self._font_semi_bold, tag, size).x + 9

    # line 2: planned -> applied accel
    if self._accel_str:
      rl.draw_text_ex(self._font_semi_bold, self._accel_str, rl.Vector2(int(x), int(base_y - line2_size)),
                      line2_size, 0, WHITE if self._engaged else DIM)
