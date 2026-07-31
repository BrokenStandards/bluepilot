"""
BluePilot: Ford runtime settings snapshot, read on card.py's params thread.

opendbc is meant to be openpilot-independent. Every other brand takes its settings
through CarParamsSP at car init (opendbc/sunnypilot/car/interfaces.py), and settings that
must stay live while driving reach the car layer as derived state on carControlSP -- see
how MADS and ICBM do it. Ford was the only brand importing openpilot Params and reading
them directly inside the 100Hz CarController.update(), which put ~1500 file reads/s into
a Priority.CTRL_HIGH realtime process.

This module does the reading in openpilot land, on card.py's existing 10Hz params thread,
and card.py hands the result to the car layer as structs.FordSettingsBP. Every key stays
runtime-tunable exactly as before; only the read cadence changes (100Hz -> 10Hz), so a
settings change now applies within ~100ms instead of ~10ms. No driver-visible behavior
depends on that difference, and it keeps the onroad lateral-mode tap target responsive.

Values are validated/clamped here, at the boundary, so the car layer can trust the struct
and an out-of-band param write can never reach the control path with a bad value.
"""

from opendbc.car import structs
from openpilot.common.params import Params


def _get_bool(p: Params, key: str, default: bool = False) -> bool:
  try:
    return p.get_bool(key)
  except Exception:
    return default


def _get_float(p: Params, key: str, default: float, lo: float, hi: float) -> float:
  try:
    v = p.get(key, return_default=True)
    if v in (None, b"", ""):
      return default
    if isinstance(v, bytes):
      v = v.decode("utf-8", errors="replace").strip("\x00")
    return min(max(float(v), lo), hi)
  except Exception:
    return default


def _get_int(p: Params, key: str, default: int, lo: int, hi: int) -> int:
  try:
    v = p.get(key, return_default=True)
    if v in (None, b"", ""):
      return default
    if isinstance(v, bytes):
      v = v.decode("utf-8", errors="replace").strip("\x00")
    return min(max(int(float(v)), lo), hi)
  except Exception:
    return default


def read_ford_settings(params: Params) -> structs.FordSettingsBP:
  """Snapshot every Ford setting the car layer needs. Called from card.py's params thread.

  Ranges mirror the UI controls in selfdrive/ui/bp/layouts/settings/bluepilot.py so a value
  written out of band (SSH, athena, a restored backup) is clamped instead of reaching the
  control path.
  """
  s = structs.FordSettingsBP()

  # --- Lateral: curvature mode ---
  s.enableHumanTurnDetectionCurv = _get_bool(params, "enable_human_turn_detection_curv", True)
  s.laneChangeFactorHighCurv = _get_float(params, "lane_change_factor_high_curv", 0.85, 0.5, 1.0)
  s.pcBlendRatioHighCurv = _get_float(params, "pc_blend_ratio_high_C_UI_curv", 0.4, 0.0, 1.0)
  s.pcBlendRatioLowCurv = _get_float(params, "pc_blend_ratio_low_C_UI_curv", 0.4, 0.0, 1.0)
  s.enableLanePositioningCurv = _get_bool(params, "enable_lane_positioning_curv")
  s.customPathOffsetCurv = _get_float(params, "custom_path_offset_curv", 0.0, -0.5, 0.5)
  s.enableLaneFullModeCurv = _get_bool(params, "enable_lane_full_mode_curv")
  s.customProfileCurv = _get_int(params, "custom_profile_curv", 0, 0, 1)
  s.lcPidGainCurv = _get_float(params, "LC_PID_gain_UI_curv", 3.0, 0.0, 50.0)

  # --- Lateral: strategy select + angle mode ---
  s.primaryLateralControl = _get_int(params, "FordPrefLateralControl", 0, 0, 1)
  s.lowSpeedFactorAng = _get_float(params, "FordLowSpeedFactor_ang", 1.0, 0.5, 1.5)
  s.highSpeedFactorAng = _get_float(params, "FordHighSpeedFactor_ang", 1.0, 0.5, 1.5)
  s.laneChangeFactorHighAng = _get_float(params, "lane_change_factor_high_ang", 1.0, 0.85, 1.5)
  s.disableBpLat = _get_bool(params, "disable_BP_lat_UI")

  # --- HUD ---
  s.sendHandsFreeClusterMsg = _get_bool(params, "send_hands_free_cluster_msg")

  # --- Longitudinal ---
  s.disableBpLong = _get_bool(params, "disable_BP_long_UI")
  s.disableDownhillComp = _get_bool(params, "disable_downhill_comp_UI")
  s.coastingMode = _get_int(params, "FordPrefCoastingMode", 0, 0, 1)

  return s
