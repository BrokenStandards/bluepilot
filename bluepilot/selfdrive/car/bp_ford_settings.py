"""
BluePilot: Ford runtime settings snapshot, read on card.py's params thread.

opendbc is meant to be openpilot-independent. Every other brand takes its settings
through CarParamsSP at car init (opendbc/sunnypilot/car/interfaces.py), and settings that
must stay live while driving reach the car layer as derived state on carControlSP -- see
how MADS and ICBM do it. Ford was the only brand importing openpilot Params and reading
them directly inside the 100Hz CarController.update(), which put ~1500 file reads/s into
a Priority.CTRL_HIGH realtime process.

This module does the reading in openpilot land and card.py hands the result to the car
layer as structs.FordSettingsBP.

The reads are event-driven (see FordSettingsReader): in steady state we do one stat() per
poll and zero file reads, refreshing only when the params store actually changes. Settings
cannot normally be edited while moving anyway -- the UI forces the ONROAD layout on
transition (selfdrive/ui/layouts/main.py:_handle_onroad_transition), closing the settings
panel -- but the two writers that CAN fire mid-drive are still caught automatically:
the MICI onroad lateral-mode overlay tap, and sunnylink pushing settings over the network.

Values are validated/clamped here, at the boundary, so the car layer can trust the struct
and an out-of-band param write can never reach the control path with a bad value.
"""

import os

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


class FordSettingsReader:
  """Event-driven Ford settings snapshot.

  openpilot params are files, and Params.put() renames the new value into the params
  directory (common/params.cc: mkstemp -> write -> fsync -> rename -> fsync_dir). A rename
  into a directory bumps that directory's mtime, for a brand-new key and for an overwrite
  of an existing one alike, while plain reads leave it untouched. So watching a single
  mtime detects ANY param write, from any writer, with no cooperation required from the
  writers themselves -- the TICI settings menu, the MICI settings menu, the MICI onroad
  lateral-mode overlay tap, and sunnylink all get picked up automatically. Nothing can be
  silently missed by forgetting to signal a change.

  Steady state costs one stat() per poll and zero file reads. If the stat is unavailable
  the reader degrades to re-reading every poll, which is exactly the previous behavior, so
  this can only ever do less I/O than before -- never more.
  """

  def __init__(self, params: Params):
    self._params = params
    try:
      # getParamPath("") -> the directory the key files live in (common/params.h:54)
      self._params_dir: str | None = params.get_param_path("")
    except Exception:
      self._params_dir = None
    self.settings = read_ford_settings(params)
    self._mtime = self._read_mtime()

  def _read_mtime(self) -> int | None:
    if self._params_dir is None:
      return None
    try:
      return os.stat(self._params_dir).st_mtime_ns
    except OSError:
      return None

  def update(self) -> bool:
    """Refresh the snapshot only if the params store changed. True if it was re-read."""
    mtime = self._read_mtime()
    if mtime is not None and mtime == self._mtime:
      return False
    self._mtime = mtime
    self.settings = read_ford_settings(self._params)
    return True
