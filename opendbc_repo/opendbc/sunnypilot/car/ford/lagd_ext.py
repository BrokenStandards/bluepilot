"""
BluePilot Ford: per-mode manual steer actuator delay.

Curvature-primary and angle-primary lateral control drive the PSCM through different
DBC signal paths (LatCtl_D_Rq vs LatCtlPath_An_Actl / path_angle geometry), and can
have genuinely different actuator delay even on the same physical EPS hardware.
lagd's live estimator is mode-blind (it correlates planner curvature against measured
yaw rate, regardless of which signal produced it) and its persisted average carries
straight through a mode switch, so a single learned value can't be trusted to represent
both modes. This only matters when live learning is disabled (LagdToggle off) and a
fixed, user-tuned delay is used instead -- see sunnypilot/livedelay/lagd_toggle.py.
"""

from opendbc.car import structs
from openpilot.common.params import Params
from opendbc.sunnypilot.car.ford.lateral_curv_ext import PrimaryLateralControl


def get_manual_steer_delay(CP: structs.CarParams, params: Params, default: float) -> float:
  mode = PrimaryLateralControl(params.get("FordPrefLateralControl", return_default=True) or 0)
  key = "FordSteerActDelayAng" if mode == PrimaryLateralControl.angle else "FordSteerActDelayCurv"
  try:
    return float(params.get(key, return_default=True))
  except (TypeError, ValueError):
    return default
