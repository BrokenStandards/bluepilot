from dataclasses import dataclass as _dataclass, field, is_dataclass
from enum import Enum, StrEnum as _StrEnum, auto
from typing import dataclass_transform, get_origin

import os
import capnp
from opendbc.car.common.basedir import BASEDIR

# TODO: remove car from cereal/__init__.py and always import from opendbc
try:
  from cereal import car
except ImportError:
  capnp.remove_import_hook()
  car = capnp.load(os.path.join(BASEDIR, "car.capnp"))

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

# sunnypilot structs

AUTO_OBJ = object()


def auto_field():
  return AUTO_OBJ


@dataclass_transform()
def auto_dataclass(cls=None, /, **kwargs):
  cls_annotations = cls.__dict__.get('__annotations__', {})
  for name, typ in cls_annotations.items():
    current_value = getattr(cls, name)
    if current_value is AUTO_OBJ:
      origin_typ = get_origin(typ) or typ
      if isinstance(origin_typ, str):
        raise TypeError(f"Forward references are not supported for auto_field: '{origin_typ}'. Use a default_factory with lambda instead.")
      elif origin_typ in (int, float, str, bytes, list, tuple, bool) or is_dataclass(origin_typ):
        setattr(cls, name, field(default_factory=origin_typ))
      elif issubclass(origin_typ, Enum):  # first enum is the default
        setattr(cls, name, field(default=next(iter(origin_typ))))
      else:
        raise TypeError(f"Unsupported type for auto_field: {origin_typ}")

  # TODO: use slots, this prevents accidentally setting attributes that don't exist
  return _dataclass(cls, **kwargs)


class StrEnum(_StrEnum):
  @staticmethod
  def _generate_next_value_(name, *args):
    # auto() defaults to name.lower()
    return name


@auto_dataclass
class CarParamsSP:
  flags: int = auto_field()        # flags for car specific quirks
  safetyParam: int = auto_field()  # flags for custom safety flags
  pcmCruiseSpeed: bool = auto_field()
  intelligentCruiseButtonManagementAvailable: bool = auto_field()
  enableGasInterceptor: bool = auto_field()

  neuralNetworkLateralControl: 'CarParamsSP.NeuralNetworkLateralControl' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl())

  @auto_dataclass
  class NeuralNetworkLateralControl:
    model: 'CarParamsSP.NeuralNetworkLateralControl.Model' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl.Model())
    fuzzyFingerprint: bool = auto_field()

    @auto_dataclass
    class Model:
      path: str = auto_field()
      name: str = auto_field()


@auto_dataclass
class ModularAssistiveDrivingSystem:
  state: 'ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState' = field(
    default_factory=lambda: ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.disabled
  )
  enabled: bool = auto_field()
  active: bool = auto_field()
  available: bool = auto_field()

  class ModularAssistiveDrivingSystemState(StrEnum):
    disabled = auto()
    paused = auto()
    enabled = auto()
    softDisabling = auto()
    overriding = auto()


@auto_dataclass
class IntelligentCruiseButtonManagement:
  state: 'IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.inactive
  )
  sendButton: 'IntelligentCruiseButtonManagement.SendButtonState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.SendButtonState.none
  )
  vTarget: float = auto_field()

  class IntelligentCruiseButtonManagementState(StrEnum):
    inactive = auto()
    preActive = auto()
    increasing = auto()
    decreasing = auto()
    holding = auto()

  class SendButtonState(StrEnum):
    none = auto()
    increase = auto()
    decrease = auto()


@auto_dataclass
class LeadData:
  dRel: float = auto_field()
  yRel: float = auto_field()
  vRel: float = auto_field()
  aRel: float = auto_field()
  vLead: float = auto_field()
  dPath: float = auto_field()
  vLat: float = auto_field()
  vLeadK: float = auto_field()
  aLeadK: float = auto_field()
  fcw: bool = auto_field()
  status: bool = auto_field()
  aLeadTau: float = auto_field()
  modelProb: float = auto_field()
  radar: bool = auto_field()
  radarTrackId: int = auto_field()

  aLeadDEPRECATED: float = auto_field()


@auto_dataclass
class CarControlSP:
  mads: 'ModularAssistiveDrivingSystem' = field(default_factory=lambda: ModularAssistiveDrivingSystem())
  params: list['CarControlSP.Param'] = auto_field()
  leadOne: 'LeadData' = field(default_factory=lambda: LeadData())
  leadTwo: 'LeadData' = field(default_factory=lambda: LeadData())
  intelligentCruiseButtonManagement: 'IntelligentCruiseButtonManagement' = field(default_factory=lambda: IntelligentCruiseButtonManagement())
  # BluePilot: Ford settings snapshot, attached by card.py (see FordSettingsBP below).
  # Not part of the carControlSP capnp schema -- it never crosses a socket, so the wire
  # format stays identical to upstream. convert_carControlSP() leaves it at its default
  # and card.py overwrites it before CI.apply().
  fordSettingsBP: 'FordSettingsBP' = field(default_factory=lambda: FordSettingsBP())

  @auto_dataclass
  class Param:
    key: str = auto_field()
    value: bytes = auto_field()
    type: 'CarControlSP.ParamType' = field(
      default_factory=lambda: CarControlSP.ParamType.string
    )

  class ParamType(StrEnum):
    string = auto()
    bool = auto()
    int = auto()
    float = auto()
    time = auto()
    json = auto()
    bytes = auto()


@auto_dataclass
class CarStateSP:
  speedLimit: float = auto_field()


# BluePilot: Ford runtime settings snapshot.
#
# opendbc is meant to be openpilot-independent: every other brand takes its settings
# through CarParamsSP at car init (opendbc/sunnypilot/car/interfaces.py), and settings
# that must stay live reach the car layer as derived state on carControlSP (see how MADS
# and ICBM do it). Ford was the only brand reading openpilot Params directly inside the
# 100Hz CarController.update(), which put ~1500 file reads/s in a realtime process.
#
# These are read on card.py's 10Hz params thread (bluepilot/selfdrive/car/bp_ford_settings.py)
# and handed to the car layer as this struct, so the control tick does no file I/O.
# Defaults MUST match common/params_keys.h so an unpopulated snapshot behaves like a
# fresh install.
@auto_dataclass
class FordSettingsBP:
  # --- Lateral: curvature mode ---
  enableHumanTurnDetectionCurv: bool = True   # enable_human_turn_detection_curv
  laneChangeFactorHighCurv: float = 0.85      # lane_change_factor_high_curv
  pcBlendRatioHighCurv: float = 0.4           # pc_blend_ratio_high_C_UI_curv
  pcBlendRatioLowCurv: float = 0.4            # pc_blend_ratio_low_C_UI_curv
  enableLanePositioningCurv: bool = False     # enable_lane_positioning_curv
  customPathOffsetCurv: float = 0.0           # custom_path_offset_curv
  enableLaneFullModeCurv: bool = False        # enable_lane_full_mode_curv
  customProfileCurv: int = 0                  # custom_profile_curv
  lcPidGainCurv: float = 3.0                  # LC_PID_gain_UI_curv
  # --- Lateral: strategy select + angle mode ---
  primaryLateralControl: int = 0              # FordPrefLateralControl: 0=curvature, 1=angle
  lowSpeedFactorAng: float = 1.0              # FordLowSpeedFactor_ang
  highSpeedFactorAng: float = 1.0             # FordHighSpeedFactor_ang
  laneChangeFactorHighAng: float = 1.0        # lane_change_factor_high_ang
  disableBpLat: bool = False                  # disable_BP_lat_UI
  # --- HUD ---
  sendHandsFreeClusterMsg: bool = False       # send_hands_free_cluster_msg
  # --- Longitudinal ---
  disableBpLong: bool = False                 # disable_BP_long_UI
  disableDownhillComp: bool = False           # disable_downhill_comp_UI
  coastingMode: int = 0                       # FordPrefCoastingMode: 0=legacy, 1=extended


# BluePilot: ControllerStateBP for lateral uncertainty (angleState vehicles)
@auto_dataclass
class ControllerStateBP:
  lateralUncertainty: float = 0.0
  angleRateLimited: bool = False       # angle mode: path_angle soft-ROC clip bit this frame
  curvatureRateLimited: bool = False   # sim: equivalent curvature would be rate-limited by lateral_curv_ext
  curvatureDeviationLimited: bool = False  # current_curvature error-clip constrained the command this frame
  humanTurnLateralPaused: bool = False  # angle mode: lateral forced inactive (mode 0) during a manual turn
  stallBlipActive: bool = False  # angle mode: brief mode-0 pulse resetting PSCM authority after a post-override stall

  # BluePilot: full BluePilot-menu settings snapshot -- see custom.capnp ControllerStateBP for
  # field-by-field param-key mapping and the field-retirement convention.
  # --- System ---
  bmsUiDebugLogging: bool = False
  bmsConnectBackend: int = 0
  bmsWebRoutesServerEnabled: bool = False
  bmsPreferredWifiNetwork: str = ""
  # --- Vehicle ---
  bmsShowBlueCruiseUiOnCluster: bool = False
  bmsTwelveVBatteryLimit: float = 11.8
  # --- Visuals ---
  bmsHideOnroadBorder: bool = False
  bmsDisableLaneLineStatusColor: bool = False
  bmsMinimalDrivingView: bool = False
  bmsEightBitRacerTheme: bool = False
  bmsRainbowLaneLines: bool = False
  bmsShowBlindspotOverlay: bool = True
  bmsShowBrakeStatus: bool = False
  bmsShowConfidenceBall: bool = True
  bmsAnimateSteeringWheel: bool = True
  bmsWheelIconStyle: int = 0
  bmsShowRadarLeadOverlay: bool = True
  bmsRadarOverlaySize: int = 1
  bmsShowHybridBatteryStatus: bool = False
  bmsShowHybridPowerFlow: bool = False
  bmsHybridDriveGaugeSize: int = 1
  bmsHybridGaugeStyle: int = 0
  bmsHybridPowerFlowStyleRound: bool = False
  bmsLowerRightDisplay: int = 0
  bmsRainbowMode: bool = False
  bmsHideOnroadFade: bool = False
  # --- Longitudinal Tuning ---
  bmsBypassBpLongitudinalControl: bool = False
  bmsDisableDownhillCompensation: bool = False
  bmsDisableFordRadarVisionOnly: bool = False
  # --- Lateral Tuning ---
  bmsDisableBpLateralControl: bool = False
  bmsPrimaryControlVariable: int = 0
  bmsDisableLaneChangeUnderSpeed: bool = False
  bmsMinimumSpeedToPauseLaneChange: int = 20
  bmsShowLateralControlMode: bool = False
  # --- Angle Tuning ---
  bmsLowSpeedAdjustmentFactor: float = 1.0
  bmsHighSpeedAdjustmentFactor: float = 1.0
  bmsLaneChangeFactorHighAngle: float = 1.0
  # --- Curvature Tuning ---
  bmsEnableHumanTurnDetection: bool = True
  bmsLaneChangeFactorHighCurvature: float = 0.85
  bmsEnableLanePositioning: bool = False
  bmsInLaneOffset: float = 0.0
  bmsEnableLanefullMode: bool = False
  bmsUseCustomTuningProfile: bool = False
  bmsPredictedCurvatureBlendRatioHigh: float = 0.4
  bmsPredictedCurvatureBlendRatioLow: float = 0.4
  bmsCenteringPidGain: float = 3.0
  # --- Fingerprint ---
  bmsFingerprintForced: bool = False
  bmsFingerprint: str = ""
