import capnp
from typing import Any

from cereal import custom
from opendbc.car import structs

_FIELDS = '__dataclass_fields__'  # copy of dataclasses._FIELDS


def is_dataclass(obj):
  """Similar to dataclasses.is_dataclass without instance type check checking"""
  return hasattr(obj, _FIELDS)


def _asdictref_inner(obj) -> dict[str, Any] | Any:
  if is_dataclass(obj):
    ret = {}
    for field in getattr(obj, _FIELDS):  # similar to dataclasses.fields()
      ret[field] = _asdictref_inner(getattr(obj, field))
    return ret
  elif isinstance(obj, (tuple, list)):
    return type(obj)(_asdictref_inner(v) for v in obj)
  else:
    return obj


def asdictref(obj) -> dict[str, Any]:
  """
  Similar to dataclasses.asdict without recursive type checking and copy.deepcopy
  Note that the resulting dict will contain references to the original struct as a result
  """
  if not is_dataclass(obj):
    raise TypeError("asdictref() should be called on dataclass instances")

  return _asdictref_inner(obj)


def convert_to_capnp(struct: structs.CarParamsSP | structs.CarStateSP | structs.ControllerStateBP) -> capnp.lib.capnp._DynamicStructBuilder:
  struct_dict = asdictref(struct)

  if isinstance(struct, structs.CarParamsSP):
    struct_capnp = custom.CarParamsSP.new_message(**struct_dict)
  elif isinstance(struct, structs.CarStateSP):
    struct_capnp = custom.CarStateSP.new_message(**struct_dict)
  elif isinstance(struct, structs.ControllerStateBP):  # BluePilot: controllerStateBP (lateral uncertainty)
    struct_capnp = custom.ControllerStateBP.new_message(**struct_dict)
  else:
    raise ValueError(f"Unsupported struct type: {type(struct)}")

  return struct_capnp


def _convert_lead_data(src: capnp.lib.capnp._DynamicStructReader) -> structs.LeadData:
  return structs.LeadData(
    dRel=src.dRel, yRel=src.yRel, vRel=src.vRel, aRel=src.aRel,
    vLead=src.vLead, dPath=src.dPath, vLat=src.vLat,
    vLeadK=src.vLeadK, aLeadK=src.aLeadK, fcw=src.fcw,
    status=src.status, aLeadTau=src.aLeadTau, modelProb=src.modelProb,
    radar=src.radar, radarTrackId=src.radarTrackId,
  )


def convert_carControlSP(struct: capnp.lib.capnp._DynamicStructReader) -> structs.CarControlSP:
  # Direct field reads instead of struct.to_dict(): ~2.5x faster on card's 100Hz path.
  # to_dict() recursively dictifies the whole message (including DEPRECATED fields and
  # the unused params list) only for most of it to be thrown away again.
  # Equivalence vs the old to_dict-based conversion, for a POPULATED message: identical
  # (enums become plain strings, which compare equal to the StrEnum fields; DEPRECATED
  # fields keep their dataclass defaults; params stays [] -- it has no publisher or
  # consumer, and the old code never converted it either). For an UNSET sub-struct
  # (reachable only via SubMaster's default message before the first carControlSP
  # arrives -- card.py gates on all_alive(['carControl']) only), this yields capnp
  # SCHEMA defaults where the old code accidentally yielded dataclass defaults:
  # concretely leadOne/leadTwo.radarTrackId is -1 instead of 0, and enum fields are
  # plain str instead of StrEnum members. No consumer distinguishes these.
  # NOTE: when a new field is added to CarControlSP in cereal/custom.capnp it must be
  # added here too; test_convert_carControlSP_covers_schema pins this.
  mads = struct.mads
  icbm = struct.intelligentCruiseButtonManagement
  return structs.CarControlSP(
    mads=structs.ModularAssistiveDrivingSystem(
      state=str(mads.state),
      enabled=mads.enabled,
      active=mads.active,
      available=mads.available,
    ),
    leadOne=_convert_lead_data(struct.leadOne),
    leadTwo=_convert_lead_data(struct.leadTwo),
    intelligentCruiseButtonManagement=structs.IntelligentCruiseButtonManagement(
      state=str(icbm.state),
      sendButton=str(icbm.sendButton),
      vTarget=icbm.vTarget,
    ),
  )
