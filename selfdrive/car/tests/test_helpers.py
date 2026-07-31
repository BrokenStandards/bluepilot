import capnp
import hypothesis.strategies as st
from hypothesis import Phase, given, settings

from cereal import custom
from opendbc.car import structs
from openpilot.selfdrive.car.helpers import convert_carControlSP
from openpilot.selfdrive.test.fuzzy_generation import FuzzyGenerator

MAX_EXAMPLES = 200


def _reference_convert_carControlSP(struct: capnp.lib.capnp._DynamicStructReader) -> structs.CarControlSP:
  """The original to_dict-based conversion, kept as the behavioral reference."""
  def remove_deprecated(s: dict) -> dict:
    return {k: v for k, v in s.items() if not k.endswith('DEPRECATED')}

  struct_dict = struct.to_dict()
  struct_dataclass = structs.CarControlSP(**remove_deprecated({k: v for k, v in struct_dict.items() if not isinstance(k, dict)}))

  struct_dataclass.mads = structs.ModularAssistiveDrivingSystem(**remove_deprecated(struct_dict.get('mads', {})))
  struct_dataclass.leadOne = structs.LeadData(**remove_deprecated(struct_dict.get('leadOne', {})))
  struct_dataclass.leadTwo = structs.LeadData(**remove_deprecated(struct_dict.get('leadTwo', {})))
  struct_dataclass.intelligentCruiseButtonManagement = structs.IntelligentCruiseButtonManagement(
    **remove_deprecated(struct_dict.get('intelligentCruiseButtonManagement', {}))
  )
  return struct_dataclass


def _assert_dataclass_equal(a, b, path=""):
  for f in type(a).__dataclass_fields__:
    if f == 'params':
      # params has no publisher and no consumer; the reference conversion left it as
      # raw to_dict output (list of dicts) while the direct conversion leaves it at
      # its default ([]) -- checked separately in test_params_left_default
      continue
    va, vb = getattr(a, f), getattr(b, f)
    if hasattr(type(va), '__dataclass_fields__'):
      _assert_dataclass_equal(va, vb, f"{path}{f}.")
    else:
      assert type(va) is type(vb), f"{path}{f}: type {type(va)} != {type(vb)}"
      assert va == vb, f"{path}{f}: {va!r} != {vb!r}"


class TestConvertCarControlSP:
  @settings(max_examples=MAX_EXAMPLES, deadline=None, phases=(Phase.reuse, Phase.generate, Phase.shrink))
  @given(data=st.data())
  def test_matches_reference_conversion(self, data):
    cc_sp_msg = FuzzyGenerator.get_random_msg(data.draw, custom.CarControlSP, real_floats=True)
    reader = custom.CarControlSP.new_message(**cc_sp_msg).as_reader()
    _assert_dataclass_equal(_reference_convert_carControlSP(reader), convert_carControlSP(reader))

  def test_params_left_default(self):
    msg = custom.CarControlSP.new_message()
    msg.init('params', 2)
    assert convert_carControlSP(msg.as_reader()).params == []

  def test_convert_carControlSP_covers_schema(self):
    # every non-DEPRECATED field in the capnp schema must be read by the converter;
    # this fails when cereal/custom.capnp gains a field that helpers.py doesn't copy yet
    handled = {
      'mads': {'state', 'enabled', 'active', 'available'},
      'leadOne': set(structs.LeadData.__dataclass_fields__) - {'aLeadDEPRECATED'},
      'leadTwo': set(structs.LeadData.__dataclass_fields__) - {'aLeadDEPRECATED'},
      'intelligentCruiseButtonManagement': {'state', 'sendButton', 'vTarget'},
      'params': set(),  # intentionally not converted (no publisher/consumer)
    }
    schema_fields = {f for f in custom.CarControlSP.schema.non_union_fields if not f.endswith('DEPRECATED')}
    assert schema_fields == set(handled), f"schema/converter drift: {schema_fields ^ set(handled)}"
    for field, sub_handled in handled.items():
      if not sub_handled:
        continue
      sub_schema = custom.CarControlSP.schema.fields[field].proto.slot.type.struct
      sub_node = next(n for n in (custom.ModularAssistiveDrivingSystem, custom.LeadData, custom.IntelligentCruiseButtonManagement)
                      if n.schema.node.id == sub_schema.typeId)
      sub_fields = {f for f in sub_node.schema.non_union_fields if not f.endswith('DEPRECATED')}
      assert sub_fields == sub_handled, f"{field}: schema/converter drift: {sub_fields ^ sub_handled}"
