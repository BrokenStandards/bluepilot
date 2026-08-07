"""Every default in params_keys.h must survive manager's default seeding.

manager.py does this to every registered key before anything else starts:

    default_value = params.get_default_value(k)      # bytes -> python, via CPP_2_PYTHON
    if default_value is not None and params.get(k) is None:
      params.put(k, default_value, block=True)       # python -> bytes, via PYTHON_2_CPP

The two tables are not inverses. CPP_2_PYTHON maps JSON through json.loads, which happily
returns a float for a scalar default, but PYTHON_2_CPP only knows how to write dict and list
into a JSON param - so a JSON key with a scalar default raises TypeError inside manager and
the device does not boot. A JSON key with no default is fine; so is any scalar default on a
STRING/BOOL/INT/FLOAT/TIME key.

This reads the header rather than Params.all_keys() on purpose: the compiled key registry in
a checkout usually predates the header being edited, which is exactly when a new key needs
checking. Nothing here touches the real param store.
"""
import re
import pathlib

import pytest

from openpilot.common.basedir import BASEDIR
from openpilot.common.params_pyx import PYTHON_2_CPP, CPP_2_PYTHON

PARAMS_KEYS_H = pathlib.Path(BASEDIR) / "common" / "params_keys.h"

TYPE_VALUES = {"STRING": 0, "BOOL": 1, "INT": 2, "FLOAT": 3, "TIME": 4, "JSON": 5, "BYTES": 6}

# {"Key", {FLAGS | FLAGS, TYPE, "default"}},  — the default is optional
ENTRY = re.compile(
  r'\{\s*"(?P<key>[A-Za-z0-9_]+)"\s*,\s*\{(?P<flags>[^,{}]+),\s*(?P<type>[A-Z]+)\s*(?:,\s*"(?P<default>[^"]*)")?\s*\}\s*\}'
)


def registered_keys():
  keys = [m.groupdict() for m in ENTRY.finditer(PARAMS_KEYS_H.read_text())]
  assert len(keys) > 100, f"parsed only {len(keys)} keys from {PARAMS_KEYS_H} - has the format changed?"
  return keys


def keys_with_defaults():
  return [k for k in registered_keys() if k["default"] is not None]


class TestParamsKeyDefaults:
  def test_every_type_name_is_known(self):
    unknown = {k["type"] for k in registered_keys()} - set(TYPE_VALUES)
    assert not unknown, f"params_keys.h uses type names this test does not know about: {unknown}"

  @pytest.mark.parametrize("entry", keys_with_defaults(), ids=lambda e: e["key"])
  def test_default_survives_manager_seeding(self, entry):
    key, type_value, default = entry["key"], TYPE_VALUES[entry["type"]], entry["default"]

    # manager: bytes -> python
    to_python = CPP_2_PYTHON.get(type_value)
    assert to_python is not None, f"{key}: no CPP_2_PYTHON entry for {entry['type']}"
    value = to_python(default.encode())

    # manager: python -> bytes, the step that kills the boot when the pair is missing
    why = " ".join([
      f"{key} is declared {entry['type']} with default {default!r}, which reads back as",
      f"{type(value).__name__} - a type Params.put() cannot write to a {entry['type']} param.",
      "manager.py seeds unset defaults on every boot, so this key would raise TypeError before",
      "any process starts. Declare a type whose default round-trips (FLOAT for a bare number),",
      "or drop the default.",
    ])
    assert (type(value), type_value) in PYTHON_2_CPP, why

  @pytest.mark.parametrize("entry", keys_with_defaults(), ids=lambda e: e["key"])
  def test_default_round_trips_to_an_equal_value(self, entry):
    """The seeded value must also mean the same thing after the round trip."""
    key, type_value, default = entry["key"], TYPE_VALUES[entry["type"]], entry["default"]
    if (type(CPP_2_PYTHON[type_value](default.encode())), type_value) not in PYTHON_2_CPP:
      pytest.skip("covered by test_default_survives_manager_seeding")

    value = CPP_2_PYTHON[type_value](default.encode())
    written = PYTHON_2_CPP[(type(value), type_value)](value)
    reread = CPP_2_PYTHON[type_value](written.encode() if isinstance(written, str) else written)
    assert reread == value, f"{key}: default {default!r} becomes {reread!r} after one seeding round trip"
