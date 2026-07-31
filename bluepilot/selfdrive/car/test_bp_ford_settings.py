"""
Tests for the BluePilot Ford settings snapshot (bp_ford_settings.py).

Pins the two properties the car layer depends on:
  1. values are clamped at this boundary, so a bad param can never reach the control path
  2. the refresh is event-driven -- zero file reads while nothing changes, and a re-read
     after ANY param write, including writers that don't cooperate (the onroad overlay
     tap, sunnylink) and overwrites of an already-existing key

Run: python3 -m pytest -o addopts="" bluepilot/selfdrive/car/test_bp_ford_settings.py
"""

import os
import sys
import tempfile
import types


def _install_params_stub():
  """The Cython params module isn't built in all envs; bp_ford_settings only needs the name."""
  if "openpilot.common.params_pyx" not in sys.modules:
    m = types.ModuleType("openpilot.common.params_pyx")
    m.Params = type("Params", (), {})
    m.ParamKeyFlag = m.ParamKeyType = m.UnknownKeyName = object
    sys.modules["openpilot.common.params_pyx"] = m


_install_params_stub()

from openpilot.bluepilot.selfdrive.car.bp_ford_settings import FordSettingsReader, read_ford_settings


class FakeParams:
  """Mimics the openpilot Params surface bp_ford_settings uses, backed by a real directory.

  Writes go through the same mkstemp -> rename dance as common/params.cc:put(), because
  that rename is precisely what the reader's mtime watch keys off.
  """

  def __init__(self, root, values=None):
    self._root = root
    self._dir = os.path.join(root, "d")
    os.makedirs(self._dir, exist_ok=True)
    self.reads = 0
    for k, v in (values or {}).items():
      self.put(k, v)

  def get_param_path(self, key=""):
    return self._dir if key == "" else os.path.join(self._dir, key)

  def put(self, key, value):
    fd, tmp = tempfile.mkstemp(dir=self._root, prefix=".tmp_value_")
    os.write(fd, str(value).encode())
    os.close(fd)
    os.rename(tmp, os.path.join(self._dir, key))

  def get(self, key, return_default=False):
    self.reads += 1
    try:
      with open(os.path.join(self._dir, key), "rb") as f:
        return f.read().decode()
    except OSError:
      return None

  def get_bool(self, key):
    return self.get(key) == "1"


def test_out_of_range_values_are_clamped():
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d, {
      "FordPrefCoastingMode": 99,
      "FordPrefLateralControl": -5,
      "LC_PID_gain_UI_curv": 9999,
      "FordLowSpeedFactor_ang": 0.0,
      "lane_change_factor_high_ang": 3.0,
      "custom_path_offset_curv": 5.0,
    })
    s = read_ford_settings(p)
    assert s.coastingMode == 1
    assert s.primaryLateralControl == 0
    assert s.lcPidGainCurv == 50.0
    assert s.lowSpeedFactorAng == 0.5
    assert s.laneChangeFactorHighAng == 1.5
    assert s.customPathOffsetCurv == 0.5


def test_garbage_falls_back_to_default():
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d, {"custom_path_offset_curv": "not-a-number", "FordPrefCoastingMode": "junk"})
    s = read_ford_settings(p)
    assert s.customPathOffsetCurv == 0.0
    assert s.coastingMode == 0


def test_empty_store_matches_params_keys_defaults():
  with tempfile.TemporaryDirectory() as d:
    s = read_ford_settings(FakeParams(d))
    assert s.laneChangeFactorHighCurv == 0.85
    assert s.pcBlendRatioHighCurv == 0.4
    assert s.lcPidGainCurv == 3.0
    assert s.coastingMode == 0
    assert not s.disableBpLat


def test_no_reads_while_nothing_changes():
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d, {"FordPrefCoastingMode": 0})
    r = FordSettingsReader(p)
    p.reads = 0  # ignore the constructor's initial read
    for _ in range(50):
      assert not r.update()
    assert p.reads == 0


def test_new_key_triggers_refresh():
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d, {"FordPrefCoastingMode": 0})
    r = FordSettingsReader(p)
    assert r.settings.coastingMode == 0
    p.put("SomeUnrelatedKey", 1)
    assert r.update()


def test_overwrite_of_existing_key_triggers_refresh():
  # the case that matters: the onroad overlay tap rewrites a key that already exists
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d, {"FordPrefLateralControl": 0})
    r = FordSettingsReader(p)
    assert r.settings.primaryLateralControl == 0
    p.put("FordPrefLateralControl", 1)
    assert r.update()
    assert r.settings.primaryLateralControl == 1
    # and it settles again afterwards
    p.reads = 0
    assert not r.update()
    assert p.reads == 0


def test_uncooperative_writer_is_caught():
  # sunnylink / SSH / athena write params without signalling anyone
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d, {"disable_BP_long_UI": 0})
    r = FordSettingsReader(p)
    assert not r.settings.disableBpLong
    p.put("disable_BP_long_UI", 1)
    assert r.update()
    assert r.settings.disableBpLong


def test_degrades_to_always_refresh_without_a_stat():
  # if the params dir can't be stat'd we must fall back to re-reading, never to stalling
  with tempfile.TemporaryDirectory() as d:
    r = FordSettingsReader(FakeParams(d))
    r._params_dir = None
    for _ in range(3):
      assert r.update()
