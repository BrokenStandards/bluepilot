"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os

import pytest

from openpilot.sunnypilot import get_file_hash
from openpilot.sunnypilot.mapd import MAPD_BIN_NAME, MAPD_PATH
from openpilot.sunnypilot.mapd.update_version import MAPD_HASH_PATH


class TestMapdVersion:
  def test_compare_versions(self):
    # BluePilot: the binary is committed by the mapd_bp build pipeline; skip on checkouts without it
    if not os.path.exists(MAPD_PATH):
      pytest.skip(f"mapd binary not present at {MAPD_PATH}")
    # End BluePilot
    mapd_hash = get_file_hash(MAPD_PATH)

    with open(MAPD_HASH_PATH) as f:
      pinned = dict(line.strip().split(":", 1) for line in f if line.strip())

    assert pinned.get(MAPD_BIN_NAME) == mapd_hash, "Run sunnypilot/mapd/update_version.py to update the current mapd version and hash"
