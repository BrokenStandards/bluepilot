"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os

import pytest

from openpilot.sunnypilot import get_file_hash
from openpilot.sunnypilot.mapd import MAPD_BIN_DIR
from openpilot.sunnypilot.mapd.update_version import MAPD_HASH_PATH


class TestMapdVersion:
  def test_compare_versions(self):
    # BluePilot: both binaries (arm64 'mapd' and 'mapd-x86_64') are committed by the mapd_bp
    # build pipeline, so validate EVERY pinned entry present in the checkout — not just the
    # running arch's binary; x86 CI must also catch a stale committed arm64 binary.
    with open(MAPD_HASH_PATH) as f:
      pinned = dict(line.strip().split(":", 1) for line in f if line.strip())

    assert pinned, f"no pinned mapd hashes in {MAPD_HASH_PATH} - run sunnypilot/mapd/update_version.py"

    checked = 0
    for bin_name, pinned_hash in pinned.items():
      bin_path = os.path.join(MAPD_BIN_DIR, bin_name)
      if not os.path.exists(bin_path):
        # genuinely absent from this checkout (e.g. shallow/artifact-stripped) - nothing to compare
        continue
      assert get_file_hash(bin_path) == pinned_hash, \
        f"hash mismatch for {bin_path} - run sunnypilot/mapd/update_version.py to update the current mapd version and hash"
      checked += 1

    if checked == 0:
      pytest.skip(f"no committed mapd binaries present in {MAPD_BIN_DIR}")
    # End BluePilot
