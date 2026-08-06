#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import argparse
import os

from openpilot.sunnypilot import get_file_hash
from openpilot.common.basedir import BASEDIR
from openpilot.sunnypilot.mapd import MAPD_BIN_DIR

MAPD_HASH_PATH = os.path.join(BASEDIR, "sunnypilot", "mapd", "tests", "mapd_hash")
# BluePilot: the fork layout keeps the version in the VERSION file shipped next to the
# committed binaries in third_party/mapd_bp (mapd_installer.py reads it from there too);
# there is no VERSION = "..." literal to rewrite anymore.
MAPD_VERSION_PATH = os.path.join(MAPD_BIN_DIR, "VERSION")

# MAPD_PATH is arch-dependent, so the hash file pins every committed binary keyed by name
MAPD_BIN_NAMES = ("mapd", "mapd-x86_64")


def update_mapd_hash():
  with open(MAPD_HASH_PATH, "w") as f:
    for bin_name in MAPD_BIN_NAMES:
      bin_hash = get_file_hash(os.path.join(MAPD_BIN_DIR, bin_name))
      f.write(f"{bin_name}:{bin_hash}\n")

  print(f"Generated and updated new mapd hashes to {MAPD_HASH_PATH}")


def get_current_mapd_version(path: str) -> str:
  print("[GET CURRENT MAPD VERSION]")
  try:
    with open(path) as f:
      ver = f.read().strip()
  except OSError:
    print(f"[ERROR] VERSION file not found at {path}!")
    return ""

  if not ver:
    print(f"[ERROR] VERSION file at {path} is empty!")
    return ""

  print(f'Current mapd version: "{ver}"')
  return ver


def update_mapd_version(ver: str, path: str):
  print("[CHANGE CURRENT MAPD VERSION]")

  with open(path, "w") as f:
    f.write(f"{ver}\n")

  print(f'New mapd version: "{ver}"')
  print("[DONE]")
# End BluePilot


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Update mapd version and hash")
  parser.add_argument("--new_ver", type=str, help="New mapd version")
  args = parser.parse_args()

  if not args.new_ver:
    print("Warning: No new mapd version provided. Use --new_ver to specify")
    print("Example:")
    print("  python sunnypilot/mapd/update_version.py --new_ver \"v1.12.0-bp1\"")
    print("Current mapd version and hash will not be updated! (aborted)")
    exit(0)

  current_ver = get_current_mapd_version(MAPD_VERSION_PATH)
  new_ver = f"{args.new_ver}"
  if current_ver == new_ver:
    print(f'Proposed mapd version: "{new_ver}"')
    confirm = input("Proposed mapd version is the same as the current mapd version. Confirm? (y/n): ").upper().strip()
    if confirm != "Y":
      print("Current mapd version and hash will not be updated! (aborted)")
      exit(0)

  update_mapd_version(new_ver, MAPD_VERSION_PATH)
  update_mapd_hash()
