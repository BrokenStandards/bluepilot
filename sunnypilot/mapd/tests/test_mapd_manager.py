"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

BluePilot: the 1 Hz mapd loop asks "is there anything to clean up?" every tick. It used to
answer that by materialising the whole recursive glob of the OSM tree. has_files_for_cleanup()
answers the same question without building the list, so these tests pin the two to each other
across the states the loop can see.
"""
import os

import pytest

import openpilot.sunnypilot.mapd.mapd_manager as mapd_manager


@pytest.fixture
def mapd_root(tmp_path, mocker):
  root = tmp_path / "media" / "0" / "osm"
  root.mkdir(parents=True)
  mocker.patch.object(mapd_manager.Paths, "mapd_root", staticmethod(lambda: str(root)))
  return root


def build_tree(root, files):
  for rel in files:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("x")


class TestHasFilesForCleanup:
  @pytest.mark.parametrize("mapd_installed", [True, False])
  @pytest.mark.parametrize("tree", [
    [],                                             # nothing downloaded
    ["db/"],                                        # empty db directory
    ["db/index"],                                   # a single file
    ["db/a/b/c.bin", "db/a/d.bin", "db/e.bin"],     # a real nested tree
    ["v1.12.0/tiles/0.bin"],                        # versioned dir (never matched: v* is literal)
  ])
  def test_matches_the_list_it_replaces(self, mapd_root, mocker, tree, mapd_installed):
    for rel in tree:
      if rel.endswith("/"):
        (mapd_root / rel).mkdir(parents=True, exist_ok=True)
      else:
        build_tree(mapd_root, [rel])

    binary = mapd_root / "mapd"
    if mapd_installed:
      binary.write_text("#!/bin/sh\n")
    mocker.patch.object(mapd_manager, "MAPD_PATH", str(binary))

    assert mapd_manager.has_files_for_cleanup() == bool(mapd_manager.get_files_for_cleanup())

  @pytest.mark.parametrize("as_symlink", [False, True])
  def test_file_where_a_root_directory_belongs(self, mapd_root, mocker, as_symlink):
    # a partial download can leave db as a regular file; globbing it yields nothing, so
    # reporting cleanup work here would raise an offroad alert no download could clear
    if as_symlink:
      (mapd_root / "db_target").write_text("junk")
      os.symlink(str(mapd_root / "db_target"), str(mapd_root / "db"))
    else:
      (mapd_root / "db").write_text("junk")

    binary = mapd_root / "mapd"
    binary.write_text("#!/bin/sh\n")
    mocker.patch.object(mapd_manager, "MAPD_PATH", str(binary))

    assert mapd_manager.get_files_for_cleanup() == []
    assert mapd_manager.has_files_for_cleanup() is False

  def test_broken_symlink_root_is_ignored_by_both(self, mapd_root, mocker):
    os.symlink(str(mapd_root / "missing"), str(mapd_root / "db"))
    mocker.patch.object(mapd_manager, "MAPD_PATH", str(mapd_root / "mapd"))
    (mapd_root / "mapd").write_text("#!/bin/sh\n")

    # os.path.exists() is False for a broken link, so neither form looks inside it
    assert mapd_manager.get_files_for_cleanup() == []
    assert mapd_manager.has_files_for_cleanup() is False
