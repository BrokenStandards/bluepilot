#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import logging
import os
import stat
import time
import traceback
import requests
from pathlib import Path
from urllib.request import urlopen

from cereal import messaging
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths
from openpilot.common.spinner import Spinner
from openpilot.system.version import is_prebuilt
from openpilot.sunnypilot.mapd import MAPD_PATH, MAPD_BIN_DIR
import openpilot.system.sentry as sentry

# BluePilot: the committed binaries in third_party/mapd_bp are authoritative; VERSION is read
# from the VERSION file shipped next to them. The pfeiferj release download below is kept ONLY
# as a fallback for when the committed binary is missing — it serves stock v1.12.0 WITHOUT the
# BP fixes (U-turn rejection, lane-count-gated flattening, speed-limit guess, divergence rematch).
FALLBACK_VERSION = "v1.12.0"
URL = f"https://github.com/pfeiferj/openpilot-mapd/releases/download/{FALLBACK_VERSION}/mapd"
VERSION_PATH = os.path.join(MAPD_BIN_DIR, 'VERSION')


def _read_committed_version() -> str:
  try:
    with open(VERSION_PATH) as f:
      version = f.read().strip()
      if version:
        return version
  except OSError:
    pass
  return FALLBACK_VERSION


VERSION = _read_committed_version()
# End BluePilot


def update_installed_version(version: str, params: Params = None) -> None:
  if params is None:
    params = Params()

  params.put("MapdVersion", version, block=True)


class MapdInstallManager:
  def __init__(self, spinner_ref: Spinner):
    self._spinner = spinner_ref
    self._params = Params()

  def download(self) -> None:
    self.ensure_directories_exist()
    self._download_file()
    # BluePilot: the fallback download serves the stock pfeiferj release, so record its version
    update_installed_version(FALLBACK_VERSION, self._params)
    # End BluePilot

  def check_and_download(self) -> None:
    if self.download_needed():
      self.download()
    # BluePilot: committed binary present — never download, only sync the installed-version param
    else:
      update_installed_version(VERSION, self._params)
    # End BluePilot

  def download_needed(self) -> bool:
    # BluePilot: the committed binary is authoritative; download only when it is missing
    return not os.path.exists(MAPD_PATH)
    # End BluePilot

  @staticmethod
  def ensure_directories_exist() -> None:
    if not os.path.exists(Paths.mapd_root()):
      os.makedirs(Paths.mapd_root())
    if not os.path.exists(MAPD_BIN_DIR):
      os.makedirs(MAPD_BIN_DIR)

  @staticmethod
  def _safe_write_and_set_executable(file_path: Path, content: bytes) -> None:
    with open(file_path, 'wb') as output:
      output.write(content)
      output.flush()
      os.fsync(output.fileno())
    current_permissions = stat.S_IMODE(os.lstat(file_path).st_mode)
    os.chmod(file_path, current_permissions | stat.S_IEXEC)

  def _download_file(self, num_retries=5) -> None:
    temp_file = Path(MAPD_PATH + ".tmp")
    download_timeout = 60
    for cnt in range(num_retries):
      try:
        response = requests.get(URL, stream=True, timeout=download_timeout)
        response.raise_for_status()
        self._safe_write_and_set_executable(temp_file, response.content)
        # No exceptions encountered. Safe to replace original file.
        temp_file.replace(MAPD_PATH)
        return
      except requests.exceptions.ReadTimeout:
        self._spinner.update(f"ReadTimeout caught. Timeout is [{download_timeout}]. Retrying download... [{cnt}]")
        time.sleep(0.5)
      except requests.exceptions.RequestException as e:
        self._spinner.update(f"RequestException caught: {e}. Retrying download... [{cnt}]")
        time.sleep(0.5)

    # Delete temp file if the process was not successful.
    if temp_file.exists():
      temp_file.unlink()
    logging.error("Failed to download file after all retries")

  def get_installed_version(self) -> str:
    return str(self._params.get("MapdVersion") or "")

  def wait_for_internet_connection(self, return_on_failure: bool = False) -> bool:
    max_retries = 10
    for retries in range(max_retries + 1):
      self._spinner.update(f"Waiting for internet connection... [{retries}/{max_retries}]")
      time.sleep(2)
      try:
        _ = urlopen('https://sentry.io', timeout=10)
        return True
      except Exception as e:
        print(f'Wait for internet failed: {e}')
        if return_on_failure and retries == max_retries:
          return False

    return False

  def non_prebuilt_install(self) -> None:
    sm = messaging.SubMaster(['deviceState'])
    metered = sm['deviceState'].networkMetered

    if metered:
      self._spinner.update("Can't proceed with mapd install since network is metered!")
      time.sleep(5)
      return

    try:
      self.ensure_directories_exist()
      if not self.download_needed():
        self._spinner.update("Mapd is good!")
        time.sleep(0.1)
        return

      if self.wait_for_internet_connection(return_on_failure=True):
        # BluePilot: fallback path — committed binary missing, fetch stock pfeiferj release
        self._spinner.update(f"Downloading pfeiferj's mapd [{self.get_installed_version()}] => [{FALLBACK_VERSION}].")
        # End BluePilot
        time.sleep(0.1)
        self.check_and_download()
      self._spinner.close()

    except Exception:
      for i in range(6):
        self._spinner.update("Failed to download OSM maps won't work until properly downloaded!" +
                             "Try again manually rebooting. " +
                             f"Boot will continue in {5 - i}s...")
        time.sleep(1)

      sentry.init(sentry.SentryProject.SELFDRIVE)
      traceback.print_exc()
      sentry.capture_exception()


if __name__ == "__main__":
  spinner = Spinner()
  install_manager = MapdInstallManager(spinner)
  install_manager.ensure_directories_exist()
  if is_prebuilt():
    debug_msg = f"[DEBUG] This is prebuilt, no mapd install required. VERSION: [{VERSION}], Param [{install_manager.get_installed_version()}]"
    spinner.update(debug_msg)
    update_installed_version(VERSION)
  else:
    spinner.update(f"Checking if mapd is installed and valid. Prebuilt [{is_prebuilt()}]")
    install_manager.non_prebuilt_install()
