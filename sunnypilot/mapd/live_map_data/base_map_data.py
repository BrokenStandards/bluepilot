"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from abc import abstractmethod, ABC

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.navd.helpers import coordinate_from_param

MAX_SPEED_LIMIT = V_CRUISE_UNSET * CV.KPH_TO_MS


class BaseMapData(ABC):
  def __init__(self):
    self.params = Params()

    # BluePilot: controlsState provides desiredCurvature for the MapdCarContext mem param
    self.sm = messaging.SubMaster(['liveLocationKalman', 'controlsState'])
    # End BluePilot
    self.pm = messaging.PubMaster(['liveMapDataSP'])

    self.localizer_valid = False
    self.last_bearing = None
    self.last_position = coordinate_from_param("LastGPSPositionLLK", self.params)

  @abstractmethod
  def update_location(self) -> None:
    pass

  @abstractmethod
  def get_current_speed_limit(self) -> float:
    pass

  @abstractmethod
  def get_next_speed_limit_and_distance(self) -> tuple[float, float]:
    pass

  @abstractmethod
  def get_current_road_name(self) -> str:
    pass

  # BluePilot: continuity-based speed-limit guess for untagged ways; sources that
  # don't provide one keep the default "no guess"
  def get_speed_limit_guess(self) -> tuple[float, str]:
    return 0.0, ""
  # End BluePilot

  def publish(self) -> None:
    speed_limit = self.get_current_speed_limit()
    next_speed_limit, next_speed_limit_distance = self.get_next_speed_limit_and_distance()

    mapd_sp_send = messaging.new_message('liveMapDataSP')
    mapd_sp_send.valid = self.sm['liveLocationKalman'].gpsOK
    live_map_data = mapd_sp_send.liveMapDataSP

    # BluePilot: fall back to the mapd_bp guess when the way has no maxspeed tag.
    # A guessed limit publishes with speedLimitValid semantics unchanged so the
    # resolver/SLA/HUD keep working; speedLimitGuessed marks its provenance.
    speed_limit_guessed = False
    speed_limit_guessed_source = ""
    if speed_limit <= 0:
      guess_speed_limit, guess_source = self.get_speed_limit_guess()
      if guess_speed_limit > 0:
        speed_limit = guess_speed_limit
        speed_limit_guessed = True
        speed_limit_guessed_source = guess_source
    live_map_data.speedLimitGuessed = speed_limit_guessed
    live_map_data.speedLimitGuessedSource = speed_limit_guessed_source
    # End BluePilot

    live_map_data.speedLimitValid = bool(MAX_SPEED_LIMIT > speed_limit > 0)
    live_map_data.speedLimit = speed_limit
    live_map_data.speedLimitAheadValid = bool(MAX_SPEED_LIMIT > next_speed_limit > 0)
    live_map_data.speedLimitAhead = next_speed_limit
    live_map_data.speedLimitAheadDistance = next_speed_limit_distance
    live_map_data.roadName = self.get_current_road_name()

    self.pm.send('liveMapDataSP', mapd_sp_send)

  def tick(self) -> None:
    self.sm.update(0)
    self.update_location()
    self.publish()
