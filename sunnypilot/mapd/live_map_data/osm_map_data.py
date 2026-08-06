"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import platform

from cereal import log
from openpilot.common.params import Params
from openpilot.sunnypilot.mapd.live_map_data.base_map_data import BaseMapData
from openpilot.sunnypilot.navd.helpers import Coordinate

# BluePilot: tick() runs at 1 Hz, so 5 ticks ~= 5 s between VisualRoutingAssist param reads
PARAM_REFRESH_TICKS = 5
# End BluePilot


class OsmMapData(BaseMapData):
  def __init__(self):
    super().__init__()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params

    # BluePilot: Visual Routing Assistance toggle, cached and refreshed at ~5 s cadence
    self._frame = 0
    self._visual_routing_assist = self.params.get_bool("VisualRoutingAssist")
    # End BluePilot

  # BluePilot: 1 Hz car context for the mapd_bp divergence-based rematch.
  # mapd only uses it when enabled (VisualRoutingAssist); the kinematic fields are
  # published regardless so toggling the param doesn't need a mapd restart.
  def _update_car_context(self) -> None:
    if self._frame % PARAM_REFRESH_TICKS == 0:
      self._visual_routing_assist = self.params.get_bool("VisualRoutingAssist")
    self._frame += 1

    location = self.sm['liveLocationKalman']
    v_ego = float(location.velocityCalibrated.value[0]) if len(location.velocityCalibrated.value) else 0.0
    yaw_rate = float(location.angularVelocityCalibrated.value[2]) if len(location.angularVelocityCalibrated.value) > 2 else 0.0
    desired_curvature = float(self.sm['controlsState'].desiredCurvature) if self.sm.seen['controlsState'] else 0.0

    self.mem_params.put("MapdCarContext", {
      "enabled": self._visual_routing_assist,
      "v_ego": v_ego,
      "yaw_rate": yaw_rate,
      # curvature = |yaw_rate| / max(v_ego, 1), signed by yaw_rate; the 1 m/s floor
      # keeps the ratio bounded at parking speeds
      "curvature": yaw_rate / max(v_ego, 1.0),
      "desired_curvature": desired_curvature,
    }, block=True)
  # End BluePilot

  def update_location(self) -> None:
    # BluePilot: publish car context for mapd_bp
    self._update_car_context()
    # End BluePilot

    location = self.sm['liveLocationKalman']
    self.localizer_valid = (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid

    if self.localizer_valid:
      self.last_bearing = math.degrees(location.calibratedOrientationNED.value[2])
      self.last_position = Coordinate(location.positionGeodetic.value[0], location.positionGeodetic.value[1])

    if self.last_position is None:
      return

    params = {
      "latitude": self.last_position.latitude,
      "longitude": self.last_position.longitude,
    }

    if self.last_bearing is not None:
      params['bearing'] = self.last_bearing

    self.mem_params.put("LastGPSPosition", json.dumps(params), block=True)

  def get_current_speed_limit(self) -> float:
    return float(self.mem_params.get("MapSpeedLimit") or 0.0)

  # BluePilot: mapd_bp writes its continuity-based guess for untagged ways here
  def get_speed_limit_guess(self) -> tuple[float, str]:
    guess = self.mem_params.get("MapSpeedLimitGuess") or {}
    return float(guess.get('speedlimit', 0.0)), str(guess.get('source', ''))
  # End BluePilot

  def get_current_road_name(self) -> str:
    return str(self.mem_params.get("RoadName") or "")

  def get_next_speed_limit_and_distance(self) -> tuple[float, float]:
    next_speed_limit_section_str = self.mem_params.get("NextMapSpeedLimit")
    next_speed_limit_section = next_speed_limit_section_str if next_speed_limit_section_str else {}
    next_speed_limit = next_speed_limit_section.get('speedlimit', 0.0)
    next_speed_limit_latitude = next_speed_limit_section.get('latitude')
    next_speed_limit_longitude = next_speed_limit_section.get('longitude')
    next_speed_limit_distance = 0.0

    if next_speed_limit_latitude and next_speed_limit_longitude:
      next_speed_limit_coordinates = Coordinate(next_speed_limit_latitude, next_speed_limit_longitude)
      next_speed_limit_distance = (self.last_position or Coordinate(0, 0)).distance_to(next_speed_limit_coordinates)

    return next_speed_limit, next_speed_limit_distance
