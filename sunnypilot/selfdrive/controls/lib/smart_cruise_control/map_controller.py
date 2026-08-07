import json
import math
import platform

from cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.navd.helpers import coordinate_from_param, Coordinate
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState

ACTIVE_STATES = (MapState.turning, )
ENABLED_STATES = (MapState.enabled, MapState.overriding, *ACTIVE_STATES)

R = 6373000.0  # approximate radius of earth in meters
TO_RADIANS = math.pi / 180
TO_DEGREES = 180 / math.pi
TARGET_JERK = -0.6  # m/s^3 There's some jounce limits that are not consistent so we're fudging this some
TARGET_ACCEL = -1.2  # m/s^2 should match up with the long planner limit
TARGET_OFFSET = 1.0  # seconds - This controls how soon before the curve you reach the target velocity. It also helps
                     # reach the target velocity when inaccuracies in the distance modeling logic would cause overshoot.
                     # The value is multiplied against the target velocity to determine the additional distance. This is
                     # done to keep the distance calculations consistent but results in the offset actually being less
                     # time than specified depending on how much of a speed differential there is between v_ego and the
                     # target velocity.
# BluePilot: thresholds for the along-path distance walk and its U-turn/looped-chain truncation
PATH_REVERSAL_DEGREES = 150.0  # degrees - a forward-path segment whose bearing reverses by more than this vs the
                               # previous segment is a looped/U-turn chain doubling back on itself (same threshold
                               # as mapd's next-way U-turn rejection); the road ahead never bends this sharply
                               # between adjacent points, so the forward path is truncated there.
MIN_SEGMENT_DISTANCE = 0.1  # meters - segments shorter than this carry no meaningful bearing (duplicate or
                            # jittering points), so they are skipped when looking for a bearing reversal.
# End BluePilot


def velocities_from_param(param: str, params: Params):
  if params is None:
    params = Params()

  json_str = params.get(param)
  if json_str is None:
    return None

  velocities = json.loads(json_str)

  return velocities


def calculate_accel(t, target_jerk, a_ego):
  return a_ego + target_jerk * t


def calculate_velocity(t, target_jerk, a_ego, v_ego):
  return v_ego + a_ego * t + target_jerk/2 * (t ** 2)


def calculate_distance(t, target_jerk, a_ego, v_ego):
  return t * v_ego + a_ego/2 * (t ** 2) + target_jerk/6 * (t ** 3)


# points should be in radians
# output is meters
def distance_to_point(ax, ay, bx, by):
  a = math.sin((bx-ax)/2)*math.sin((bx-ax)/2) + math.cos(ax) * math.cos(bx)*math.sin((by-ay)/2)*math.sin((by-ay)/2)
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

  return R * c  # in meters


# BluePilot: bearing helpers for the forward-path reversal truncation
# points should be in radians
# output is the initial bearing from a to b in degrees, in [-180, 180]
def bearing_to_point(ax, ay, bx, by):
  d_lon = by - ay
  x = math.sin(d_lon) * math.cos(bx)
  y = math.cos(ax) * math.sin(bx) - math.sin(ax) * math.cos(bx) * math.cos(d_lon)

  return math.atan2(x, y) * TO_DEGREES


# bearings should be in degrees
# output is the signed smallest angle from bearing a to bearing b in degrees, in [-180, 180)
def bearing_delta(a, b):
  return (b - a + 180) % 360 - 180


# the LastGPSPosition JSON (written by osm_map_data.update_location) also carries the
# vehicle's own 'bearing'; it seeds the path-reversal walk so the FIRST forward segment
# is checked against the reversal rule too. Returns None when the field is absent.
def bearing_from_param(param: str, params: Params) -> float | None:
  json_str = params.get(param)
  if json_str is None:
    return None

  try:
    pos = json.loads(json_str)
  except (json.JSONDecodeError, TypeError):
    return None

  if not isinstance(pos, dict):
    return None

  bearing = pos.get('bearing')
  return float(bearing) if isinstance(bearing, (int, float)) else None


class MapPath:
  """One parse of MapTargetVelocities plus every derived quantity that depends only on the
  points and not on where the car is.

  mapd republishes MapTargetVelocities once per second while this controller runs at 20 Hz
  (DT_MDL), so 19 of every 20 cycles used to re-read the blob, re-run json.loads, rebuild the
  placeholder-filtered list and re-derive the same per-segment geometry. Everything here is a
  pure function of the point list, so it is built once per publish and reused; the
  position-dependent work (nearest point, along-path accumulation, braking window) still runs
  every cycle. Values are computed with exactly the same expressions the per-cycle code used,
  so the cached results are bit-identical to recomputing them.
  """
  __slots__ = ('points', 'lat', 'lon', 'vel', 'lat_r', 'lon_r', 'cos_lat', 'seg', 'seg_bearing')

  def __init__(self, points: list):
    self.points = points
    n = len(points)
    self.lat = lat = [p["latitude"] for p in points]
    self.lon = lon = [p["longitude"] for p in points]
    self.vel = [p["velocity"] for p in points]
    self.lat_r = lat_r = [v * TO_RADIANS for v in lat]
    self.lon_r = lon_r = [v * TO_RADIANS for v in lon]
    self.cos_lat = [math.cos(v) for v in lat_r]

    # seg[i] / seg_bearing[i] describe the segment from point i-1 to point i. seg_bearing is
    # None exactly when the segment is shorter than MIN_SEGMENT_DISTANCE, which is the same
    # condition under which the walk used to skip the bearing check.
    self.seg = seg = [0.0] * n
    self.seg_bearing = seg_bearing = [None] * n
    for i in range(1, n):
      segment_distance = distance_to_point(lat_r[i - 1], lon_r[i - 1], lat_r[i], lon_r[i])
      seg[i] = segment_distance
      if segment_distance >= MIN_SEGMENT_DISTANCE:
        seg_bearing[i] = bearing_to_point(lat_r[i - 1], lon_r[i - 1], lat_r[i], lon_r[i])


_UNREAD = object()  # sentinel: no param value has been observed yet
# End BluePilot


class SmartCruiseControlMap:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.enabled = self.params.get_bool("SmartCruiseControlMap")
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.state = MapState.disabled
    self.v_cruise = 0
    self.target_lat = 0.0
    self.target_lon = 0.0
    self.frame = -1

    # BluePilot: caches of the two mem params this controller reads every cycle, each keyed on
    # the raw string the param held when the cached value was derived. mapd writes them at 1 Hz
    # via a temp-file+rename, so an unchanged string is the same publish and the parse can be
    # reused; a byte difference re-parses on that very cycle. Keying on the bytes themselves
    # (rather than on mtime/inode) makes the cached value a pure function of the payload.
    self._gps_raw = _UNREAD
    self._gps: tuple[Coordinate, float | None] = (Coordinate(0.0, 0.0), None)
    self._path_raw = _UNREAD
    self._path = MapPath([])
    # End BluePilot

    self.last_position = coordinate_from_param("LastGPSPosition", self.mem_params) or Coordinate(0.0, 0.0)
    self.target_velocities = velocities_from_param("MapTargetVelocities", self.mem_params) or []

  # BluePilot: single read+parse of LastGPSPosition for both the position and the vehicle
  # bearing. update_calculations used to read and json-parse this param twice per cycle - once
  # through coordinate_from_param and once through bearing_from_param - which also let the two
  # values come from different GPS fixes when mapd's write landed between the reads.
  @staticmethod
  def _parse_gps(json_str) -> tuple[Coordinate, float | None]:
    if json_str is None:
      return Coordinate(0.0, 0.0), None

    pos = json.loads(json_str)

    # same acceptance rule as coordinate_from_param: a payload missing either field is no fix
    if 'latitude' in pos and 'longitude' in pos:
      position = Coordinate(pos['latitude'], pos['longitude'])
    else:
      position = Coordinate(0.0, 0.0)

    bearing = pos.get('bearing') if isinstance(pos, dict) else None
    return position, (float(bearing) if isinstance(bearing, (int, float)) else None)

  def _update_gps(self) -> tuple[Coordinate, float | None]:
    json_str = self.mem_params.get("LastGPSPosition")
    if json_str != self._gps_raw:
      # parse first: a malformed payload must raise exactly as it did before, and must not
      # install a cache key that would suppress the raise on later cycles
      self._gps = self._parse_gps(json_str)
      self._gps_raw = json_str
    return self._gps

  def _update_target_path(self) -> MapPath:
    json_str = self.mem_params.get("MapTargetVelocities")
    if json_str != self._path_raw:
      velocities = json.loads(json_str) if json_str is not None else None

      # mapd's GetTargetVelocities leaves (0,0,0) placeholder entries for zero-curvature
      # points; a real 0 m/s target is never emitted, so drop them before they poison the
      # nearest-point search, the along-path accumulation and the bearing walk.
      self.target_velocities = [tv for tv in (velocities or []) if tv["velocity"] != 0]

      self._path = MapPath(self.target_velocities)
      self._path_raw = json_str
    else:
      # a copy, not the cached list itself: MapPath's arrays are derived from its points
      # once at parse time, so a caller mutating this attribute would leave the two
      # disagreeing for as long as mapd republishes the same payload
      self.target_velocities = list(self._path.points)
    return self._path
  # End BluePilot

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V)

    return V_CRUISE_UNSET

  def get_a_target_from_control(self) -> float:
    return self.a_ego

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlMap")

  def update_calculations(self) -> None:
    # BluePilot: one read+parse of LastGPSPosition covers both the position and the vehicle's
    # own bearing (which seeds the path-reversal truncation below; None when absent), and the
    # target-velocity path arrives pre-parsed with its position-independent geometry already
    # derived. Both are reused only while the underlying param bytes are unchanged.
    self.last_position, ego_bearing = self._update_gps()
    lat = self.last_position.latitude
    lon = self.last_position.longitude

    path = self._update_target_path()
    # End BluePilot

    if self.last_position is None or self.target_velocities is None:
      return

    # BluePilot: local aliases for the cached per-point arrays - same values the point dicts
    # carried, hoisted out of the hot loops below
    n = len(path.points)
    p_lat, p_lon, p_vel = path.lat, path.lon, path.vel
    p_lat_r, p_lon_r, p_cos_lat = path.lat_r, path.lon_r, path.cos_lat
    p_seg, p_seg_bearing = path.seg, path.seg_bearing

    ego_lat_r = lat * TO_RADIANS
    ego_lon_r = lon * TO_RADIANS
    ego_cos_lat = math.cos(ego_lat_r)
    sin, sqrt, atan2 = math.sin, math.sqrt, math.atan2
    # End BluePilot

    min_dist = 1000
    min_idx = 0

    # find our location in the path
    # BluePilot: distance_to_point inlined with cos(latitude) of both endpoints hoisted /
    # precomputed. Same operands in the same order, so the result is bit-identical.
    for i in range(n):
      s_lat = sin((p_lat_r[i] - ego_lat_r) / 2)
      s_lon = sin((p_lon_r[i] - ego_lon_r) / 2)
      a = s_lat * s_lat + ego_cos_lat * p_cos_lat[i] * s_lon * s_lon
      d = R * (2 * atan2(sqrt(a), sqrt(1 - a)))
      if d < min_dist:
        min_dist = d
        min_idx = i

    # crow-flight distance to the nearest point. min_dist is it whenever any point came within
    # the 1000 m seed; when none did, min_idx stayed 0 and the old full distances[] list would
    # have yielded the distance to point 0, so compute just that one.
    if min_dist < 1000:
      min_idx_distance = min_dist
    elif n:
      min_idx_distance = distance_to_point(ego_lat_r, ego_lon_r, p_lat_r[0], p_lon_r[0])
    else:
      min_idx_distance = 0.0
    # End BluePilot

    # BluePilot: measure distance to each forward point along the road instead of as the crow flies:
    # seed with the crow-flight distance to the nearest point, then accumulate point-to-point segment
    # lengths. The walk stops at the first segment whose bearing reverses by more than
    # PATH_REVERSAL_DEGREES vs the previous one - points past a reversal belong to a looped/U-turn
    # chain doubling back on us, not the road ahead. prev_bearing starts from the vehicle's own
    # bearing (when known) so a reversal at the very FIRST forward segment - nearest point at a
    # U-turn apex - is caught too; without it the check gracefully starts at the second segment.
    #
    # The walk and the braking-window test below are fused into one forward pass over
    # [min_idx, forward_end): the window test only reads the accumulated distance for its own
    # index, so interleaving them is equivalent to the two separate passes and drops the
    # per-cycle forward_distances[] list. forward_end is the exclusive end of the forward path,
    # i.e. where the old code truncated forward_points. bearing_delta() is inlined below to
    # keep the pass free of per-segment call overhead.
    forward_end = n
    prev_bearing = ego_bearing
    d = min_idx_distance

    # find velocities that we are within the distance we need to adjust for
    valid_velocities = []
    v_ego = self.v_ego
    a_ego = self.a_ego
    # loop invariant: depends only on a_ego / v_ego, which do not change inside the pass
    a_diff = (a_ego - TARGET_ACCEL)
    accel_t = abs(a_diff / TARGET_JERK)
    min_accel_v = calculate_velocity(accel_t, TARGET_JERK, a_ego, v_ego)

    for i in range(min_idx, n):
      if i != min_idx:
        bearing = p_seg_bearing[i]
        if bearing is not None:
          if prev_bearing is not None and abs((bearing - prev_bearing + 180) % 360 - 180) > PATH_REVERSAL_DEGREES:
            forward_end = i
            break
          prev_bearing = bearing
        d += p_seg[i]

      tv = p_vel[i]
      if tv > v_ego:
        continue

      max_d = 0
      if tv > min_accel_v:
        # calculate time needed based on target jerk
        a = 0.5 * TARGET_JERK
        b = a_ego
        c = v_ego - tv
        t_a = -1 * ((b**2 - 4 * a * c) ** 0.5 + b) / 2 * a
        t_b = ((b**2 - 4 * a * c) ** 0.5 - b) / 2 * a
        if not isinstance(t_a, complex) and t_a > 0:
          t = t_a
        else:
          t = t_b
        if isinstance(t, complex):
          continue

        max_d = max_d + calculate_distance(t, TARGET_JERK, a_ego, v_ego)
      else:
        t = accel_t
        max_d = calculate_distance(t, TARGET_JERK, a_ego, v_ego)

        # calculate additional time needed based on target accel
        t = abs((min_accel_v - tv) / TARGET_ACCEL)
        max_d += calculate_distance(t, 0, TARGET_ACCEL, min_accel_v)

      if d < max_d + tv * TARGET_OFFSET:
        valid_velocities.append((float(tv), p_lat[i], p_lon[i]))
    # End BluePilot

    # Find the smallest velocity we need to adjust for
    min_v = 100.0
    target_lat = 0.0
    target_lon = 0.0
    for tv, lat, lon in valid_velocities:
      if tv < min_v:
        min_v = tv
        target_lat = lat
        target_lon = lon

    if self.v_target < min_v and not (self.target_lat == 0 and self.target_lon == 0):
      held_lat, held_lon, held_v = self.target_lat, self.target_lon, self.v_target
      for i in range(min_idx, forward_end):
        tv = p_vel[i]
        if tv > v_ego:
          continue

        if p_lat[i] == held_lat and p_lon[i] == held_lon and tv == held_v:
          return

      # not found so let's reset
      self.v_target = 0.0
      self.target_lat = 0.0
      self.target_lon = 0.0

    self.v_target = min_v
    self.target_lat = target_lat
    self.target_lon = target_lon

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, TURNING
    if self.state != MapState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = MapState.disabled
      elif self.long_override:
        self.state = MapState.overriding

      else:
        # ENABLED
        if self.state == MapState.enabled:
          if self.v_cruise > self.v_target != 0:
            self.state = MapState.turning

        # TURNING
        elif self.state == MapState.turning:
          if self.v_cruise <= self.v_target or self.v_target == 0:
            self.state = MapState.enabled

        # OVERRIDING
        elif self.state == MapState.overriding:
          if not self.long_override:
            if self.v_cruise > self.v_target != 0:
              self.state = MapState.turning
            else:
              self.state = MapState.enabled

    # DISABLED
    elif self.state == MapState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = MapState.overriding
        else:
          self.state = MapState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update(self, long_enabled: bool, long_override: bool, v_ego, a_ego, v_cruise) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise = v_cruise

    self.update_params()
    self.update_calculations()

    self.is_enabled, self.is_active = self._update_state_machine()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
