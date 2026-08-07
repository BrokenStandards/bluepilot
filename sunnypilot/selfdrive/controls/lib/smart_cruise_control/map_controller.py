import json
import math
import platform
import threading

from cereal import custom
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
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

# BluePilot: continuous curve-approach profile.
#
# The old control law was a binary window: a curve either was or was not inside a jerk+accel
# braking distance, and points whose target velocity was above v_ego were skipped outright.
# Combined with the state machine only entering MapState.turning while v_cruise > v_target,
# that meant a mapped curve could only ever ask for a hard, late deceleration - or, whenever
# the published target sat above the driver's set speed, ask for nothing at all while cruise
# kept accelerating into the bend.
#
# What replaces it is a speed *profile*: every forward point contributes the speed the car
# would have to be doing right now to reach that point's target velocity under a constant,
# gentle approach deceleration, and the published target is the minimum of those. Far away the
# profile sits well above the set speed and loses the planner's min() to cruise, so the two
# never fight; as the curve comes in the profile crosses below the set speed smoothly and
# simply stops the car from accelerating, long before any friction braking would be needed.
NO_TARGET_V = 100.0  # m/s - "no curve is asking for anything"; also the value v_target keeps when
                     # no forward point produced a profile value, which never wins the planner's min()
HORIZON_MIN_M = 60.0  # meters - floor on the look-ahead distance so the profile still sees a curve
                      # at low speed, where v_ego * horizon collapses to nothing
TARGET_SLEW_RATE = 1.5  # m/s per second - hard limit on how fast the PUBLISHED target may RISE.
                        # The profile itself is continuous, but the set of points feeding it is not
                        # (a curve entering the horizon, a held target releasing, mapd republishing
                        # a different chain), and any step in the target lands on the long planner
                        # as a step in demanded acceleration. Ramping the release keeps the return
                        # to cruise smooth. The DOWNWARD rate is the profile's max_decel: when a
                        # curve is discovered late (a ramp branch mapd only sees once the car is on
                        # it), the target is allowed to fall exactly as fast as a human who noticed
                        # late would brake - firmly, up to the profile's ceiling, and no faster.
MAX_STALE_CYCLES = int(5.0 / DT_MDL)  # cycles - how long the position underneath the profile may
                                      # repeat before the profile gives up. mapd republishes at 1 Hz
                                      # into this 20 Hz loop, so ~20 repeats is the normal cadence.
HOLD_RELEASE_DISTANCE = 10.0  # meters - a held target is only given up once its point is this far
                              # BEHIND the car, so the hold survives the apex instead of dropping
                              # the moment the car reaches the target speed
HELD_TARGET_POS_TOL = 1.0 * TO_DEGREES / R  # degrees - ~1 m; a held target is re-found by POSITION,
                                            # not by (position, velocity), so the profile value moving
                                            # as the car approaches cannot lose track of the point
# End BluePilot

# BluePilot: curve speed presets. Single source of truth for the three profiles the UI selects
# between with "CurveSpeedProfile"; the UI only ever stores the index.
#
# The numbers are fitted to how humans actually drive, from route-verified OSM GPS trace
# bands over Nashville (city arterial curves n~80/curve, a verified cloverleaf loop and
# freeway exit, interstate sweepers) cross-checked against the published literature
# (AASHTO side-friction comfort curve, Fitzpatrick FHWA-RD-99-171 decel rates, SHRP2
# ramp/exit naturalistic studies):
#   - the median human on a 30 mph arterial curve runs 2.0 m/s^2 against the map radius
#     (p25 1.7, p75 2.3) - so the three lat anchors ARE those three percentiles;
#   - city curves are barely braking events (1-3 mph shaved at 0.2-0.5 m/s^2), while a
#     61->34 mph freeway-exit approach sustains ~2.0 m/s^2 and peaks at 2.5 - no constant
#     decel fits both, hence the drop-proportional ramp between base and max below;
#   - humans reach minimum speed AT or slightly past curve entry, not comfortably before
#     it, so the offsets shrink toward zero as the profiles get sportier.
#
#   lat_accel       m/s^2 - handed to mapd through "MapTargetLatA"; the lateral budget AT THE
#                           30 MPH ANCHOR. mapd shapes the budget over speed (AASHTO comfort
#                           curve) and applies the ramp-context boost; see math.go latBudget.
#   base_decel      m/s^2 - approach deceleration for small speed trims (city curves)
#   max_decel       m/s^2 - approach deceleration ceiling for large drops (freeway exits);
#                           also the fastest the published target may fall per second
#   offset          s     - the target velocity is reached this long before the curve point
#   horizon         s     - how far ahead, in time, a curve may bind (see HORIZON_MIN_M)
CURVE_SPEED_PRESETS = (
  (1.7, 0.30, 1.2, 1.5, 12.0),  # 0 Comfort
  (2.0, 0.45, 1.8, 0.8, 10.0),  # 1 Normal
  (2.4, 0.60, 2.5, 0.3, 8.0),   # 2 Sport
)
# A drop of this many m/s (v_ego above the point's target) is where the approach decel
# reaches max_decel; between 0 and this the decel rises QUADRATICALLY from base_decel.
# Quadratic, not linear, because that is what the verified bands show at all three measured
# drops: a 1-2 mph city trim is taken at ~0.3-0.5 m/s^2, a 10 mph freeway trim (I-65,
# 66->55 mph) at a gentle 0.46 m/s^2 sustained - linear gain would have doubled that - and
# the 27 mph shed into the verified cloverleaf at ~2.0 m/s^2 sustained. 12 m/s is that
# cloverleaf drop, where humans were at their sustained maximum.
DECEL_FULL_DROP = 12.0
DEFAULT_CURVE_SPEED_PROFILE = 1
CURVE_SPEED_PROFILE_PARAM = "CurveSpeedProfile"
MAP_TARGET_LAT_A_PARAM = "MapTargetLatA"
# End BluePilot

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


# BluePilot: the jerk/accel braking-window model (calculate_accel / calculate_velocity /
# calculate_distance and the quadratic solve that used them) is gone - the continuous approach
# profile subsumes it. That also retires the operator-precedence bug in the old solve, where
# '-1 * ((b**2 - 4*a*c)**0.5 + b) / 2 * a' multiplied by a instead of dividing by 2*a.
# End BluePilot


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

    # BluePilot: curve speed preset, plus the slew state of the published target. The preset is
    # applied here rather than on the first update() because update_params() only runs on frames
    # that are a multiple of PARAMS_UPDATE_PERIOD / DT_MDL and frame starts at -1.
    self.profile_index = -1
    self.approach_decel, self.max_decel, self.target_offset, self.horizon = CURVE_SPEED_PRESETS[DEFAULT_CURVE_SPEED_PROFILE][1:]
    self._profile_param_supported = True
    self._lat_accel_param_supported = True
    self._published_v_target: float | None = None  # None == released, publishing V_CRUISE_UNSET
    self._apply_profile(self._read_profile_index())
    # End BluePilot

    # BluePilot: caches of the two mem params this controller reads every cycle, each keyed on
    # the raw string the param held when the cached value was derived. mapd writes them at 1 Hz
    # via a temp-file+rename, so an unchanged string is the same publish and the parse can be
    # reused; a byte difference re-parses on that very cycle. Keying on the bytes themselves
    # (rather than on mtime/inode) makes the cached value a pure function of the payload.
    self._gps_raw = _UNREAD
    self._stale_cycles = 0
    self._gps: tuple[Coordinate, float | None] = (Coordinate(0.0, 0.0), None)
    self._path_raw = _UNREAD
    self._path = MapPath([])
    # End BluePilot

    self.last_position = coordinate_from_param("LastGPSPosition", self.mem_params) or Coordinate(0.0, 0.0)
    self.target_velocities = velocities_from_param("MapTargetVelocities", self.mem_params) or []

  # BluePilot: curve speed preset selection. The UI stores an index in "CurveSpeedProfile"; the
  # preset table above is the only place the numbers live. Both params are guarded against
  # UnknownKeyName the same way osm_map_data.py guards its optional keys, so a build whose
  # params library predates them silently keeps the default preset instead of crashing the
  # planner. Each key is probed at most once - after a miss the feature stays inert.
  def _read_profile_index(self) -> int:
    if not self._profile_param_supported:
      return DEFAULT_CURVE_SPEED_PROFILE

    try:
      value = self.params.get(CURVE_SPEED_PROFILE_PARAM, return_default=True)
    except UnknownKeyName:
      cloudlog.warning(f"map_controller: {CURVE_SPEED_PROFILE_PARAM} not registered in this build, keeping the default preset")
      self._profile_param_supported = False
      return DEFAULT_CURVE_SPEED_PROFILE

    try:
      index = int(value)
    except (TypeError, ValueError):
      return DEFAULT_CURVE_SPEED_PROFILE

    return index if 0 <= index < len(CURVE_SPEED_PRESETS) else DEFAULT_CURVE_SPEED_PROFILE

  def _apply_profile(self, index: int) -> None:
    if index == self.profile_index:
      return

    self.profile_index = index
    lat_accel, self.approach_decel, self.max_decel, self.target_offset, self.horizon = CURVE_SPEED_PRESETS[index]
    self._write_map_target_lat_a(lat_accel)

  def _write_map_target_lat_a(self, lat_accel: float) -> None:
    """Hand the preset's lateral acceleration to mapd.

    mapd reads the PERSISTENT copy once in main() and the /dev/shm copy on every 1 Hz tick,
    deleting the shm copy after applying it - so the shm write is what takes effect without a
    mapd restart, and the persistent write is what survives one. The persistent copy is only
    rewritten when it actually differs, to keep preset changes off the flash on every boot.
    """
    if not self._lat_accel_param_supported:
      return

    try:
      # shm first and inline: it is tmpfs, it is what mapd actually picks up, and it costs
      # microseconds. The persistent copy is an fsync'd write to flash, which has no business
      # blocking a 20 Hz control thread just because the driver touched a setting while
      # moving, so it goes to a daemon thread.
      self.mem_params.put(MAP_TARGET_LAT_A_PARAM, lat_accel)
      if self.params.get(MAP_TARGET_LAT_A_PARAM, return_default=True) != lat_accel:
        threading.Thread(target=self._write_persistent_lat_a, args=(lat_accel,), daemon=True).start()
    except UnknownKeyName:
      cloudlog.warning(f"map_controller: {MAP_TARGET_LAT_A_PARAM} not registered in this build, mapd keeps its own lat accel")
      self._lat_accel_param_supported = False
    except OSError:
      cloudlog.exception(f"map_controller: could not write {MAP_TARGET_LAT_A_PARAM}")
  def _write_persistent_lat_a(self, lat_accel: float) -> None:
    try:
      self.params.put(MAP_TARGET_LAT_A_PARAM, lat_accel, block=True)
    except (UnknownKeyName, OSError):
      cloudlog.exception(f"map_controller: could not persist {MAP_TARGET_LAT_A_PARAM}")
  # End BluePilot

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
      self._stale_cycles = 0
    else:
      self._stale_cycles += 1
    return self._gps

  @property
  def _data_is_stale(self) -> bool:
    """Whether the position underneath the profile has stopped moving.

    The old control law could only ever ask for a deceleration the car was not already
    achieving, so frozen inputs quietly stopped mattering. The profile is a standing speed
    limit, so a frozen position would pin the car at whatever the last curve asked for -
    and osm_map_data keeps republishing its last known fix at 1 Hz when the localizer goes
    invalid, so a tunnel or a dead mapd looks exactly like a stationary car. mapd publishes
    at 1 Hz against this 20 Hz loop, so a couple of dozen repeats is normal and only a long
    silence means the data is gone.
    """
    return self._stale_cycles > MAX_STALE_CYCLES

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

  # BluePilot: slew-limited publish (P3). The value the planner sees may never step, in either
  # direction, so the map source can neither slam a lower target onto the long planner (sudden
  # friction braking) nor snap back to cruise on release. Inactive is published as
  # V_CRUISE_UNSET exactly as before, but only once the ramp has climbed back to the level at
  # which the map target no longer constrains anything (v_cruise) - until then the ramp itself
  # is published, which is what makes the release smooth. A hard release (feature off,
  # longitudinal off, driver override) still snaps, because in those states there is nothing
  # left to ramp towards.
  #
  # This advances the ramp, so like the rest of the update() chain it is called exactly once
  # per 20 Hz cycle; read self.output_v_target for the value published on the current cycle.
  def get_v_target_from_control(self) -> float:
    if not self.is_enabled or self.long_override:
      self._published_v_target = None
      return V_CRUISE_UNSET

    released = self.v_cruise  # the level at which this source stops winning the planner's min()
    prev = self._published_v_target

    if self.is_active:
      desired = max(self.v_target, MIN_V)
      if prev is None:
        # first cycle of an engagement: start from whatever already governs the car, so the
        # very first published value is not itself a step. The continuous profile crosses
        # v_cruise on its way down, so in normal operation desired is already ~= this seed.
        prev = min(released, max(self.v_ego, desired))
    elif prev is None:
      return V_CRUISE_UNSET
    else:
      desired = released

    max_up = TARGET_SLEW_RATE * DT_MDL
    max_down = self.max_decel * DT_MDL
    if desired > prev + max_up:
      new = prev + max_up
    elif desired < prev - max_down:
      new = prev - max_down
    else:
      new = desired

    if not self.is_active and new >= released:
      self._published_v_target = None
      return V_CRUISE_UNSET

    self._published_v_target = new
    return max(new, MIN_V)
  # End BluePilot

  def get_a_target_from_control(self) -> float:
    return self.a_ego

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlMap")
      # BluePilot: preset selection rides the existing param cadence - no new per-cycle reads
      self._apply_profile(self._read_profile_index())
      # End BluePilot

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

    # BluePilot: frozen inputs release the profile rather than pinning the car at the last
    # curve's target (see _data_is_stale). Reset exactly like a path with nothing in range.
    if self._data_is_stale:
      self.v_target = NO_TARGET_V
      self.target_lat = 0.0
      self.target_lon = 0.0
      return
    # End BluePilot

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

    # BluePilot: the held target is re-found by POSITION in this same sweep (P5). The old code
    # looked it up in the forward path by (lat, lon, velocity) *and* re-applied a tv > v_ego
    # filter, so the hold collapsed the instant the car slowed to the target - right at the
    # entry of the curve, letting cruise re-accelerate through the apex. Matching on position
    # alone, over the whole path, keeps the hold alive while the point is ahead and for
    # HOLD_RELEASE_DISTANCE after it has gone behind.
    held_lat = self.target_lat
    held_lon = self.target_lon
    has_held = not (held_lat == 0.0 and held_lon == 0.0)
    held_idx = -1
    held_dist = 0.0
    # End BluePilot

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

      # BluePilot: held-target lookup, folded into the sweep that already visits every point
      if has_held and held_idx < 0 and abs(p_lat[i] - held_lat) <= HELD_TARGET_POS_TOL \
         and abs(p_lon[i] - held_lon) <= HELD_TARGET_POS_TOL:
        held_idx = i
        held_dist = d
      # End BluePilot

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
    # The walk and the approach-profile evaluation below are fused into one forward pass over
    # [min_idx, forward_end): the profile only reads the accumulated distance for its own
    # index, so interleaving them is equivalent to the two separate passes and drops the
    # per-cycle forward_distances[] list. forward_end is the exclusive end of the forward path,
    # i.e. where the old code truncated forward_points. bearing_delta() is inlined below to
    # keep the pass free of per-segment call overhead.
    #
    # P1/P2 - continuous approach profile. For every forward point within the horizon:
    #
    #   d_eff  = max(0, d_i - tv_i * offset)                  # reach tv 'offset' seconds early
    #   v_ref_i = sqrt(tv_i**2 + 2 * approach_decel * d_eff)   # constant-decel speed profile
    #
    # and the published target is min(v_ref_i). v_ref is >= tv everywhere, equals tv at
    # d = tv * offset, and rises monotonically with distance, so a curve that is still far off
    # simply yields a value above the set speed and loses the planner's min() to cruise. There
    # is no gate, no window and no "target is above v_ego so ignore it" skip: the transition
    # from "cruise governs" to "the curve governs" is the two continuous curves crossing.
    forward_end = n
    prev_bearing = ego_bearing
    d = min_idx_distance

    v_ego = self.v_ego
    # loop invariants: the preset and v_ego do not change inside the pass.
    #
    # The approach decel is drop-proportional (see CURVE_SPEED_PRESETS and DECEL_FULL_DROP):
    # a point asking for a 2 mph trim gets base_decel, a freeway exit asking to shed 25 mph
    # gets up to max_decel - which is exactly how humans split those cases. Using v_ego for
    # the drop means the decel the profile is built on relaxes as the car actually slows,
    # easing the tail of the deceleration instead of holding one fixed rate to the end.
    base_decel = self.approach_decel
    decel_gain = (self.max_decel - base_decel) / (DECEL_FULL_DROP * DECEL_FULL_DROP)
    offset = self.target_offset
    horizon_m = max(HORIZON_MIN_M, v_ego * self.horizon)

    min_v = NO_TARGET_V
    target_lat = 0.0
    target_lon = 0.0

    for i in range(min_idx, n):
      if i != min_idx:
        bearing = p_seg_bearing[i]
        if bearing is not None:
          if prev_bearing is not None and abs((bearing - prev_bearing + 180) % 360 - 180) > PATH_REVERSAL_DEGREES:
            forward_end = i
            break
          prev_bearing = bearing
        d += p_seg[i]

      # the walk keeps going past the horizon so the reversal truncation still finds
      # forward_end, but points out there may not bind
      if d > horizon_m:
        continue

      tv = p_vel[i]
      d_eff = d - tv * offset
      if d_eff > 0.0:
        drop = v_ego - tv
        a_eff = base_decel if drop <= 0.0 else base_decel + decel_gain * drop * drop
        if a_eff > self.max_decel:
          a_eff = self.max_decel
        v_ref = sqrt(tv * tv + 2.0 * a_eff * d_eff)
      else:
        v_ref = tv

      if v_ref < min_v:
        min_v = v_ref
        target_lat = p_lat[i]
        target_lon = p_lon[i]
    # End BluePilot

    # BluePilot: P5 - hold the target through the apex. The profile value for a given point
    # only rises again once that point is behind us (or drops out of the path entirely), so
    # this branch is exactly the "the curve stopped asking for anything" case. Release it only
    # when the held point really is behind the car - the nearest index has moved past it and it
    # is more than HOLD_RELEASE_DISTANCE away - or when it is no longer on the forward path at
    # all (gone from the publish, or beyond a path reversal).
    if self.v_target < min_v and has_held:
      still_ahead = 0 <= held_idx < forward_end and held_dist <= horizon_m \
          and (held_idx >= min_idx or held_dist <= HOLD_RELEASE_DISTANCE)
      if still_ahead:
        # Keep holding - but never below the held point's CURRENT target velocity. If mapd
        # retracts the curve (same node republished with a higher velocity) the hold relaxes
        # with it instead of freezing a stale number all the way to the node.
        held_v = p_vel[held_idx]
        if held_v < min_v:
          if held_v > self.v_target:
            self.v_target = held_v
          return

      else:
        # not found so let's reset
        self.v_target = 0.0
        self.target_lat = 0.0
        self.target_lon = 0.0
    # End BluePilot

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
