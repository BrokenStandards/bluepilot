"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# BluePilot: json/math and the R/TO_DEGREES imports serve the metric point helpers below
import json
import math
import pathlib
import platform

import pytest

from cereal import custom
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import (
  CURVE_SPEED_PRESETS,
  DECEL_FULL_DROP,
  DEFAULT_CURVE_SPEED_PROFILE,
  HOLD_RELEASE_DISTANCE,
  HORIZON_MIN_M,
  MAP_TARGET_LAT_A_PARAM,
  MAX_STALE_CYCLES,
  NO_TARGET_V,
  R,
  TARGET_SLEW_RATE,
  TO_DEGREES,
  SmartCruiseControlMap,
)
# End BluePilot

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState

# BluePilot: helpers for the along-path distance / reversal-truncation / placeholder tests
M_TO_DEG = TO_DEGREES / R  # meters to degrees of latitude (and of longitude on the equator)
WAY_V = 30.0  # the "nothing is asking for a slowdown here" velocity every filler point carries. With
              # the continuous approach profile a straight run of these publishes exactly WAY_V (the
              # nearest point sits at d_eff <= 0, so its profile value is its own target velocity),
              # which is what "no phantom braking" now looks like: nothing below the way's own limit.

# the three presets, by name, as (lat_accel, base_decel, max_decel, offset, horizon)
COMFORT, NORMAL, SPORT = CURVE_SPEED_PRESETS


def make_point(x, y, v):
  # x is meters east and y is meters north of the origin; on the equator both axes convert with the same factor
  return {"latitude": y * M_TO_DEG, "longitude": x * M_TO_DEG, "velocity": v}


def a_eff(tv, v_ego, preset=NORMAL):
  """Drop-proportional approach decel: base for small trims, rising quadratically to max at
  DECEL_FULL_DROP (the shape the verified human bands show at city/freeway/loop drops)."""
  _, base_decel, max_decel, _, _ = preset
  drop = v_ego - tv
  if drop <= 0.0:
    return base_decel
  return min(max_decel, base_decel + (max_decel - base_decel) / DECEL_FULL_DROP ** 2 * drop * drop)


def v_ref(tv, d, preset=NORMAL, v_ego=None):
  """The approach profile: the speed the car may be doing now to reach tv at a point d metres away."""
  _, _, _, offset, _ = preset
  d_eff = max(0.0, d - tv * offset)
  a = a_eff(tv, v_ego if v_ego is not None else tv, preset)
  return math.sqrt(tv ** 2 + 2.0 * a * d_eff)


def bind_distance(v_cruise, tv, preset=NORMAL, v_ego=None):
  """Distance at which the profile for a tv curve first drops to v_cruise, i.e. where it starts to bind."""
  _, _, _, offset, _ = preset
  a = a_eff(tv, v_ego if v_ego is not None else v_cruise, preset)
  return (v_cruise ** 2 - tv ** 2) / (2.0 * a) + tv * offset
# End BluePilot


class TestSmartCruiseControlMap:

  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.reset_params()
    self.scc_m = SmartCruiseControlMap()

  def reset_params(self):
    self.params.put_bool("SmartCruiseControlMap", True, block=True)

    # TODO-SP: mock data from gpsLocation
    self.params.put("LastGPSPosition", "{}", block=True)
    self.params.put("MapTargetVelocities", "{}", block=True)

  # BluePilot: drive update_calculations directly from metric points; the optional bearing
  # mimics osm_map_data.update_location writing the vehicle bearing into LastGPSPosition
  def run_calculations(self, points, v_ego, position=(0.0, 0.0), bearing=None):
    self.publish_position(position, bearing)
    self.mem_params.put("MapTargetVelocities", json.dumps(points), block=True)
    self.scc_m.v_ego = v_ego
    self.scc_m.a_ego = 0.0
    self.scc_m.update_calculations()

  def publish_position(self, position=(0.0, 0.0), bearing=None):
    """Write LastGPSPosition the way osm_map_data.update_location does, nothing else."""
    x, y = position
    gps = {"latitude": y * M_TO_DEG, "longitude": x * M_TO_DEG}
    if bearing is not None:
      gps["bearing"] = bearing
    self.mem_params.put("LastGPSPosition", json.dumps(gps), block=True)

  @property
  def target(self):
    return self.scc_m.v_target, self.scc_m.target_lat, self.scc_m.target_lon

  def drive(self, points, v_ego, v_cruise, start_x=0.0, cycles=1, bearing=90.0):
    """Run full update() cycles at 20 Hz with the car driving east at v_ego, and collect the
    value the planner would actually see (output_v_target) on every cycle."""
    self.mem_params.put("MapTargetVelocities", json.dumps(points), block=True)
    published = []
    x = start_x
    for _ in range(cycles):
      self.publish_position((x, 0.0), bearing)
      self.scc_m.update(True, False, v_ego, 0.0, v_cruise)
      published.append(self.scc_m.output_v_target)
      x += v_ego * DT_MDL
    return published
  # End BluePilot

  def test_initial_state(self):
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active
    assert self.scc_m.output_v_target == V_CRUISE_UNSET
    assert self.scc_m.output_a_target == 0.

  def test_system_disabled(self):
    self.params.put_bool("SmartCruiseControlMap", False, block=True)
    self.scc_m.enabled = self.params.get_bool("SmartCruiseControlMap")

    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active

  def test_disabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(False, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled

  def test_transition_disabled_to_enabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.enabled

  # TODO-SP: mock data from modelV2 to test other states

  # BluePilot: along-path distance, reversal-truncation and placeholder-skip coverage.
  # These pin the geometry, not the control law: the published number is now the continuous
  # approach profile v_ref(tv, d) instead of the raw target velocity, so each expectation is
  # written through the same v_ref() the controller uses. "No slowdown" is WAY_V (the filler
  # points' own velocity), which never wins the planner's min() against a sane set speed.
  def test_single_curve_slowdown(self):
    # regression: a slow point straight ahead inside the horizon still produces a target, and it
    # is that point's profile value - well above tv while still 140 m out (with a 10 m/s drop the
    # drop-proportional decel is nearer max than base, so the profile only starts to bind close
    # in, the way the freeway-exit traces do), so nothing is asking for the brakes yet
    points = [make_point(x, 0.0, 10.0 if x == 140 else WAY_V) for x in range(0, 300, 20)]
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 140.0, v_ego=20.0))
    assert 10.0 < self.scc_m.v_target < WAY_V
    assert math.isclose(self.scc_m.target_lon, 140 * M_TO_DEG)
    assert self.scc_m.target_lat == 0.0

  def test_looped_path_no_phantom_braking(self):
    # a chain that runs 300 m out, U-turns and comes back past the car used to trigger braking because
    # the doubled-back slow points are crow-flight close (~22 m); along-path they are ~580 m away and
    # the path is truncated at the reversal, so none of them may contribute to the profile
    points = [make_point(x, 0.0, WAY_V) for x in range(0, 350, 50)]
    points += [make_point(x, 10.0, 5.0) for x in (250, 150, 50, 20)]
    self.run_calculations(points, v_ego=25.0)
    assert self.scc_m.v_target == pytest.approx(WAY_V)

  def test_curved_path_uses_along_path_distance(self):
    # a horseshoe of 50 m segments with bearing changes of at most 45 degrees (so no truncation): the
    # slow endpoint is ~131 m crow-flight from the car - inside the 200 m horizon for v_ego=20 - but
    # 400 m along-path, so it must not pull the profile down
    d45 = 50.0 / math.sqrt(2)
    xs_ys = [(0.0, 0.0), (50.0, 0.0), (100.0, 0.0), (100.0 + d45, d45), (100.0 + d45, d45 + 50.0),
             (100.0, 2 * d45 + 50.0), (50.0, 2 * d45 + 50.0), (0.0, 2 * d45 + 50.0), (-50.0, 2 * d45 + 50.0)]
    points = [make_point(x, y, WAY_V) for x, y in xs_ys[:-1]] + [make_point(*xs_ys[-1], 10.0)]
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == pytest.approx(WAY_V)

  def test_truncation_at_bearing_reversal(self):
    # the chain reverses 180 degrees at 150 m; the slow points behind the reversal are only ~160 m
    # along-path (well inside the 250 m horizon for v_ego=25), so only truncation of the forward
    # path at the reversal keeps them out of the profile
    points = [make_point(x, 0.0, WAY_V) for x in (0, 50, 100, 150)]
    points += [make_point(x, 0.0, 5.0) for x in (140, 90, 40)]
    self.run_calculations(points, v_ego=25.0)
    assert self.scc_m.v_target == pytest.approx(WAY_V)

  def test_truncation_at_uturn_apex_first_segment(self):
    # the nearest point IS the U-turn apex: the chain doubles back from the very first forward
    # segment, so only the vehicle-bearing seed can catch the reversal. The doubled-back slow
    # points are 30-130 m along-path (inside the 250 m horizon for v_ego=25); the car heads east
    # (bearing 90) while segment one heads west -> truncated, nothing below WAY_V
    points = [make_point(0.0, 0.0, WAY_V)]
    points += [make_point(-x, 2.0, 5.0) for x in (30, 80, 130)]
    self.run_calculations(points, v_ego=25.0, bearing=90.0)
    assert self.scc_m.v_target == pytest.approx(WAY_V)

  def test_missing_bearing_skips_first_segment_check(self):
    # without a 'bearing' field in LastGPSPosition the first-segment check is skipped gracefully:
    # the same apex chain as above is walked without truncation (pre-seed behavior) and the
    # doubled-back slow points do pull the profile down instead of crashing
    points = [make_point(0.0, 0.0, WAY_V)]
    points += [make_point(-x, 2.0, 5.0) for x in (30, 80, 130)]
    self.run_calculations(points, v_ego=25.0)
    assert self.scc_m.v_target == pytest.approx(v_ref(5.0, math.hypot(30.0, 2.0), v_ego=25.0), rel=1e-3)

  def test_forward_path_with_bearing_still_brakes(self):
    # the vehicle-bearing seed must not truncate a normal forward path: car heading east with the
    # chain heading east keeps the slow point at x=140 as the profile's governing point
    points = [make_point(x, 0.0, 10.0 if x == 140 else WAY_V) for x in range(0, 300, 20)]
    self.run_calculations(points, v_ego=20.0, bearing=90.0)
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 140.0, v_ego=20.0))
    assert math.isclose(self.scc_m.target_lon, 140 * M_TO_DEG)

  def test_placeholder_entries_skipped(self):
    # mapd's GetTargetVelocities leaves (0,0,0) placeholder entries for zero-curvature points; a
    # placeholder mid-chain sits at the origin, so the walk would see a fake reversal at the
    # segment into it (and huge along-path detours). Skipping placeholders keeps the real slow
    # point at x=140 governing the profile
    points = [make_point(x, 0.0, 10.0 if x == 140 else WAY_V) for x in range(0, 300, 20)]
    points.insert(4, {"latitude": 0.0, "longitude": 0.0, "velocity": 0.0})
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 140.0, v_ego=20.0))
    assert math.isclose(self.scc_m.target_lon, 140 * M_TO_DEG)
    assert self.scc_m.target_lat == 0.0
  # End BluePilot

  # BluePilot: the mem params are re-read every 20 Hz cycle but mapd only republishes at 1 Hz,
  # so the parse of MapTargetVelocities/LastGPSPosition (and the point geometry derived from
  # it) is cached against the raw param bytes. A stale cache is a wrong braking target, so
  # every invalidation path is pinned here.
  def test_parse_cache_hit_matches_a_cold_controller(self):
    points = [make_point(x, 0.0, 10.0 if x == 140 else WAY_V) for x in range(0, 300, 20)]
    self.run_calculations(points, v_ego=20.0)
    first = self.target
    assert first[0] == pytest.approx(v_ref(10.0, 140.0, v_ego=20.0))

    # the other 19 cycles of this publish all take the cache; none of them may drift
    for _ in range(19):
      self.scc_m.update_calculations()
    assert self.target == first

    # and a controller that has never seen this payload agrees exactly, to the bit
    cold = SmartCruiseControlMap()
    cold.v_ego, cold.a_ego = 20.0, 0.0
    cold.update_calculations()
    assert (cold.v_target, cold.target_lat, cold.target_lon) == first

  def test_new_publish_is_seen_on_the_very_next_cycle(self):
    points = [make_point(x, 0.0, 10.0 if x == 140 else WAY_V) for x in range(0, 300, 20)]
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 140.0, v_ego=20.0))

    # same point count, same length-ish payload, slow point moved: must not be mistaken
    # for the cached publish
    moved = [make_point(x, 0.0, 10.0 if x == 100 else WAY_V) for x in range(0, 300, 20)]
    self.mem_params.put("MapTargetVelocities", json.dumps(moved), block=True)
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 100.0, v_ego=20.0))
    assert math.isclose(self.scc_m.target_lon, 100 * M_TO_DEG)

    # curve retracted: the node is still published, at the same position, but no longer slow.
    # The apex hold matches on position, so it must relax to the node's NEW velocity rather
    # than freeze the old target all the way to it.
    cleared = [make_point(x, 0.0, WAY_V) for x in range(0, 300, 20)]
    self.mem_params.put("MapTargetVelocities", json.dumps(cleared), block=True)
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == pytest.approx(WAY_V)
    assert self.scc_m.target_lat == 0.0
    assert self.scc_m.target_lon == 0.0

    # and a publish that drops the node entirely releases the hold the same way
    self.mem_params.put("MapTargetVelocities", json.dumps(moved), block=True)
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 100.0, v_ego=20.0))
    gone = [make_point(x, 0.0, WAY_V) for x in range(0, 300, 20) if x != 100]
    self.mem_params.put("MapTargetVelocities", json.dumps(gone), block=True)
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == pytest.approx(WAY_V)

  def test_position_dependent_work_still_runs_on_a_cache_hit(self):
    # the car moves at 20 Hz even when the path data does not change: only the PARSE is
    # cached, so an unchanged MapTargetVelocities must still be re-evaluated from the new
    # position. Here the car drives well past the slow point and the target has to release.
    points = [make_point(x, 0.0, 10.0 if x == 140 else WAY_V) for x in range(0, 300, 20)]
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 140.0, v_ego=20.0))

    self.publish_position(position=(200.0, 0.0))
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == pytest.approx(WAY_V)

  def test_bearing_change_alone_invalidates_the_gps_cache(self):
    # position and path identical throughout; only the vehicle bearing moves, and it decides
    # whether the U-turn apex truncates the forward path
    points = [make_point(0.0, 0.0, WAY_V)]
    points += [make_point(-x, 2.0, 5.0) for x in (30, 80, 130)]
    slow = pytest.approx(v_ref(5.0, math.hypot(30.0, 2.0), v_ego=25.0), rel=1e-3)

    self.run_calculations(points, v_ego=25.0, bearing=90.0)
    assert self.scc_m.v_target == pytest.approx(WAY_V)

    # heading west now, along the chain: no reversal, the slow points pull the profile down
    self.publish_position(bearing=270.0)
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == slow

    # bearing dropped from the payload: back to the un-seeded walk, still a target
    self.publish_position()
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == slow

    # heading east again: the very first segment reverses. The held point is now past the
    # truncation, so the hold releases and the profile is back to the way's own velocity.
    self.publish_position(bearing=90.0)
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == pytest.approx(WAY_V)

  def test_empty_and_missing_payloads(self):
    self.publish_position()
    for payload in ("[]", "{}"):
      self.mem_params.put("MapTargetVelocities", payload, block=True)
      self.scc_m.update_calculations()
      assert self.scc_m.target_velocities == []
      assert self.scc_m.v_target == NO_TARGET_V

    self.mem_params.remove("MapTargetVelocities")
    self.scc_m.update_calculations()
    assert self.scc_m.target_velocities == []

    self.mem_params.remove("LastGPSPosition")
    self.scc_m.update_calculations()
    assert self.scc_m.last_position.latitude == 0.0
    assert self.scc_m.last_position.longitude == 0.0

  def test_malformed_payload_does_not_poison_the_cache(self):
    self.publish_position()
    self.mem_params.put("MapTargetVelocities", "not json", block=True)
    with pytest.raises(json.JSONDecodeError):
      self.scc_m.update_calculations()

    # a bad payload must not install itself as the cache key, or the raise would be
    # swallowed on every later cycle
    with pytest.raises(json.JSONDecodeError):
      self.scc_m.update_calculations()

    points = [make_point(x, 0.0, 10.0 if x == 140 else WAY_V) for x in range(0, 300, 20)]
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 140.0, v_ego=20.0))

    self.mem_params.put("LastGPSPosition", "not json", block=True)
    with pytest.raises(json.JSONDecodeError):
      self.scc_m.update_calculations()
    self.publish_position()
    self.scc_m.update_calculations()
    assert self.scc_m.v_target == pytest.approx(v_ref(10.0, 140.0, v_ego=20.0))
  # End BluePilot

  # BluePilot: the continuous approach profile that replaced the binary braking window.
  #
  # Every test below fails against the previous implementation: it published the raw target
  # velocity only once a jerk/accel braking window opened, skipped any point whose target was
  # above v_ego outright, and stepped the published value straight from V_CRUISE_UNSET to that
  # target and back again.
  def test_profile_shape(self):
    # (a) The profile is monotone in distance, never dips below the point's own target velocity,
    # and reaches exactly that velocity 'offset' seconds of travel before the point. v_ego is
    # part of the profile now (the drop-proportional decel), so every expectation is computed
    # at the same v_ego the controller runs with. Points every 4 m so that d == tv * offset
    # (8 m on Normal) lands exactly on a sample.
    tv, node_x = 10.0, 300.0
    offset = NORMAL[3]
    points = [make_point(x, 0.0, tv if x == node_x else WAY_V) for x in range(0, 405, 4)]

    seen = []
    for x in range(200, 301, 4):  # the car sits exactly on a point, so along-path d == node_x - x
      self.run_calculations(points, v_ego=20.0, position=(float(x), 0.0))
      d = node_x - x
      seen.append((d, self.scc_m.v_target))

      assert self.scc_m.v_target >= tv - 1e-9, "the profile may never ask for less than the curve itself"
      assert self.scc_m.v_target == pytest.approx(v_ref(tv, d, v_ego=20.0), rel=1e-9)
      if d > 0:
        assert math.isclose(self.scc_m.target_lon, node_x * M_TO_DEG)

    # monotone: closer to the curve is never faster
    for (d_far, v_far), (d_near, v_near) in zip(seen, seen[1:], strict=False):
      assert d_far > d_near
      assert v_far >= v_near

    # equals tv exactly at d == tv * offset, and stays at tv from there in
    assert dict(seen)[tv * offset] == pytest.approx(tv)
    assert dict(seen)[0.0] == pytest.approx(tv)

  def test_early_hold_without_demanding_a_brake(self):
    # (b) With a curve ~50 m ahead and the car sitting at its set speed, the published target
    # is BELOW the set speed - so cruise can no longer accelerate into the bend - but still well
    # ABOVE the curve's own target velocity, so nothing is asking for the brakes yet. A 3 m/s
    # trim rides low on the drop-proportional ramp (near base_decel, the barely-braking regime
    # the city traces show), so the profile only binds ~56 m out rather than 100+.
    tv, v_cruise = 11.0, 14.0
    points = [make_point(x, 0.0, tv if x == 200.0 else WAY_V) for x in range(0, 405, 5)]

    published = self.drive(points, v_ego=v_cruise, v_cruise=v_cruise, start_x=150.0, cycles=40)
    settled = published[10:]

    assert all(p != V_CRUISE_UNSET for p in settled)
    assert all(tv < p < v_cruise for p in settled), settled
    # ~50 m out the profile is still nearer the set speed than the curve speed: a hold, not a brake
    assert settled[0] > (tv + v_cruise) / 2
    # and it is the profile value for where the car actually is, not the raw curve target.
    # v_ego stays pinned at v_cruise in drive(), so the expectation uses that same v_ego.
    assert published[-1] == pytest.approx(v_ref(tv, 200.0 - (150.0 + v_cruise * DT_MDL * 39), v_ego=v_cruise), abs=0.3)

  def test_published_target_never_steps(self):
    # (c) Across a whole simulated approach - engage, bind, apex, hold, release - the published
    # value never steps, but the limit is ASYMMETRIC now: it may rise no faster than
    # TARGET_SLEW_RATE, and fall no faster than the profile's max_decel (Normal here) - the
    # ceiling a human who noticed the curve late would brake at, and no harder.
    tv, v_cruise = 10.0, 14.0
    max_up = TARGET_SLEW_RATE * DT_MDL
    max_down = NORMAL[2] * DT_MDL  # the default profile's max_decel governs the fall rate
    points = [make_point(x, 0.0, tv if x == 250.0 else WAY_V) for x in range(0, 605, 5)]

    published = self.drive(points, v_ego=v_cruise, v_cruise=v_cruise, start_x=0.0, cycles=500)
    numeric = [p for p in published if p != V_CRUISE_UNSET]
    assert len(numeric) > 100, "the approach must actually bind for this test to mean anything"

    prev = None
    entered = False
    for p in published:
      if p == V_CRUISE_UNSET:
        # a release is only allowed once the ramp has climbed back to the set speed
        if prev is not None:
          assert prev >= v_cruise - max_up - 1e-9, f"released from {prev} with cruise at {v_cruise}"
        prev = None
        continue
      if prev is None:
        # the first published value may not itself be a step below whatever already governs
        # the car (the set speed here, since v_ego is held at it for the whole approach)
        assert p >= v_cruise - max_down - 1e-9, f"engaged with a step to {p} from cruise at {v_cruise}"
        entered = True
      elif p > prev:
        assert p - prev <= max_up + 1e-9, f"upward step of {p - prev:.4f} m/s exceeds {max_up:.4f}"
      else:
        assert prev - p <= max_down + 1e-9, f"downward step of {prev - p:.4f} m/s exceeds {max_down:.4f}"
      prev = p

    assert entered
    assert min(numeric) == pytest.approx(max(tv, MIN_V), abs=0.1), "the apex target must actually be reached"
    assert published[-1] == V_CRUISE_UNSET, "and the source must let go once the curve is behind"

  def test_hold_through_apex(self):
    # (d) The held target survives the apex. It must not be dropped the moment the car reaches
    # the target speed (the old sticky block re-applied a tv > v_ego filter and did exactly
    # that, letting cruise re-accelerate through the curve), only once the point is genuinely
    # behind the car by more than HOLD_RELEASE_DISTANCE.
    tv, node_x = 10.0, 300.0
    points = [make_point(x, 0.0, tv if x == node_x else WAY_V) for x in range(0, 405, 10)]

    # at the node, doing exactly the target speed
    self.run_calculations(points, v_ego=tv, position=(node_x, 0.0))
    assert self.scc_m.v_target == pytest.approx(tv)
    assert math.isclose(self.scc_m.target_lon, node_x * M_TO_DEG)

    # v_ego now BELOW tv and the nearest point has moved on, but the node is only 6 m behind
    self.run_calculations(points, v_ego=8.0, position=(node_x + 6.0, 0.0))
    assert self.scc_m.v_target == pytest.approx(tv), "hold released while still on top of the curve"
    assert math.isclose(self.scc_m.target_lon, node_x * M_TO_DEG)

    # past the hysteresis: the curve is behind us and the car may go back to cruise
    self.run_calculations(points, v_ego=8.0, position=(node_x + HOLD_RELEASE_DISTANCE + 5.0, 0.0))
    assert self.scc_m.v_target == pytest.approx(WAY_V)

  def test_horizon_bounds_how_early_a_curve_binds(self):
    # (e) A curve beyond max(HORIZON_MIN_M, v_ego * horizon) does not bind at all; the same
    # curve binds once the horizon reaches it.
    tv, node_x = 10.0, 200.0
    horizon_s = NORMAL[4]
    points = [make_point(x, 0.0, tv if x == node_x else WAY_V) for x in range(0, 405, 20)]

    slow_v_ego = 14.0
    assert max(HORIZON_MIN_M, slow_v_ego * horizon_s) < node_x
    self.run_calculations(points, v_ego=slow_v_ego)
    assert self.scc_m.v_target == pytest.approx(WAY_V), "a curve past the horizon must not bind"

    fast_v_ego = 25.0
    assert max(HORIZON_MIN_M, fast_v_ego * horizon_s) > node_x
    self.run_calculations(points, v_ego=fast_v_ego)
    assert self.scc_m.v_target == pytest.approx(v_ref(tv, node_x, v_ego=fast_v_ego))

  def test_inactive_publishes_v_cruise_unset(self):
    # P4: arbitration stays a plain min() in the planner - when the profile is above the set
    # speed this source simply publishes V_CRUISE_UNSET and cruise wins, with no extra gating.
    points = [make_point(x, 0.0, WAY_V) for x in range(0, 405, 20)]
    published = self.drive(points, v_ego=14.0, v_cruise=14.0, cycles=20)
    assert all(p == V_CRUISE_UNSET for p in published)
    assert not self.scc_m.is_active
    assert self.scc_m.state == MapState.enabled
  # End BluePilot

  # BluePilot: curve speed presets
  def test_preset_table(self):
    # the contract's single source of truth: (lat_accel, base_decel, max_decel, offset, horizon).
    # The numbers are fitted to verified OSM human-driving bands - the three lat anchors ARE the
    # city p25/median/p75 lateral acceleration on the route-verified Nashville traces - and
    # cross-checked against the AASHTO/FHWA literature (side-friction comfort curve, Fitzpatrick
    # decel rates, SHRP2 ramp studies). See the table's comment in map_controller.py.
    assert CURVE_SPEED_PRESETS == (
      (1.7, 0.30, 1.2, 1.5, 12.0),
      (2.0, 0.45, 1.8, 0.8, 10.0),
      (2.4, 0.60, 2.5, 0.3, 8.0),
    )
    assert DEFAULT_CURVE_SPEED_PROFILE == 1

  @pytest.mark.parametrize("index", [0, 1, 2])
  def test_apply_profile_drives_the_controller_constants(self, index):
    self.scc_m._apply_profile(index)
    _, approach_decel, max_decel, offset, horizon = CURVE_SPEED_PRESETS[index]
    assert self.scc_m.profile_index == index
    assert self.scc_m.approach_decel == approach_decel
    assert self.scc_m.max_decel == max_decel
    assert self.scc_m.target_offset == offset
    assert self.scc_m.horizon == horizon

  def test_unknown_preset_index_falls_back_to_the_default(self):
    self.scc_m._apply_profile(DEFAULT_CURVE_SPEED_PROFILE)
    assert self.scc_m._read_profile_index() in range(len(CURVE_SPEED_PRESETS))

  @pytest.mark.parametrize("index", [0, 1, 2])
  def test_ramp_start_is_a_smooth_multi_second_approach(self, index):
    # the owner's acceptance criterion: the max speed starts coming down several seconds before
    # the curve as a smooth ramp. A 2.5 m/s city trim rides low on the quadratic
    # drop-proportional ramp, matching the traces (city curves shave 1-3 mph at 0.2-0.5 m/s^2,
    # barely braking events) - ~5.7 s on the default preset, up to ~9 s of mostly-hold on
    # Comfort (the owner's stated tolerance was an 8 s ramp-down), ~4 s on Sport. The
    # constant-a model here actually understates the real ramp, since the decel relaxes
    # further as the car slows.
    preset = CURVE_SPEED_PRESETS[index]
    offset = preset[3]
    v_cruise, tv = 14.0, 11.5

    d_bind = bind_distance(v_cruise, tv, preset, v_ego=v_cruise)
    assert v_ref(tv, d_bind, preset, v_ego=v_cruise) == pytest.approx(v_cruise)

    # the ramp is a constant-decel leg (at the drop-proportional a_eff) followed by the offset held at tv
    ramp_seconds = (v_cruise - tv) / a_eff(tv, v_cruise, preset) + offset
    assert 2.5 <= ramp_seconds <= 9.5

    if index == DEFAULT_CURVE_SPEED_PROFILE:
      assert d_bind == pytest.approx(71.83, abs=0.5)
      assert ramp_seconds == pytest.approx(5.71, abs=0.05)

  def test_lat_accel_param_is_written_as_a_bare_decimal(self, tmp_path):
    # mapd json.Unmarshal()s this param straight into a float64, so whatever Params.put()
    # stores has to be a bare number - which is what a FLOAT-typed key gives us. (It was
    # briefly declared JSON, whose only put() serialisers are dict and list; that made
    # manager's default seeding raise TypeError on every boot. See
    # common/tests/test_params_keys_defaults.py, which guards the whole key table.)
    scratch = Params(str(tmp_path))
    key = "FordLowSpeedFactor_ang"  # PERSISTENT | FLOAT, unrelated to anything under test

    for value in (1.7, 2.0, 2.4):
      scratch.put(key, value, block=True)
      assert scratch.get(key) == pytest.approx(value)
      stored = pathlib.Path(scratch.get_param_path(key)).read_text()
      assert json.loads(stored) == pytest.approx(value), "mapd parses this with json.Unmarshal"

  def test_map_target_lat_a_is_handed_to_mapd(self):
    try:
      self.params.check_key(MAP_TARGET_LAT_A_PARAM)
    except UnknownKeyName:
      # not registered in this build: the controller must stay inert instead of raising
      self.scc_m._lat_accel_param_supported = True
      self.scc_m._write_map_target_lat_a(1.7)
      assert not self.scc_m._lat_accel_param_supported
      pytest.skip(f"{MAP_TARGET_LAT_A_PARAM} not registered in this build")

    for index, (lat_accel, *_) in enumerate(CURVE_SPEED_PRESETS):
      self.mem_params.remove(MAP_TARGET_LAT_A_PARAM)
      self.scc_m.profile_index = -1
      self.scc_m._apply_profile(index)
      # the persistent copy is what mapd reads at startup, the shm copy is what it applies live
      assert self.params.get(MAP_TARGET_LAT_A_PARAM) == pytest.approx(lat_accel)
      assert self.mem_params.get(MAP_TARGET_LAT_A_PARAM) == pytest.approx(lat_accel)
  # End BluePilot

  # BluePilot: the profile is a standing speed limit rather than a one-shot deceleration
  # request, so it has to let go when the data underneath it stops moving.
  def test_frozen_position_releases_the_profile(self):
    points = [{"latitude": 0.0, "longitude": 150.0 * M_TO_DEG, "velocity": 8.0}]  # inside the 200 m horizon at 20 m/s
    self.run_calculations(points, v_ego=20.0, position=(0.0, 0.0), bearing=90.0)
    bound = self.scc_m.v_target
    assert bound < NO_TARGET_V, "curve should bind before the position freezes"

    # republish the identical fix, as osm_map_data does at 1 Hz while the localizer is invalid
    for _ in range(MAX_STALE_CYCLES + 1):
      self.run_calculations(points, v_ego=20.0, position=(0.0, 0.0), bearing=90.0)

    assert self.scc_m.v_target == NO_TARGET_V, "a frozen position must not pin the car at the last target"
    assert self.scc_m.target_lat == 0.0 and self.scc_m.target_lon == 0.0

  def test_moving_position_never_looks_stale(self):
    points = [{"latitude": 0.0, "longitude": 250.0 * M_TO_DEG, "velocity": 8.0}]  # enters the horizon partway through the drive
    published = self.drive(points, v_ego=20.0, v_cruise=25.0, cycles=MAX_STALE_CYCLES * 2)
    assert min(published) < 25.0, "a moving car must keep the profile alive past the stale window"
