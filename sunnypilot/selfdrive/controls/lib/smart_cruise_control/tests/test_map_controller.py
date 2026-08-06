"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# BluePilot: json/math and the R/TO_DEGREES imports serve the metric point helpers below
import json
import math
import platform

from cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import R, TO_DEGREES, SmartCruiseControlMap
# End BluePilot

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState

# BluePilot: helpers for the along-path distance / reversal-truncation / placeholder tests
M_TO_DEG = TO_DEGREES / R  # meters to degrees of latitude (and of longitude on the equator)
NO_TARGET_V = 100.0  # min_v's initial value in update_calculations - left untouched when no braking target is selected


def make_point(x, y, v):
  # x is meters east and y is meters north of the origin; on the equator both axes convert with the same factor
  return {"latitude": y * M_TO_DEG, "longitude": x * M_TO_DEG, "velocity": v}
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
    x, y = position
    gps = {"latitude": y * M_TO_DEG, "longitude": x * M_TO_DEG}
    if bearing is not None:
      gps["bearing"] = bearing
    self.mem_params.put("LastGPSPosition", json.dumps(gps), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps(points), block=True)
    self.scc_m.v_ego = v_ego
    self.scc_m.a_ego = 0.0
    self.scc_m.update_calculations()
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

  # BluePilot: along-path distance, reversal-truncation and placeholder-skip coverage
  def test_single_curve_slowdown(self):
    # regression: a slow point straight ahead within the braking window still produces a target.
    # for v_ego=20, a_ego=0 and tv=10 the jerk/accel window works out to ~154.8 m
    points = [make_point(x, 0.0, 10.0 if x == 140 else 30.0) for x in range(0, 300, 20)]
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == 10.0
    assert math.isclose(self.scc_m.target_lon, 140 * M_TO_DEG)
    assert self.scc_m.target_lat == 0.0

  def test_looped_path_no_phantom_braking(self):
    # a chain that runs 300 m out, U-turns and comes back past the car used to trigger braking because
    # the doubled-back slow points are crow-flight close (~22 m); along-path they are ~580 m away and
    # the path is truncated at the reversal, so no target may be selected
    points = [make_point(x, 0.0, 30.0) for x in range(0, 350, 50)]
    points += [make_point(x, 10.0, 5.0) for x in (250, 150, 50, 20)]
    self.run_calculations(points, v_ego=25.0)
    assert self.scc_m.v_target == NO_TARGET_V
    assert self.scc_m.target_lat == 0.0
    assert self.scc_m.target_lon == 0.0

  def test_curved_path_uses_along_path_distance(self):
    # a horseshoe of 50 m segments with bearing changes of at most 45 degrees (so no truncation): the
    # slow endpoint is ~131 m crow-flight from the car (inside the ~154.8 m window for v_ego=20, tv=10)
    # but 400 m along-path, so it must not trigger early braking
    d45 = 50.0 / math.sqrt(2)
    xs_ys = [(0.0, 0.0), (50.0, 0.0), (100.0, 0.0), (100.0 + d45, d45), (100.0 + d45, d45 + 50.0),
             (100.0, 2 * d45 + 50.0), (50.0, 2 * d45 + 50.0), (0.0, 2 * d45 + 50.0), (-50.0, 2 * d45 + 50.0)]
    points = [make_point(x, y, 30.0) for x, y in xs_ys[:-1]] + [make_point(*xs_ys[-1], 10.0)]
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == NO_TARGET_V

  def test_truncation_at_bearing_reversal(self):
    # the chain reverses 180 degrees at 150 m; the slow points behind the reversal are only ~160 m
    # along-path (inside the ~279.8 m window for v_ego=25, tv=5), so only truncation of the forward
    # path at the reversal keeps them from becoming targets
    points = [make_point(x, 0.0, 30.0) for x in (0, 50, 100, 150)]
    points += [make_point(x, 0.0, 5.0) for x in (140, 90, 40)]
    self.run_calculations(points, v_ego=25.0)
    assert self.scc_m.v_target == NO_TARGET_V
    assert self.scc_m.target_lat == 0.0
    assert self.scc_m.target_lon == 0.0

  def test_truncation_at_uturn_apex_first_segment(self):
    # the nearest point IS the U-turn apex: the chain doubles back from the very first forward
    # segment, so only the vehicle-bearing seed can catch the reversal. The doubled-back slow
    # points are 30-130 m along-path (inside the ~279.8 m window for v_ego=25, tv=5); the car
    # heads east (bearing 90) while segment one heads west -> truncated, no braking
    points = [make_point(0.0, 0.0, 30.0)]
    points += [make_point(-x, 2.0, 5.0) for x in (30, 80, 130)]
    self.run_calculations(points, v_ego=25.0, bearing=90.0)
    assert self.scc_m.v_target == NO_TARGET_V
    assert self.scc_m.target_lat == 0.0
    assert self.scc_m.target_lon == 0.0

  def test_missing_bearing_skips_first_segment_check(self):
    # without a 'bearing' field in LastGPSPosition the first-segment check is skipped gracefully:
    # the same apex chain as above is walked without truncation (pre-seed behavior) and the
    # doubled-back slow points become a target instead of crashing
    points = [make_point(0.0, 0.0, 30.0)]
    points += [make_point(-x, 2.0, 5.0) for x in (30, 80, 130)]
    self.run_calculations(points, v_ego=25.0)
    assert self.scc_m.v_target == 5.0

  def test_forward_path_with_bearing_still_brakes(self):
    # the vehicle-bearing seed must not truncate a normal forward path: car heading east with the
    # chain heading east keeps the slow point at x=140 as the braking target
    points = [make_point(x, 0.0, 10.0 if x == 140 else 30.0) for x in range(0, 300, 20)]
    self.run_calculations(points, v_ego=20.0, bearing=90.0)
    assert self.scc_m.v_target == 10.0
    assert math.isclose(self.scc_m.target_lon, 140 * M_TO_DEG)

  def test_placeholder_entries_skipped(self):
    # mapd's GetTargetVelocities leaves (0,0,0) placeholder entries for zero-curvature points; a
    # placeholder mid-chain sits at the origin, so the walk would see a fake reversal at the
    # segment into it (and huge along-path detours). Skipping placeholders keeps the real slow
    # point at x=140 braking correctly
    points = [make_point(x, 0.0, 10.0 if x == 140 else 30.0) for x in range(0, 300, 20)]
    points.insert(4, {"latitude": 0.0, "longitude": 0.0, "velocity": 0.0})
    self.run_calculations(points, v_ego=20.0)
    assert self.scc_m.v_target == 10.0
    assert math.isclose(self.scc_m.target_lon, 140 * M_TO_DEG)
    assert self.scc_m.target_lat == 0.0
  # End BluePilot
