"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

import pytest
from pytest_mock import MockerFixture

from cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def create_mock(properties, mocker: MockerFixture):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker: MockerFixture, gps_fix_age: float = 0., map_limit: float | None = None,
                  ahead_limit: float = 0., ahead_distance: float = 0.):
  cruise_speed_limit = random.uniform(0, 120)
  live_map_data_limit = map_limit if map_limit is not None else random.uniform(0, 120)

  car_state = create_mock({
    'gasPressed': False,
    'brakePressed': False,
    'standstill': False,
  }, mocker)
  car_state_sp = create_mock({
    'speedLimit': cruise_speed_limit,
  }, mocker)
  live_map_data = create_mock({
    'speedLimit': live_map_data_limit,
    'speedLimitValid': True,
    'speedLimitAhead': ahead_limit,
    'speedLimitAheadValid': ahead_limit > 0.,
    'speedLimitAheadDistance': ahead_distance,
  }, mocker)
  gps_data = create_mock({
    # Production publishers (ubloxd, qcomgpsd, the sim) fill unixTimestampMillis with Unix
    # epoch wall time, so the fixture must model that clock domain, not time.monotonic().
    'unixTimestampMillis': (time.time() - gps_fix_age) * 1e3,  # noqa: TID251
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
    'gpsLocation': gps_data,
  }[key]
  return sm_mock


parametrized_policies = pytest.mark.parametrize(
  "policy, sm_key, function_key", [
    (Policy.car_state_only, 'carStateSP', SpeedLimitSource.car),
    (Policy.car_state_priority, 'carStateSP', SpeedLimitSource.car),
    (Policy.map_data_only, 'liveMapDataSP', SpeedLimitSource.map),
    (Policy.map_data_priority, 'liveMapDataSP', SpeedLimitSource.map),
  ],
  ids=lambda val: val.name if hasattr(val, 'name') else str(val)
)


@pytest.mark.parametrize("resolver_class", [SpeedLimitResolver])
class TestSpeedLimitResolverValidation:

  @pytest.mark.parametrize("policy", list(Policy), ids=lambda policy: policy.name)
  def test_initial_state(self, resolver_class, policy):
    resolver = resolver_class()
    resolver.policy = policy
    for source in ALL_SOURCES:
      if source in resolver.limit_solutions:
        assert resolver.limit_solutions[source] == 0.
        assert resolver.distance_solutions[source] == 0.

  @parametrized_policies
  def test_resolver(self, resolver_class, policy, sm_key, function_key, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
    socket_to_source = {'carStateSP': SpeedLimitSource.car, 'liveMapDataSP': SpeedLimitSource.map}
    minimum_key, minimum_speed_limit = min(
      ((key, sm_mock[key].speedLimit) for key in
       socket_to_source.keys()), key=lambda x: x[1])

    # Assert the resolver
    resolver.update(minimum_speed_limit, sm_mock)
    assert resolver.speed_limit == minimum_speed_limit
    assert resolver.source == socket_to_source[minimum_key]

  @parametrized_policies
  def test_parser(self, resolver_class, policy, sm_key, function_key, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the parsing
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.limit_solutions[ALL_SOURCES[function_key]] == source_speed_limit
    assert resolver.distance_solutions[ALL_SOURCES[function_key]] == 0.

  @pytest.mark.parametrize("policy", list(Policy), ids=lambda policy: policy.name)
  def test_resolve_interaction_in_update(self, resolver_class, policy, mocker: MockerFixture):
    v_ego = 50
    resolver = resolver_class()
    resolver.policy = policy

    sm_mock = setup_sm_mock(mocker)
    resolver.update(v_ego, sm_mock)

    # After resolution
    assert resolver.speed_limit is not None
    assert resolver.distance is not None
    assert resolver.source is not None

  @pytest.mark.parametrize("policy", list(Policy), ids=lambda policy: policy.name)
  def test_old_map_data_ignored(self, resolver_class, policy, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = policy
    resolver.v_ego = 27.8
    sm_mock = setup_sm_mock(mocker, gps_fix_age=2 * LIMIT_MAX_MAP_DATA_AGE, map_limit=25.)
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.


@pytest.mark.parametrize("resolver_class", [SpeedLimitResolver])
class TestSpeedLimitResolverMapDataClock:
  """The map-data guards compare the GPS fix time (Unix epoch wall time in
  unixTimestampMillis) against the current time. These tests feed production-faithful
  epoch timestamps; comparing that field against time.monotonic() makes the fix age
  hugely negative, silently disabling both the staleness guard and the upcoming-limit
  adaptation."""

  def test_fresh_gps_fix_accepts_map_data(self, resolver_class, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    resolver.v_ego = 27.8
    sm_mock = setup_sm_mock(mocker, gps_fix_age=0., map_limit=25.)
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 25.

  def test_no_gps_fix_rejects_map_data(self, resolver_class, mocker: MockerFixture):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    resolver.v_ego = 27.8
    sm_mock = setup_sm_mock(mocker, map_limit=25.)
    sm_mock['gpsLocation'].unixTimestampMillis = 0
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.

  def test_ahead_limit_adopted_within_adapt_distance(self, resolver_class, mocker: MockerFixture):
    # 100 km/h now, 50 km/h posted 50 m ahead: braking at LIMIT_ADAPT_ACC needs ~290 m,
    # so the resolver must adopt the upcoming limit and expose the distance to it.
    v_ego = 27.8
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    sm_mock = setup_sm_mock(mocker, map_limit=27.8, ahead_limit=13.9, ahead_distance=50.)
    resolver.update(v_ego, sm_mock)
    assert resolver.speed_limit == pytest.approx(13.9, abs=0.01)
    assert resolver.distance == pytest.approx(50., abs=5.)

  def test_ahead_limit_ignored_beyond_adapt_distance(self, resolver_class, mocker: MockerFixture):
    v_ego = 27.8
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    sm_mock = setup_sm_mock(mocker, map_limit=27.8, ahead_limit=13.9, ahead_distance=5000.)
    resolver.update(v_ego, sm_mock)
    assert resolver.speed_limit == pytest.approx(27.8, abs=0.01)
    assert resolver.distance == 0.


@pytest.mark.parametrize("resolver_class", [SpeedLimitResolver])
class TestSpeedLimitResolverLastLimitHold:
  """speed_limit_last bridges short gaps in coverage (OSM holes, source flaps), but must
  not persist a stale limit indefinitely: SLA feeds speed_limit_final_last into the
  planner's min() whenever it is enabled, so a limit latched forever keeps capping the
  car long after leaving the road it was posted on."""

  def _run_gap_frames(self, resolver, sm_mock, frames):
    sm_mock['liveMapDataSP'].speedLimitValid = False
    sm_mock['liveMapDataSP'].speedLimit = 0.
    for _ in range(frames):
      resolver.update(27.8, sm_mock)

  def _make_resolver_with_limit(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    mocker.patch.object(resolver, 'update_params')
    sm_mock = setup_sm_mock(mocker, map_limit=25.)
    resolver.update(27.8, sm_mock)
    assert resolver.speed_limit_last == 25.
    return resolver, sm_mock

  def test_last_limit_held_through_short_gap(self, resolver_class, mocker: MockerFixture):
    resolver, sm_mock = self._make_resolver_with_limit(resolver_class, mocker)
    self._run_gap_frames(resolver, sm_mock, 40)  # 2 s at DT_MDL
    assert resolver.speed_limit_last == 25.
    assert resolver.speed_limit_last_valid
    assert not resolver.speed_limit_stale  # inside the grace period: bridged silently

  def test_last_limit_is_kept_past_hold_period_and_marked_stale(self, resolver_class, mocker: MockerFixture):
    # zeroing the held limit made SLA release the cap, snapping the car back up to the cruise
    # set speed on every OSM hole. The limit is kept and flagged stale instead.
    resolver, sm_mock = self._make_resolver_with_limit(resolver_class, mocker)
    self._run_gap_frames(resolver, sm_mock, 240)  # 12 s at DT_MDL, past the 10 s grace period
    assert resolver.speed_limit_last == 25.
    assert resolver.speed_limit_final_last == 25.
    assert resolver.speed_limit_last_valid
    assert resolver.speed_limit_stale
    assert not resolver.speed_limit_valid  # live data is gone: the sign greys the numeral

  def test_stale_clears_when_data_returns(self, resolver_class, mocker: MockerFixture):
    resolver, sm_mock = self._make_resolver_with_limit(resolver_class, mocker)
    self._run_gap_frames(resolver, sm_mock, 240)
    assert resolver.speed_limit_stale
    sm_mock['liveMapDataSP'].speedLimitValid = True
    sm_mock['liveMapDataSP'].speedLimit = 25.
    resolver.update(27.8, sm_mock)
    assert not resolver.speed_limit_stale
    assert resolver.speed_limit_valid

  def test_limit_reacquired_after_expiry(self, resolver_class, mocker: MockerFixture):
    resolver, sm_mock = self._make_resolver_with_limit(resolver_class, mocker)
    self._run_gap_frames(resolver, sm_mock, 240)
    assert resolver.speed_limit_last == 25.  # held, not zeroed
    sm_mock['liveMapDataSP'].speedLimitValid = True
    sm_mock['liveMapDataSP'].speedLimit = 19.4
    resolver.update(27.8, sm_mock)
    assert resolver.speed_limit_last == pytest.approx(19.4)
    assert resolver.speed_limit_last_valid
