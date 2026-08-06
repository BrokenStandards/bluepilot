"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

BluePilot: tests for the mapd_bp exchange in live_map_data — the speed-limit guess
publish path and the MapdCarContext mem param.
"""
import pytest
from pytest_mock import MockerFixture

from openpilot.common.params import Params
from openpilot.sunnypilot.mapd.live_map_data.osm_map_data import OsmMapData, PARAM_REFRESH_TICKS


def create_mock(properties, mocker: MockerFixture):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker: MockerFixture, v_ego: float = 10.0, yaw_rate: float = 0.0,
                  desired_curvature: float = 0.0, controls_state_seen: bool = True):
  live_location_kalman = create_mock({
    'gpsOK': True,
    'velocityCalibrated': create_mock({'value': [v_ego, 0.0, 0.0]}, mocker),
    'angularVelocityCalibrated': create_mock({'value': [0.0, 0.0, yaw_rate]}, mocker),
  }, mocker)
  controls_state = create_mock({
    'desiredCurvature': desired_curvature,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'liveLocationKalman': live_location_kalman,
    'controlsState': controls_state,
  }[key]
  sm_mock.seen = {'liveLocationKalman': True, 'controlsState': controls_state_seen}
  return sm_mock


@pytest.fixture
def osm_map_data(mocker: MockerFixture):
  mocker.patch('openpilot.sunnypilot.mapd.live_map_data.base_map_data.messaging.SubMaster')
  mocker.patch('openpilot.sunnypilot.mapd.live_map_data.base_map_data.messaging.PubMaster')
  osm = OsmMapData()
  osm.sm = setup_sm_mock(mocker)
  # ensure the localizer-dependent state stays inert so tests exercise only the mem param exchange
  osm.last_position = None
  return osm


def published_live_map_data(osm: OsmMapData):
  osm.publish()
  msg = osm.pm.send.call_args[0][1]
  return msg.liveMapDataSP


class TestSpeedLimitGuessPublish:
  def test_guessed_limit_published_when_untagged(self, osm_map_data):
    osm_map_data.mem_params.put("MapSpeedLimit", 0.0, block=True)
    osm_map_data.mem_params.put("MapSpeedLimitGuess", {
      "speedlimit": 12.5, "source": "backward",
      "backward_value": 12.5, "backward_distance": 800.0,
      "forward_value": 0.0, "forward_distance": 0.0,
    }, block=True)

    live_map_data = published_live_map_data(osm_map_data)

    assert live_map_data.speedLimit == 12.5
    assert live_map_data.speedLimitValid  # guessed limits count as valid so resolver/SLA/HUD work
    assert live_map_data.speedLimitGuessed
    assert live_map_data.speedLimitGuessedSource == "backward"

  def test_tagged_limit_takes_precedence_over_guess(self, osm_map_data):
    osm_map_data.mem_params.put("MapSpeedLimit", 20.0, block=True)
    osm_map_data.mem_params.put("MapSpeedLimitGuess", {"speedlimit": 12.5, "source": "both"}, block=True)

    live_map_data = published_live_map_data(osm_map_data)

    assert live_map_data.speedLimit == 20.0
    assert live_map_data.speedLimitValid
    assert not live_map_data.speedLimitGuessed
    assert live_map_data.speedLimitGuessedSource == ""

  def test_no_tag_and_no_guess_publishes_invalid(self, osm_map_data):
    osm_map_data.mem_params.put("MapSpeedLimit", 0.0, block=True)

    live_map_data = published_live_map_data(osm_map_data)

    assert live_map_data.speedLimit == 0.0
    assert not live_map_data.speedLimitValid
    assert not live_map_data.speedLimitGuessed
    assert live_map_data.speedLimitGuessedSource == ""

  def test_zero_guess_is_no_guess(self, osm_map_data):
    osm_map_data.mem_params.put("MapSpeedLimit", 0.0, block=True)
    osm_map_data.mem_params.put("MapSpeedLimitGuess", {"speedlimit": 0.0, "source": ""}, block=True)

    live_map_data = published_live_map_data(osm_map_data)

    assert not live_map_data.speedLimitValid
    assert not live_map_data.speedLimitGuessed


class TestMapdCarContext:
  def read_context(self, osm: OsmMapData) -> dict:
    context = osm.mem_params.get("MapdCarContext")
    assert context is not None
    return context

  def test_context_content(self, osm_map_data, mocker):
    osm_map_data.params.put_bool("VisualRoutingAssist", True, block=True)
    osm_map_data.sm = setup_sm_mock(mocker, v_ego=10.0, yaw_rate=0.02, desired_curvature=0.003)

    osm_map_data.update_location()
    context = self.read_context(osm_map_data)

    assert context['enabled'] is True
    assert context['v_ego'] == pytest.approx(10.0)
    assert context['yaw_rate'] == pytest.approx(0.02)
    assert context['curvature'] == pytest.approx(0.002)  # yaw_rate / max(v_ego, 1)
    assert context['desired_curvature'] == pytest.approx(0.003)

  def test_curvature_speed_floor(self, osm_map_data, mocker):
    # below 1 m/s the curvature denominator is floored at 1 to keep the ratio bounded
    osm_map_data.sm = setup_sm_mock(mocker, v_ego=0.2, yaw_rate=-0.05)

    osm_map_data.update_location()
    context = self.read_context(osm_map_data)

    assert context['curvature'] == pytest.approx(-0.05)

  def test_desired_curvature_zero_when_unavailable(self, osm_map_data, mocker):
    osm_map_data.sm = setup_sm_mock(mocker, desired_curvature=0.01, controls_state_seen=False)

    osm_map_data.update_location()
    context = self.read_context(osm_map_data)

    assert context['desired_curvature'] == 0.0

  def test_enabled_gating(self, osm_map_data):
    osm_map_data.params.put_bool("VisualRoutingAssist", False, block=True)

    osm_map_data.update_location()
    context = self.read_context(osm_map_data)

    assert context['enabled'] is False

  def test_enabled_refreshes_at_param_cadence(self, osm_map_data):
    osm_map_data.params.put_bool("VisualRoutingAssist", False, block=True)
    osm_map_data.update_location()
    assert self.read_context(osm_map_data)['enabled'] is False

    # param flips between refreshes: cached value holds until the next ~5 s boundary
    osm_map_data.params.put_bool("VisualRoutingAssist", True, block=True)
    for _ in range(PARAM_REFRESH_TICKS - 1):
      osm_map_data.update_location()
      assert self.read_context(osm_map_data)['enabled'] is False

    osm_map_data.update_location()
    assert self.read_context(osm_map_data)['enabled'] is True


class TestMemParamRegistration:
  def test_visual_routing_assist_registered_off_by_default(self):
    assert Params().get_bool("VisualRoutingAssist") is False
