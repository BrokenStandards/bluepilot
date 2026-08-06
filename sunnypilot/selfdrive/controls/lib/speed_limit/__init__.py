"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
LIMIT_ADAPT_ACC = -1.  # m/s^2 Ideal acceleration for the adapting (braking) phase when approaching speed limits.
LIMIT_MAX_MAP_DATA_AGE = 10.  # s Maximum time to hold to map data, then consider it invalid inside limits controllers.
# s Grace period after the resolved limit becomes invalid before the held limit is announced as
# stale. Short coverage gaps (OSM holes, source flaps) are bridged silently within it. Past it the
# limit is still held — releasing it would snap the car back up to the cruise set speed — but the
# driver is told once with a downbeat chime that confidence in the limit is gone, and the sign
# greys the numeral (speedLimitValid false) for as long as it stays stale.
LIMIT_LAST_HOLD_TIME = 10.

# Speed Limit Assist constants
# BluePilot: under pcm long (openpilotLongitudinalControl + pcmCruise, e.g. Ford alpha long) the
# cluster set speed is only a ceiling — SLA tracks the limit in software. Any set speed at/above
# the limit works; this is the recommended ceiling so SLA has full range without the PCM's
# near-set-speed accel gate binding. It replaces the old exact-match 120/130 km/h requirement
# (PCM_LONG_REQUIRED_MAX_SET_SPEED), which had no hardware basis.
PCM_LONG_RECOMMENDED_SET_SPEED = {
  True: 36.1111,   # m/s, 130 km/h
  False: 35.7632,  # m/s, 80 mph
}

CONFIRM_SPEED_THRESHOLD = {
  True: 80,   # km/h
  False: 50,  # mph
}
