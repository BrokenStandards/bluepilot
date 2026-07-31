#!/usr/bin/env python3
"""Report engagement state and blocking events. Run while the stack is up.

Subscribes briefly to selfdriveState + onroadEvents and prints enabled/active/
engageable plus every current event name, so a non-engaging profile run is
diagnosable from its log instead of needing a live session.
"""
import time

import cereal.messaging as messaging


def main():
  sm = messaging.SubMaster(['selfdriveState', 'onroadEvents', 'managerState'])
  deadline = time.monotonic() + 5.0
  while time.monotonic() < deadline:
    sm.update(100)
    if sm.seen['selfdriveState'] and sm.seen['onroadEvents']:
      break

  ss = sm['selfdriveState']
  print(f"selfdriveState: enabled={ss.enabled} active={ss.active} engageable={ss.engageable} state={ss.state} alertText1={ss.alertText1!r}")
  events = [(e.name, {'noEntry': e.noEntry, 'softDisable': e.softDisable, 'immediateDisable': e.immediateDisable})
            for e in sm['onroadEvents']]
  print(f"onroadEvents: {events if events else '(none)'}")
  if sm.seen['managerState']:
    not_running = [p.name for p in sm['managerState'].processes if p.shouldBeRunning and not p.running]
    print(f"managerState not_running: {not_running if not_running else '(none)'}")


if __name__ == "__main__":
  main()
