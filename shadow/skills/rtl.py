#!/usr/bin/env python3
"""
RtlSkill - Return To Launch: fly home and land there, using ArduCopter's
own RTL mode. Tier 2. Added Nov 8, 2026.

Land vs RTL, kept as two separate skills on purpose (same split every
autopilot uses): LAND = come down right where you are; RTL = go back to
home first, then come down there. Missions end with RTL; LAND is for
"stop now".

ArduPilot does all the flying: climbs to RTL_ALT_M if lower, flies home,
pauses briefly, descends, and auto-disarms on touchdown. This skill just
switches the mode and then CHECKS the result, so a pass means "really
landed at home", not "the mode command was accepted".

"Home" is wherever Shadow was ARMED, not where it spawned. ArduPilot sets
home at arming. Arm at the spawn point and the two are the same; land
somewhere else and re-arm there, and home moves to that spot. So this
reads ArduPilot's real home position (/mavros/home_position/home) and
checks against that. If that message never arrives, it falls back to the
local origin (the spawn point) and says so in the log.

Parameters live in config/shadow_sitl.parm (ArduPilot 4.7+ names, metres):
RTL_ALT_M 3.0 keeps the return height inside shadow/limits.py's 6 m
ceiling - the old default (15 m) would trip limits() mid-return.
"""

import math
import time

from .base import Skill

HOME_TOLERANCE_M = 1.5   # horizontal distance from home that counts as "landed at home"


class RtlSkill(Skill):
    name = 'rtl'
    # Farthest point in the runway world is ~11.4 m out. At WP_SPD 3 m/s:
    # climb + ~4 s transit + pause + ~6-8 s descent. 90 s is generous.
    timeout_sec = 90.0

    def __init__(self, body):
        super().__init__(body)
        self._home = None
        self._home_sub = None
        self._landed_far = None

    def _home_cb(self, msg):
        try:
            self._home = (msg.position.x, msg.position.y)
        except AttributeError:
            pass

    def requirements(self):
        if not self.body.wait_until_connected(timeout_sec=10.0):
            self.body.get_logger().warn('rtl: requirements failed - no MAVROS link.')
            return False
        if not self.body.is_armed():
            self.body.get_logger().warn('rtl: requirements failed - not armed (nothing to return).')
            return False
        # Listen for ArduPilot's real home position. Give it a moment to
        # arrive before starting; fall back to the origin if it doesn't.
        try:
            import rclpy
            from mavros_msgs.msg import HomePosition
            self._home_sub = self.body.create_subscription(
                HomePosition, '/mavros/home_position/home', self._home_cb, 10)
            deadline = time.monotonic() + 3.0
            while self._home is None and time.monotonic() < deadline:
                rclpy.spin_once(self.body, timeout_sec=0.2)
        except Exception as exc:  # message type missing etc. - fall back, don't fail
            self.body.get_logger().warn(f'rtl: could not subscribe to home position ({exc}).')
        return True

    def start(self):
        if self._home is None:
            self._home = (0.0, 0.0)
            self.body.get_logger().warn(
                'rtl: no home position from ArduPilot - checking against the spawn '
                'point (0, 0). Correct only if Shadow was armed there.')
        else:
            self.body.get_logger().info(
                f'rtl: home is ({self._home[0]:.2f}, {self._home[1]:.2f}).')
        self.body.set_mode('RTL')

    def _dist_home(self):
        pos = self.body.current_position()
        if pos is None:
            return None
        return math.hypot(pos[0] - self._home[0], pos[1] - self._home[1])

    def limits(self):
        if not super().limits():
            return False
        # Disarmed (landed) but NOT at home = RTL went wrong. Fail rather
        # than call it a pass.
        if not self.body.is_armed():
            d = self._dist_home()
            if d is not None and d > HOME_TOLERANCE_M:
                self.body.get_logger().warn(
                    f'rtl: landed {d:.2f} m from home (limit {HOME_TOLERANCE_M} m).')
                return False
        return True

    def update(self):
        if self.body.is_armed():
            return False
        d = self._dist_home()
        if d is not None:
            self.body.get_logger().info(f'rtl: landed {d:.2f} m from home.')
        return d is not None and d <= HOME_TOLERANCE_M
