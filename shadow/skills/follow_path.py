#!/usr/bin/env python3
"""
FollowPathSkill - fly a chain of waypoints in order, flowing through them
instead of stopping at each one. Tier 2. Added Nov 8, 2026.

The one idea that makes this different from chaining go_to: switch to
the next waypoint as soon as Shadow is within ACCEPT_RADIUS_M of the
current one, instead of waiting to arrive. Only the LAST waypoint needs
real arrival. ArduPilot still slows near each target it is given, so
this is smoother than go_to-go_to-go_to but not perfectly smooth; that
leftover is exactly what a trained policy (the Waypoint Circuit
training task) could earn this skill's slot by beating.

Two built-in paths (pick with run_skill.py ... follow_path --path X):
  zigzag  (default) - the 8-point lawnmower SEARCH_WAYPOINTS sweep from
          deliver_payload.py, the zigzag in the Mission module. This is
          the search pattern the delivery mission flies.
  circuit - the 6-point loop from learned_flight.py's
            DEFAULT_WAYPOINTS_XY / agility_circuit_env.py, i.e. the same
            course the Waypoint Circuit training task practises.
Both are COPIED from those files (not imported - deliver_payload pulls in
Gazebo bindings this skill has no use for). Keep in sync by hand.

Flown at PATH_ALTITUDE_M (2.5 m): Nov 8 live test showed Shadow sagging
into the ground on a long go_to leg at 1 m - it hovers at roughly 70%
throttle, so a hard tilt to accelerate costs height. Speed is capped by
WP_SPD in config/shadow_sitl.parm.
"""

import math

from .base import Skill
from .heading import NoseFirst

PATH_ALTITUDE_M = 2.5
ACCEPT_RADIUS_M = 1.0   # roll on to the next waypoint inside this (3D distance)
FINAL_RADIUS_M = 0.5    # the last waypoint needs real arrival

PATHS = {
    'zigzag': [
        (-7.0, 0.0), (7.0, 0.0),
        (7.0, 3.0), (-7.0, 3.0),
        (-7.0, 6.0), (7.0, 6.0),
        (7.0, 9.0), (-7.0, 9.0),
    ],
    'circuit': [
        (6.0, 2.0), (1.0, 9.0), (-5.0, 7.0),
        (-6.0, -3.0), (0.0, -4.0), (3.0, -4.0),
    ],
}


class FollowPathSkill(Skill):
    name = 'follow_path'
    # Zigzag is ~65 m of path plus the leg out to its start. At WP_SPD
    # 3 m/s that's ~25-35 s of cruising, plus slow-downs near each
    # corner. 150 s leaves real margin without hiding a stuck run.
    timeout_sec = 150.0

    def __init__(self, body, path='zigzag'):
        super().__init__(body)
        if path not in PATHS:
            raise ValueError(f'unknown path {path!r}; choose from {sorted(PATHS)}')
        self.path_name = path
        self.waypoints = [(x, y, PATH_ALTITUDE_M) for x, y in PATHS[path]]
        self.i = 0

    def requirements(self):
        if not self.body.wait_until_connected(timeout_sec=10.0):
            self.body.get_logger().warn('follow_path: requirements failed - no MAVROS link.')
            return False
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            self.body.get_logger().warn('follow_path: requirements failed - no position estimate.')
            return False
        # Same 3 s poll go_to uses, so one early state message can't fail it.
        import rclpy
        deadline = self.body.get_clock().now().nanoseconds + int(3.0 * 1e9)
        armed, mode = False, ''
        while rclpy.ok():
            armed, mode = self.body.is_armed(), self.body.current_mode()
            if armed and mode == 'GUIDED':
                return True
            if self.body.get_clock().now().nanoseconds > deadline:
                break
            rclpy.spin_once(self.body, timeout_sec=0.2)
        self.body.get_logger().warn(
            f'follow_path: requirements failed - armed={armed}, mode={mode!r} '
            '(press Arm then Takeoff first).')
        return False

    def limits(self):
        # Base check covers the drone's live position; also refuse a path
        # that would leave the geofence before flying any of it.
        from shadow.limits import within_limits
        if not all(within_limits(wp) for wp in self.waypoints):
            self.body.get_logger().warn('follow_path: a waypoint is outside the geofence.')
            return False
        return super().limits()

    def start(self):
        self.i = 0
        self.body.get_logger().info(
            f'follow_path: flying "{self.path_name}" ({len(self.waypoints)} waypoints) '
            f'at {PATH_ALTITUDE_M} m.')
        # Nov 8, 2026 - nose-first, turning WHILE flying (no stop at corners).
        self._nose = NoseFirst(self.body, turn_first=False)
        sx, sy, sz, yaw = self._nose.setpoint(self.waypoints[0])
        self.body.send_position_setpoint(sx, sy, sz, yaw=yaw)

    def update(self):
        target = self.waypoints[self.i]
        # Re-send every tick - GUIDED expects a steady setpoint stream.
        sx, sy, sz, yaw = self._nose.setpoint(target)
        self.body.send_position_setpoint(sx, sy, sz, yaw=yaw)
        pos = self.body.current_position()
        if pos is None:
            return False
        d = math.dist(pos[:3], target)
        last = self.i == len(self.waypoints) - 1
        if last:
            return d <= FINAL_RADIUS_M
        if d <= ACCEPT_RADIUS_M:
            self.i += 1
            self.body.get_logger().info(
                f'follow_path: waypoint {self.i}/{len(self.waypoints)} passed, '
                f'heading to {self.waypoints[self.i][:2]}.')
            sx, sy, sz, yaw = self._nose.setpoint(self.waypoints[self.i])
            self.body.send_position_setpoint(sx, sy, sz, yaw=yaw)
        return False
