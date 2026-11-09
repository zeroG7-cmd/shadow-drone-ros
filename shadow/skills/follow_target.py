#!/usr/bin/env python3
"""
FollowTargetSkill - keep a moving object in view and stay a set distance
behind it. Tier 3, Perception. Added Nov 9, 2026.

Same camera-to-ground fusion as approach_target (GroundTargetSensor), run
continuously: every tick the target's ground position is estimated
(smoothed), and Shadow is sent to the point FOLLOW_DIST_M back from it
along the line between them, nose pointing at it. That's "position
follow"; it keeps the object in the picture because at FOLLOW_ALT_M the
camera sees the ground from ~3.2 m out, inside FOLLOW_DIST_M.

Lost it: hold position and keep looking where it was for LOST_HOLD_S,
then turn on the spot to re-find it. Lost for LOST_FAIL_S in total = fail.

Test (--move-target, the Sim Bay button does this): starts
scripts/move_target.py, which slides payload_orange_1 back and forth
along y = 2 between x = 6 and x = 0 at walking pace and logs its true
position. Pass after FOLLOW_S seconds if:
  - the target was in view at least MIN_IN_VIEW of the time,
  - the target really moved at least MIN_TRAVEL_M (otherwise it's
    a hover test),
  - Shadow's estimate of where it was stayed within TRUTH_TOL_M of
    its true position on average (it followed the right thing),
  - Shadow kept between DIST_MIN_M and DIST_MAX_M from it on average.
Without the mover (real drone, or a manual target) only the in-view
check applies.
"""

import math
import os
import subprocess
import time

from .base import Skill
from .heading import wrap
from . import sim_truth
from shadow.senses.ground_target import GroundTargetSensor

FOLLOW_DIST_M = 4.0
FOLLOW_ALT_M = 2.0
FOLLOW_S = 40.0
EMA = 0.35
LOST_HOLD_S = 1.5
LOST_FAIL_S = 10.0
ACQUIRE_FAIL_S = 15.0       # one full turn at SCAN_RATE_DPS is 12 s
SCAN_RATE_DPS = 30.0
MIN_IN_VIEW = 0.8
MIN_TRAVEL_M = 5.0
TRUTH_TOL_M = 0.8
DIST_MIN_M, DIST_MAX_M = 2.5, 6.0
MOVER = os.path.expanduser('~/shadow_ws/scripts/move_target.py')


class FollowTargetSkill(Skill):
    name = 'follow_target'
    timeout_sec = FOLLOW_S + 40.0

    def __init__(self, body, move_target=False):
        super().__init__(body)
        self.sensor = GroundTargetSensor(body, source='orange')
        self.move_target = move_target
        self.failed = None
        self._mover = None
        self.est = None
        self.yaw = None
        self._t0 = None
        self._lost_total = 0.0
        self._last_tick = None
        self._ticks = self._in_view = 0
        self._errs, self._dists = [], []
        self._travel = 0.0

    def requirements(self):
        if not self.body.wait_until_connected(timeout_sec=10.0):
            return False
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            return False
        import rclpy
        deadline = time.time() + 3.0
        while rclpy.ok():
            if self.body.is_armed() and self.body.current_mode() == 'GUIDED':
                return True
            if time.time() > deadline:
                break
            rclpy.spin_once(self.body, timeout_sec=0.2)
        self.body.get_logger().warn('follow_target: needs armed + GUIDED (Arm, Takeoff first).')
        return False

    def start(self):
        if self.move_target:
            self._mover = subprocess.Popen(['python3', MOVER], stdout=subprocess.DEVNULL,
                                           stderr=subprocess.DEVNULL)
            self.body.get_logger().info('follow_target: started move_target.py (payload_orange_1 will move).')
        self.yaw = self.body.current_yaw() or 0.0
        self._t0 = None             # follow clock starts at first sighting
        self._start_t = self._last_tick = time.time()
        self.body.get_logger().info('follow_target: looking for the orange payload.')

    def _stop_mover(self):
        if self._mover is not None and self._mover.poll() is None:
            self._mover.terminate()   # move_target.py puts the payload back on SIGTERM
            try:
                self._mover.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._mover.kill()
        self._mover = None

    def update(self):
        try:
            return self._update()
        except Exception:
            self._stop_mover()
            raise

    def _update(self):
        pos = self.body.current_position()
        if pos is None:
            return False
        now = time.time()
        dt, self._last_tick = now - self._last_tick, now

        if self._t0 is None:
            # ACQUIRE: turn on the spot until the target is seen
            if self.sensor.latest(max_age_s=0.4) is None:
                if now - self._start_t > ACQUIRE_FAIL_S:
                    self._stop_mover()
                    self.failed = 'never saw the orange payload (one full turn)'
                    return False
                self.yaw = wrap(self.yaw + math.radians(SCAN_RATE_DPS) * dt)
                self.body.send_position_setpoint(pos[0], pos[1], FOLLOW_ALT_M, yaw=self.yaw)
                return False
            self._t0 = now
            self.body.get_logger().info(f'follow_target: got it, following for {FOLLOW_S:.0f} s at {FOLLOW_DIST_M} m.')
        self._ticks += 1

        hit = self.sensor.latest(max_age_s=0.4)
        if hit is not None:
            self._in_view += 1
            self.est = hit if self.est is None else (
                self.est[0] + EMA * (hit[0] - self.est[0]), self.est[1] + EMA * (hit[1] - self.est[1]))

        seen = self.sensor.seen_within(LOST_HOLD_S)
        if not seen:
            self._lost_total += dt
            if self._lost_total > LOST_FAIL_S:
                self._stop_mover()
                self.failed = f'lost the target for over {LOST_FAIL_S:.0f} s'
                return False

        if self.est is not None and seen:
            tx, ty = self.est
            dx, dy = tx - pos[0], ty - pos[1]
            d = math.hypot(dx, dy) or 1e-3
            self.yaw = math.atan2(dy, dx)
            gx, gy = tx - dx / d * FOLLOW_DIST_M, ty - dy / d * FOLLOW_DIST_M
            self.body.send_position_setpoint(gx, gy, FOLLOW_ALT_M, yaw=self.yaw)
            self._dists.append(d)
            truth = sim_truth.mover_position()
            if truth is not None:
                self._errs.append(math.hypot(tx - truth[1], ty - truth[2]))
                self._travel = truth[3]
        elif self.est is not None and not self.sensor.seen_within(LOST_HOLD_S * 3):
            # turn on the spot to re-find it
            self.yaw = wrap(self.yaw + math.radians(SCAN_RATE_DPS) * dt)
            self.body.send_position_setpoint(pos[0], pos[1], FOLLOW_ALT_M, yaw=self.yaw)
        else:
            self.body.send_position_setpoint(pos[0], pos[1], FOLLOW_ALT_M, yaw=self.yaw)

        if now - self._t0 >= FOLLOW_S:
            self._stop_mover()
            return self._grade()
        return False

    def _grade(self):
        in_view = self._in_view / max(1, self._ticks)
        mean_d = sum(self._dists) / len(self._dists) if self._dists else 0.0
        summary = f'in view {in_view:.0%}, mean distance {mean_d:.1f} m'
        problems = []
        if in_view < MIN_IN_VIEW:
            problems.append(f'in view only {in_view:.0%}')
        if self.move_target:
            mean_err = sum(self._errs) / len(self._errs) if self._errs else float('inf')
            summary += f', target moved {self._travel:.1f} m, estimate off by {mean_err:.2f} m on average'
            if self._travel < MIN_TRAVEL_M:
                problems.append(f'target only moved {self._travel:.1f} m (mover not running?)')
            if mean_err > TRUTH_TOL_M:
                problems.append(f'estimate {mean_err:.2f} m from the real target (following the wrong thing?)')
            if not DIST_MIN_M <= mean_d <= DIST_MAX_M:
                problems.append(f'kept {mean_d:.1f} m away (wanted {DIST_MIN_M}-{DIST_MAX_M} m)')
        self.body.get_logger().info(f'follow_target: {summary}.')
        if problems:
            self.failed = '; '.join(problems)
            return False
        return True
