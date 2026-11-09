#!/usr/bin/env python3
"""
ApproachTargetSkill - close in on a detected payload and settle right
above it. Tier 3, Delivery. Added Nov 9, 2026.

Why it isn't just "go_to the estimate": the front camera looks straight
ahead, so the closer Shadow gets, the further below the camera's view
the payload drops (at 2 m altitude the nearest visible ground is ~3.2 m
ahead). Estimates from far away are rough (a degree of tilt is tens of
centimetres at 10 m). So it works in phases:

  ACQUIRE  - start from search's found_target.json if it's recent;
             otherwise turn on the spot (one full turn) until the orange
             payload is confirmed.
  VIEW     - fly to a viewpoint VIEW_DIST_M short of the estimate, at
             VIEW_ALT_M, nose pointing at it. Close enough for a sharp
             estimate, far enough that it's still in the picture.
  REFINE   - hover there and collect fresh sightings for REFINE_S. The
             refined estimate replaces the rough one. Lost it = fail.
  OVERHEAD - fly to directly above the refined estimate at HOVER_ALT_M
             (blind for the last few metres, on purpose, see above) and
             hold within SETTLE_TOL_M for SETTLE_S.

Pass: settled above the refined estimate. Sim exam: the refined
estimate must be within TRUTH_TOL_M of a real orange payload.
Writes ~/shadow_sim_runtime/state/approached_target.json.

Learned-slot candidate: the controller in VIEW/OVERHEAD is where a
trained policy could do it faster or with a camera that keeps the
payload in view the whole way down.
"""

import json
import math
import time
from pathlib import Path

from .base import Skill
from .heading import wrap
from . import sim_truth
from .search import read_found, write_found
from shadow.senses.ground_target import GroundTargetSensor

VIEW_DIST_M = 4.5
VIEW_ALT_M = 2.0
VIEW_TOL_M = 0.5
REFINE_S = 3.0
REFINE_MIN_SIGHTINGS = 10
HOVER_ALT_M = 1.5
SETTLE_TOL_M = 0.3
SETTLE_S = 3.0
SCAN_RATE_DPS = 30.0
ACQUIRE_MIN_SIGHTINGS = 6
TRUTH_TOL_M = 0.5
APPROACHED_FILE = Path.home() / 'shadow_sim_runtime' / 'state' / 'approached_target.json'


class ApproachTargetSkill(Skill):
    name = 'approach_target'
    timeout_sec = 90.0

    def __init__(self, body):
        super().__init__(body)
        self.sensor = GroundTargetSensor(body, source='orange')
        self.failed = None
        self.phase = None
        self.target = None          # (x, y) current best estimate
        self.view = None            # (x, y, yaw)
        self._t = None
        self._scan_yaw = None
        self._scan_turned = 0.0
        self._settle_since = None

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
        self.body.get_logger().warn('approach_target: needs armed + GUIDED (Arm, Takeoff first).')
        return False

    def _enter(self, phase):
        self.phase = phase
        self._t = time.time()
        self.body.get_logger().info(f'approach_target: {phase}')

    def start(self):
        self.sensor.clear()
        found = read_found()
        if found is not None:
            self.target = (found['x'], found['y'])
            self.body.get_logger().info(
                f'approach_target: starting from search\'s find at ({found["x"]:.2f}, {found["y"]:.2f}).')
            self._plan_view()
            self._enter('VIEW')
        else:
            self._scan_yaw = self.body.current_yaw() or 0.0
            self._enter('ACQUIRE')

    def _plan_view(self):
        pos = self.body.current_position()
        tx, ty = self.target
        dx, dy = tx - pos[0], ty - pos[1]
        d = math.hypot(dx, dy) or 1.0
        ux, uy = dx / d, dy / d
        self.view = (tx - ux * VIEW_DIST_M, ty - uy * VIEW_DIST_M, math.atan2(uy, ux))

    def update(self):
        pos = self.body.current_position()
        if pos is None:
            return False
        now = time.time()

        if self.phase == 'ACQUIRE':
            step = math.radians(SCAN_RATE_DPS) * 0.1
            self._scan_yaw = wrap(self._scan_yaw + step)
            self._scan_turned += step
            self.body.send_position_setpoint(pos[0], pos[1], max(pos[2], VIEW_ALT_M), yaw=self._scan_yaw)
            est = self.sensor.estimate(window_s=3.0)
            if est and est[2] >= ACQUIRE_MIN_SIGHTINGS:
                self.target = (est[0], est[1])
                self._plan_view()
                self._enter('VIEW')
            elif self._scan_turned > 2 * math.pi:
                self.failed = 'no orange payload in sight after a full turn (run search first)'
            return False

        if self.phase == 'VIEW':
            vx, vy, vyaw = self.view
            # Face the TARGET the whole way (not the direction of travel):
            # the viewpoint is usually on the line towards it, but if
            # search found it closer than VIEW_DIST_M, Shadow backs up
            # to the viewpoint while still looking at it.
            tyaw = math.atan2(self.target[1] - pos[1], self.target[0] - pos[0])
            self.body.send_position_setpoint(vx, vy, VIEW_ALT_M, yaw=tyaw)
            if math.hypot(pos[0] - vx, pos[1] - vy) <= VIEW_TOL_M and abs(pos[2] - VIEW_ALT_M) < 0.3:
                self.sensor.clear()
                self._enter('REFINE')
            return False

        if self.phase == 'REFINE':
            vx, vy, vyaw = self.view
            self.body.send_position_setpoint(vx, vy, VIEW_ALT_M, yaw=vyaw)
            if now - self._t < REFINE_S:
                return False
            est = self.sensor.estimate(window_s=REFINE_S + 0.5, radius_m=0.5)
            if est is None or est[2] < REFINE_MIN_SIGHTINGS:
                self.failed = f'lost the payload at the viewpoint ({0 if est is None else est[2]} sightings)'
                return False
            moved = math.hypot(est[0] - self.target[0], est[1] - self.target[1])
            self.target = (est[0], est[1])
            self.body.get_logger().info(
                f'approach_target: refined to ({est[0]:.2f}, {est[1]:.2f}), '
                f'{moved:.2f} m from the first estimate, spread {est[3]:.2f} m.')
            self._enter('OVERHEAD')
            return False

        if self.phase == 'OVERHEAD':
            tx, ty = self.target
            self.body.send_position_setpoint(tx, ty, HOVER_ALT_M, yaw=self.view[2])
            off = math.hypot(pos[0] - tx, pos[1] - ty)
            if off <= SETTLE_TOL_M and abs(pos[2] - HOVER_ALT_M) < 0.3:
                if self._settle_since is None:
                    self._settle_since = now
                if now - self._settle_since >= SETTLE_S:
                    return self._finish(off)
            else:
                self._settle_since = None
            return False
        return False

    def _finish(self, off):
        tx, ty = self.target
        name, err = sim_truth.nearest(tx, ty, sim_truth.ORANGE)
        APPROACHED_FILE.parent.mkdir(parents=True, exist_ok=True)
        APPROACHED_FILE.write_text(json.dumps({'x': tx, 'y': ty, 't': time.time(),
                                               'truth': name, 'truth_err_m': round(err, 3)}))
        write_found(tx, ty, REFINE_MIN_SIGHTINGS, 0.0, 'approach_target')
        self.body.get_logger().info(
            f'approach_target: settled {off:.2f} m from the estimate; estimate is {err:.2f} m '
            f'from {name} (sim truth).')
        if err > TRUTH_TOL_M:
            self.failed = f'settled over the wrong spot: {err:.2f} m from {name}'
            return False
        return True
