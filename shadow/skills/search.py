#!/usr/bin/env python3
"""
SearchSkill - fly a search pattern while watching for the payload, and
pin down where it is on the ground. Tier 2, Perception. Added Nov 9, 2026.

This is "follow_path + detect" as one skill: FollowPathSkill flies the
zigzag (same 8 points as the delivery search sweep), and a
GroundTargetSensor turns every orange detection into a ground position.
A find counts once FIND_MIN_SIGHTINGS sightings agree to within
FIND_MAX_SPREAD_M, so one glitchy frame can't trigger it.

Stop or finish the path? Default: STOP on the first confirmed find.
For a single-payload delivery, flying the rest of the zigzag after you
have found it only burns battery. --full-path keeps flying (and keeps
the best estimate), for survey-style jobs where you want everything.

Writes the result to ~/shadow_sim_runtime/state/found_target.json for
approach_target (and the delivery_rehearsal sequence) to pick up.

Pass: a confirmed find. Sim exam: the estimate must also be within
TRUTH_TOL_M of a real orange payload (sim_truth.py), otherwise it found
something that isn't there.
Fail: path finished with nothing found, or a find that isn't real.
"""

import json
import time
from pathlib import Path

from .follow_path import FollowPathSkill
from . import sim_truth
from shadow.senses.ground_target import GroundTargetSensor

FIND_MIN_SIGHTINGS = 10
FIND_MAX_SPREAD_M = 0.8
TRUTH_TOL_M = 1.0
FOUND_FILE = Path.home() / 'shadow_sim_runtime' / 'state' / 'found_target.json'


def write_found(x, y, n, spread, source):
    FOUND_FILE.parent.mkdir(parents=True, exist_ok=True)
    FOUND_FILE.write_text(json.dumps({
        'x': round(x, 3), 'y': round(y, 3), 'sightings': n,
        'spread_m': round(spread, 3), 'source': source, 't': time.time()}))


def read_found(max_age_s=900.0):
    try:
        d = json.loads(FOUND_FILE.read_text())
    except (OSError, ValueError):
        return None
    if time.time() - d.get('t', 0) > max_age_s:
        return None
    return d


class SearchSkill(FollowPathSkill):
    name = 'search'
    timeout_sec = 160.0

    def __init__(self, body, path='zigzag', full_path=False):
        super().__init__(body, path=path)
        self.full_path = full_path
        self.sensor = GroundTargetSensor(body, source='orange')
        self.failed = None
        self.found = None

    def start(self):
        self.sensor.clear()
        super().start()
        self.body.get_logger().info(
            f'search: looking for the orange payload ({"full path" if self.full_path else "stop on find"}).')

    def _check_find(self):
        est = self.sensor.estimate(window_s=10.0, radius_m=FIND_MAX_SPREAD_M)
        if est is None or est[2] < FIND_MIN_SIGHTINGS:
            return False
        x, y, n, spread = est
        if self.found is None:
            self.body.get_logger().info(
                f'search: payload confirmed at ({x:.2f}, {y:.2f}) from {n} sightings '
                f'(spread {spread:.2f} m).')
        self.found = est
        return True

    def update(self):
        path_done = super().update()
        found_now = self._check_find()
        if found_now and not self.full_path:
            return self._finish()
        if path_done:
            if self.found is None:
                self.failed = 'path finished, no payload found'
                return False
            return self._finish()
        return False

    def _finish(self):
        x, y, n, spread = self.found
        write_found(x, y, n, spread, 'search')
        name, err = sim_truth.nearest(x, y, sim_truth.ORANGE)
        self.body.get_logger().info(
            f'search: estimate is {err:.2f} m from {name} (sim truth).')
        if err > TRUTH_TOL_M:
            self.failed = f'found something {err:.1f} m from any real payload (false find)'
            return False
        return True
