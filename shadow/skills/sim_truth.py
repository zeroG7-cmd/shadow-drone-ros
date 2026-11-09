#!/usr/bin/env python3
"""
Simulation ground truth for the vision skills' exams. Nov 9, 2026.

The skills themselves never use these coordinates to fly (that would be
cheating: the whole point is finding the payload with the camera). They
are only used AFTER the fact, to check whether what the camera estimated
matches where the payload really is in shadow_runway.sdf. On the real
drone there is no truth file, so these checks simply don't run.

Positions are go_to.py's WAYPOINTS (same source of truth), and follow
_target's moving payload is read from move_target.py's state file.
"""

import json
import math
import time
from pathlib import Path

from .go_to import WAYPOINTS

PAYLOADS = {name: (x, y) for name, x, y in WAYPOINTS}
ORANGE = {n: p for n, p in PAYLOADS.items() if 'orange' in n}
PAD = (0.0, -4.0)   # delivery_pad, see shadow/limits.py docstring
MOVER_STATE = Path.home() / 'shadow_sim_runtime' / 'state' / 'target_mover.json'


def nearest(x, y, candidates=None):
    """(name, distance_m) of the nearest known payload."""
    cands = PAYLOADS if candidates is None else candidates
    return min(((n, math.hypot(x - px, y - py)) for n, (px, py) in cands.items()),
               key=lambda item: item[1])


def mover_position(max_age_s=1.0):
    """(name, x, y, travelled_m) of the payload move_target.py is driving,
    or None when the mover isn't running."""
    try:
        d = json.loads(MOVER_STATE.read_text())
    except (OSError, ValueError):
        return None
    if time.time() - d.get('t', 0) > max_age_s:
        return None
    return d['name'], d['x'], d['y'], d.get('travelled', 0.0)
