#!/usr/bin/env python3
"""
SequenceSkill + DeliveryRehearsalSkill - a mission is a string of skills.
Added Nov 9, 2026.

SequenceSkill runs other skills one after another inside one run_skill.py
run, with each stage keeping its own contract: its requirements() is
checked when the stage starts, its limits() every tick, its own timeout,
and its own failure reason. If any stage fails, the whole sequence fails
with "<stage>: <reason>", so the log says exactly where it broke.

DeliveryRehearsalSkill is the delivery mission minus the two parts that
need hardware Shadow doesn't have yet (pick up, drop off):

  arm -> takeoff -> search (zigzag, stop on find) -> approach_target
      -> hover over it -> go_to the pad -> hover over the pad -> rtl

When a gripper exists, "hover over it" becomes pickup and "hover over
the pad" becomes release, and nothing else changes. That's the point of
building missions out of skills.
"""

import time

from .base import Skill
from .arm import ArmSkill
from .takeoff import TakeoffSkill
from .hover import HoverSkill
from .go_to import GoToSkill
from .rtl import RtlSkill
from .search import SearchSkill
from .approach_target import ApproachTargetSkill
from . import sim_truth

PAD_ALT_M = 2.0


class SequenceSkill(Skill):
    name = 'sequence'
    stages = []   # [(label, factory(body) -> Skill)]

    def __init__(self, body):
        super().__init__(body)
        self.failed = None
        self.i = -1
        self.cur = None
        self._t = None
        self.timeout_sec = 600.0

    def requirements(self):
        return True   # each stage checks its own when it starts

    def limits(self):
        return True   # each stage's own limits() runs in update()

    def _enter(self, i):
        self.i = i
        label, factory = self.stages[i]
        self.cur = factory(self.body)
        self.body.get_logger().info(f'{self.name}: stage {i + 1}/{len(self.stages)} {label}')
        if not self.cur.requirements():
            self.failed = f'{label}: requirements() failed'
            return
        self.cur.start()
        self._t = time.time()

    def start(self):
        self._enter(0)

    def update(self):
        if self.failed:
            return False
        label = self.stages[self.i][0]
        if not self.cur.limits():
            self.failed = f'{label}: limits() failed - aborted for safety'
            return False
        if self.cur.update():
            self.body.get_logger().info(f'{self.name}: {label} done ({time.time() - self._t:.1f} s)')
            if self.i + 1 >= len(self.stages):
                return True
            self._enter(self.i + 1)
            return False
        if getattr(self.cur, 'failed', None):
            self.failed = f'{label}: {self.cur.failed}'
        elif time.time() - self._t > self.cur.timeout_sec:
            self.failed = f'{label}: timed out after {self.cur.timeout_sec:.0f}s'
        return False


class DeliveryRehearsalSkill(SequenceSkill):
    name = 'delivery_rehearsal'
    stages = [
        ('arm', lambda b: ArmSkill(b)),
        ('takeoff', lambda b: TakeoffSkill(b, altitude_m=2.0)),
        ('search', lambda b: SearchSkill(b, path='zigzag')),
        ('approach_target', lambda b: ApproachTargetSkill(b)),
        ('hover over payload (pickup stand-in)', lambda b: HoverSkill(b, duration_sec=3.0)),
        ('go_to pad', lambda b: GoToSkill(b, sim_truth.PAD[0], sim_truth.PAD[1], PAD_ALT_M)),
        ('hover over pad (drop-off stand-in)', lambda b: HoverSkill(b, duration_sec=3.0)),
        ('rtl', lambda b: RtlSkill(b)),
    ]
