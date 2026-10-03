#!/usr/bin/env python3
"""
Skill - the base contract every trainable behaviour follows.

Four methods, checked in this order by run_skill.py:

  requirements() - checked ONCE, before start(). "Can this even be
                    attempted right now" (connected to ArduPilot, has a
                    position estimate, etc). Return False and the run
                    fails immediately, before anything happens.
  limits()       - checked every tick, for as long as the skill runs.
                    "Is Shadow still inside what's safe" (altitude,
                    geofence). Return False at ANY point and the run is
                    aborted - this is the actual safety check, not
                    requirements().
  start()        - called once, after requirements() passes. Issues the
                    one-time command (set_mode, arm, takeoff...).
  update()       - called every tick after start(). Returns True only on
                    genuine completion - reaching the target, not "the
                    mode-change message was accepted". That mode_sent
                    vs. actually-changed-mode confusion is the exact bug
                    documented as fixed from the old mission_planner.py,
                    and it's worth stating plainly: a Skill that returns
                    True from update() because a service call succeeded,
                    instead of because the drone is actually where it's
                    supposed to be, reintroduces that same bug under a
                    new name.

A timeout is handled by run_skill.py, not by individual skills - every
skill gets the same timeout behaviour (fail, don't hang forever) without
having to implement it five times.
"""


class Skill:
    name = 'base_skill'
    timeout_sec = 30.0

    def __init__(self, body):
        self.body = body

    def requirements(self):
        """Checked once before start(). Override and return False (with
        a logged reason) if this skill can't even be attempted."""
        return True

    def limits(self):
        """Checked every tick while the skill runs. Return False to
        abort immediately - this is the safety check, not requirements().
        The base implementation just defers to shadow/limits.py's shared
        geofence/altitude check, since every skill that moves the drone
        needs the same one."""
        from shadow.limits import within_limits
        pos = self.body.current_position()
        if pos is None:
            return True  # no position yet isn't a limits violation
        return within_limits(pos)

    def start(self):
        """Issue the one-time command. Called once, after requirements()
        passes."""
        raise NotImplementedError

    def update(self):
        """Called every tick after start(). Return True only on genuine
        completion, never on a command merely being accepted."""
        raise NotImplementedError
