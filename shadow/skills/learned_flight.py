#!/usr/bin/env python3
"""
LearnedFlightSkill - Tier 3, the first Skill driven by a TRAINED policy
instead of hand-coded control math.

Bridges the gap Zer0 asked about directly tonight (Oct 25, 2026):
"how could training eventually link to the missions?" Up to this file,
training (train_go_to.py/train_agility.py, saved PPO checkpoints under
shadow_ws/ml/models/) and flying (every Skill in this package) have
been on completely separate tracks - a checkpoint got saved to disk
and nothing ever loaded it back in. This Skill is that missing link:
every tick, instead of computing the next setpoint by hand the way
GoToSkill or DeliverPayloadSkill's APPROACH phase do, it asks a loaded
Stable-Baselines3 PPO model what to do (`model.predict(obs)`) and flies
whatever it says - through the exact same safety net (Skill.limits(),
inherited unchanged from base.py) every hand-coded Skill already flies
under, so a bad policy output gets caught and aborted exactly like a
bad hand-written control law would.

Observation/action contract - this MUST match whatever Gym env the
loaded checkpoint actually trained in, or its outputs are meaningless:
  - obs: (goal - current_position), a 3-vector, float32. Confirmed by
    reading both training envs side by side: go_to_env.py's GoToEnv
    AND agility_circuit_env.py's AgilityCircuitEnv use this exact
    shape - which is exactly why ONE Skill here can fly either
    checkpoint without caring which task trained it.
  - action: a 3-vector position delta, clipped to
    +/-MAX_STEP_DELTA_M - the same 0.3m-per-tick-at-10Hz scale both
    envs trained with.

Default wiring (Zer0's own ask, same night: "I want to train my drone
and then showcase this to my dad later on today"): flies the go_to
checkpoint (models/go_to/go_to_ppo_final.zip) - the one with real
training hours already behind it (several runs, crash-free rate
checked against real numbers in areas/drone-attributes-system.md) -
around the SAME six real points agility_circuit_env.py race-trains
against (DEFAULT_WAYPOINTS_XY below, duplicated rather than imported,
same reasoning agility_circuit_env.py's own duplication of
deliver_payload.py's coordinates already documents). The go_to policy
was never trained ON a sequential circuit - it only ever learned "fly
toward wherever THIS tick's one single goal is" - but since this Skill
simply hands it a fresh single goal each time the current one is
reached, that's exactly the primitive go_to actually learned, chained
manually here rather than inside the env. Pass checkpoint_path=/
waypoints= explicitly to point this at the agility-circuit checkpoint
instead, once that one has real training hours of its own (it has
none yet as of tonight - built, never trained).

HONESTY, not hedging, about what to expect tonight: this policy is
genuinely early-stage (see areas/drone-attributes-system.md's own real
crash-free numbers - roughly 30-65% depending on which run, nowhere
near polished). This Skill does not pretend otherwise, and the safety
clamp below still aborts the whole run the instant the policy's output
would fly Shadow somewhere unsafe. A rough, imperfect flight is the
honest, expected result of a demo tonight - not a bug in this bridge.
"""

import os

import numpy as np

from .base import Skill

MAX_STEP_DELTA_M = 0.3          # same per-tick action scale both training envs used
ARRIVAL_TOLERANCE_M = 0.5       # same as agility_circuit_env.py's own tolerance
FLIGHT_ALTITUDE_M = 1.5         # matches agility_circuit_env.py's CIRCUIT_ALTITUDE_M
TAKEOFF_ALT_TOLERANCE_M = 0.3
MAX_TICKS_PER_WAYPOINT = 250    # ~25s at 10Hz - an early policy giving up on one leg
                                # shouldn't hang the whole run; skip to the next
                                # waypoint instead, same "don't trust one frozen
                                # state forever" lesson Run 7/8 already taught.

DEFAULT_CHECKPOINT = os.path.expanduser(
    '~/shadow_ws/ml/models/go_to/go_to_ppo_final.zip')

# The same six real points agility_circuit_env.py's CIRCUIT_WAYPOINTS_XY
# race-trains against (five payload anchors + the delivery pad, from
# shadow_runway.sdf) - duplicated here on purpose rather than imported,
# so this file has no dependency on the training stack at import time
# beyond stable_baselines3 itself.
DEFAULT_WAYPOINTS_XY = [
    (6.0, 2.0),
    (1.0, 9.0),
    (-5.0, 7.0),
    (-6.0, -3.0),
    (0.0, -4.0),
    (3.0, -4.0),
]


class LearnedFlightSkill(Skill):
    """Tier 3 - the first Skill whose flight decisions come from a
    trained policy's forward pass instead of hand-written control
    math. requirements()/update()/limits() per the usual Skill
    contract; limits() is inherited unchanged from base.py, same
    geofence/altitude safety net every other Skill flies under."""

    name = 'learned_flight'
    timeout_sec = 240.0

    def __init__(self, body, checkpoint_path=None, waypoints=None,
                 altitude_m=FLIGHT_ALTITUDE_M, laps=1):
        super().__init__(body)
        self.checkpoint_path = checkpoint_path or DEFAULT_CHECKPOINT
        self.waypoints = list(waypoints) if waypoints else list(DEFAULT_WAYPOINTS_XY)
        self.altitude_m = altitude_m
        self.laps_target = laps
        self._model = None
        self._phase = 'TAKEOFF'
        self._takeoff_commanded = False
        self._wp_idx = 0
        self._ticks_on_current_wp = 0
        self._laps_completed = 0
        self._land_commanded = False
        self._tick_count = 0

    # ---- Skill contract ------------------------------------------------

    def requirements(self):
        if not os.path.exists(self.checkpoint_path):
            self.body.get_logger().error(
                f'learned_flight: no checkpoint at {self.checkpoint_path} - '
                f'train one first (Start Training / Start Agility Training).')
            return False
        try:
            from stable_baselines3 import PPO
        except ImportError:
            self.body.get_logger().error(
                'learned_flight: stable_baselines3 not installed - '
                'pip install gymnasium stable-baselines3 mlflow')
            return False
        try:
            self._model = PPO.load(self.checkpoint_path)
        except Exception as exc:  # noqa: BLE001 - a bad/corrupt checkpoint should
            # surface clearly here rather than disappear behind a narrow except
            # list that doesn't happen to match whatever SB3/torch raises.
            self.body.get_logger().error(
                f'learned_flight: failed to load checkpoint {self.checkpoint_path}: {exc}')
            return False

        if not self.body.wait_until_connected(timeout_sec=10.0):
            self.body.get_logger().error('learned_flight: no MAVROS connection.')
            return False
        # 25.0, not 10.0 - same reasoning as every other Skill's matching
        # comment (takeoff.py, go_to.py, deliver_payload.py): a live run
        # on shadow_runway saw this still timing out well after the
        # FCU's own log reported GPS in use.
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            self.body.get_logger().error('learned_flight: no position estimate yet.')
            return False
        return True

    def start(self):
        if not self.body.is_armed():
            self.body.set_mode('GUIDED')
            self.body.arm()
        self._takeoff_commanded = self.body.takeoff(self.altitude_m)

    def update(self):
        self._tick_count += 1
        if self._phase == 'TAKEOFF':
            return self._tick_takeoff()
        if self._phase == 'FLY':
            return self._tick_fly()
        if self._phase == 'LAND':
            return self._tick_land()
        self.body.get_logger().error(f'learned_flight: unknown phase {self._phase!r}')
        return False

    # ---- phases ---------------------------------------------------------

    def _tick_takeoff(self):
        if not self._takeoff_commanded:
            return False
        pos = self.body.current_position()
        if pos is None:
            return False
        if abs(pos[2] - self.altitude_m) <= TAKEOFF_ALT_TOLERANCE_M:
            self.body.get_logger().info(
                f'learned_flight: at altitude - handing control to '
                f'{os.path.basename(self.checkpoint_path)}.')
            self._phase = 'FLY'
        return False

    def _current_goal(self):
        x, y = self.waypoints[self._wp_idx % len(self.waypoints)]
        return (x, y, self.altitude_m)

    def _tick_fly(self):
        pos = self.body.current_position()
        if pos is None:
            return False

        goal = self._current_goal()
        obs = np.array(goal, dtype=np.float32) - np.array(pos, dtype=np.float32)

        # deterministic=True - exploration noise is a TRAINING tool (it
        # helps a policy discover better behaviour while learning), never
        # something a real flight in front of an audience should fly
        # with switched on.
        action, _state = self._model.predict(obs, deterministic=True)
        delta = np.clip(action, -MAX_STEP_DELTA_M, MAX_STEP_DELTA_M)
        target = (
            pos[0] + float(delta[0]),
            pos[1] + float(delta[1]),
            pos[2] + float(delta[2]),
        )

        if self._tick_count % 10 == 0:
            self.body.get_logger().info(
                f'learned_flight: waypoint {self._wp_idx % len(self.waypoints)} '
                f'goal=({goal[0]:.1f},{goal[1]:.1f}) '
                f'pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) '
                f'action=({delta[0]:.2f},{delta[1]:.2f},{delta[2]:.2f})')

        self.body.send_position_setpoint(*target)

        distance = float(np.linalg.norm(np.array(goal) - np.array(pos)))
        self._ticks_on_current_wp += 1
        if distance <= ARRIVAL_TOLERANCE_M:
            self.body.get_logger().info(
                f'learned_flight: reached waypoint {self._wp_idx % len(self.waypoints)}.')
            self._advance_waypoint()
        elif self._ticks_on_current_wp > MAX_TICKS_PER_WAYPOINT:
            self.body.get_logger().warning(
                f'learned_flight: stuck on waypoint {self._wp_idx % len(self.waypoints)} '
                f'for {MAX_TICKS_PER_WAYPOINT} ticks - the policy isn\'t '
                f'converging on this leg, skipping to the next one rather than '
                f'hanging for the rest of the run.')
            self._advance_waypoint()
        return False

    def _advance_waypoint(self):
        self._wp_idx += 1
        self._ticks_on_current_wp = 0
        if self._wp_idx % len(self.waypoints) == 0:
            self._laps_completed += 1
            self.body.get_logger().info(
                f'learned_flight: lap {self._laps_completed}/{self.laps_target} complete.')
            if self._laps_completed >= self.laps_target:
                self._phase = 'LAND'

    def _tick_land(self):
        if not self._land_commanded:
            self.body.set_mode('LAND')
            self._land_commanded = True
        return not self.body.is_armed()
