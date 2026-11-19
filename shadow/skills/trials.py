#!/usr/bin/env python3
"""
Trials - short, fixed, repeatable flights that measure Shadow, for the
attribute engine. Added Nov 9, 2026. No machine learning: fly a set test,
record what happens, do arithmetic.

A trial is a Skill with a measuring tape. Same contract (requirements,
limits, start, update) so run_skill.py, the Command Dock, the run strip
and SequenceSkill all work with it unchanged. The extra part: while it
flies it records samples, and when it's done it turns them into a few
numbers (measure), scores them against config/attributes.json and saves
one row with the tuning snapshot (shadow/skills/trial_record.py).

A trial PASSES when it completes a valid measurement. Bad numbers are not
a failure, they're the result: a wobbly hover still passes the trial and
earns a low STABILITY score. It only FAILS if it can't measure (not
flying, lost position, limits()).

Trials (all start from a hover, so Arm + Takeoff first, or use the suite):
  trial_hover_hold     5 s settle + 30 s hold -> DRIFT, HOVER THROTTLE,
                       HOVER AMPS, FLIGHT TIME (from BATT_CAPACITY)
  trial_step_response  sudden 1 m step -> RESPONSE, OVERSHOOT, SETTLE
  trial_turn           commanded 180 deg yaw -> TURN RATE
  trial_climb          climb 2.5 m and come back -> CLIMB RATE
  trial_speed          straight 12 m run -> TOP SPEED
  trial_goto           fly 4 m to a point and hold -> GO TO ERROR
  trial_lap            fly the 6-point circuit -> LAP TIME, PATH ERROR
  trial_detection      hover 5 m from a payload, watch the tracker ->
                       DETECTION, FALSE ALARMS, JITTER
  trial_disturbance    Gazebo shoves Shadow sideways -> RECOVERY
  trial_landing        fly 4 m out, RTL -> LANDING (ends the flight)
  trial_suite          arm, takeoff, then all of the above in a row

Nov 12: only real measurements are scored. If a trial can't measure
something (no current from the sim, ArduPilot param unreadable, the
shove didn't land), that value stays blank instead of being estimated.

Honest limits:
  - Everything here flies in GUIDED, so ArduPilot's own speed and
    acceleration limits (WP_SPD, WP_ACC, climb speed, yaw slew) shape
    RESPONSE, TURN RATE and CLIMB RATE. They measure "Shadow as tuned",
    which is what a tune change should move. A true full-throttle climb
    needs a mode the companion computer doesn't fly yet.
  - Samples come from MAVROS at the 10 Hz tick. Fine for drift and
    steps; fine vibration detail would need the ArduPilot .BIN logs.
"""

import math
import time

from .base import Skill
from .heading import wrap
from .trial_record import read_params, save_trial
from .follow_path import FollowPathSkill, PATHS

SETTLE_S = 5.0
MIN_AIRBORNE_M = 1.0


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else None


class Trial(Skill):
    name = 'trial'
    timeout_sec = 60.0

    def __init__(self, body):
        super().__init__(body)
        self.failed = None
        self.phase = None
        self.samples = []
        self.metrics = None
        self._t0 = None

    def requirements(self):
        if not self.body.wait_until_connected(timeout_sec=10.0):
            return False
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            return False
        import rclpy
        deadline = time.time() + 6.0   # Nov 12: 3 -> 6 s, a trial can start mid-wobble
        pos = None
        while rclpy.ok():
            pos = self.body.current_position()
            if (self.body.is_armed() and self.body.current_mode() == 'GUIDED'
                    and pos is not None and pos[2] >= MIN_AIRBORNE_M):
                return True
            if time.time() > deadline:
                break
            rclpy.spin_once(self.body, timeout_sec=0.2)
        z = None if pos is None else round(pos[2], 2)
        self.body.get_logger().warn(
            f'{self.name}: requirements failed - armed={self.body.is_armed()}, '
            f'mode={self.body.current_mode()!r}, altitude={z} m (needs armed, GUIDED, above '
            f'{MIN_AIRBORNE_M} m; Arm + Takeoff first, or run trial_suite).')
        return False

    def elapsed(self):
        return time.time() - self._t0

    def start(self):
        self._t0 = time.time()
        self.hold = self.body.current_position()
        self.hold_yaw = self.body.current_yaw()
        self.body.get_logger().info(f'{self.name}: started at {tuple(round(v, 2) for v in self.hold)}.')

    def finish(self, notes=None):
        """Measure, score, save. Returns True (the trial completed)."""
        self.metrics = self.measure()
        params = read_params(self.body)
        self.after_params(params)
        scored = save_trial(self.name, self.metrics, params, notes)
        summary = ', '.join(f'{v["label"]} {v["value"]} -> {v["score"]}' for v in scored.values())
        self.body.get_logger().info(f'{self.name}: {summary or "no scored metrics"}')
        self.phase = 'SAVED'
        return True

    def after_params(self, params):
        """Hook for metrics that need a parameter (flight time needs BATT_CAPACITY)."""

    def measure(self):
        raise NotImplementedError


class HoverHoldTrial(Trial):
    name = 'trial_hover_hold'
    timeout_sec = 50.0
    HOLD_S = 30.0

    def __init__(self, body):
        super().__init__(body)
        from mavros_msgs.msg import RCOut
        from sensor_msgs.msg import BatteryState
        from rclpy.qos import qos_profile_sensor_data
        self._rc = None
        self._batt = None
        body.create_subscription(RCOut, '/mavros/rc/out', self._rc_cb, qos_profile_sensor_data)
        body.create_subscription(BatteryState, '/mavros/battery', self._batt_cb, qos_profile_sensor_data)

    def _rc_cb(self, msg):
        self._rc = list(msg.channels)

    def _batt_cb(self, msg):
        self._batt = msg

    def update(self):
        x, y, z = self.hold
        self.body.send_position_setpoint(x, y, z, yaw=self.hold_yaw)
        t = self.elapsed()
        if t < SETTLE_S:
            self.phase = 'SETTLE'
            return False
        self.phase = 'HOLD'
        pos = self.body.current_position()
        if pos is not None:
            thr = None
            if self._rc and len(self._rc) >= 4:
                pwm = [c for c in self._rc[:4] if 900 <= c <= 2100]
                if pwm:
                    thr = (sum(pwm) / len(pwm) - 1000) / 10.0
            amps = abs(self._batt.current) if (self._batt is not None and self._batt.current == self._batt.current) else None
            if amps is not None and amps < 0.05:
                amps = None   # the Gazebo-backed sim reports 0 A: estimate from throttle instead
            self.samples.append((t, pos[0], pos[1], pos[2], thr, amps))
        if t >= SETTLE_S + self.HOLD_S:
            return self.finish()
        return False

    def measure(self):
        x0, y0, z0 = self.hold
        if not self.samples:
            return {}
        horiz = [math.hypot(s[1] - x0, s[2] - y0) for s in self.samples]
        alt = [s[3] - z0 for s in self.samples]
        thr = [s[4] for s in self.samples if s[4] is not None]
        amps = [s[5] for s in self.samples if s[5]]
        return {
            'drift_m': round(max(horiz), 3),
            'drift_rms_m': round(rms(horiz), 3),
            'alt_rms_m': round(rms(alt), 3),
            'hover_throttle_pct': round(sum(thr) / len(thr), 1) if thr else None,
            'hover_amps': round(sum(amps) / len(amps), 2) if amps else None,
        }

    def after_params(self, params):
        from .trial_record import load_config
        cfg = load_config()
        # THRUST / WEIGHT from the hover throttle MEASURED in this trial,
        # through ArduPilot's own motor thrust curve (MOT_SPIN_MIN/MAX,
        # MOT_THST_EXPO). At hover, thrust = weight, so
        # T/W = 1 / (thrust fraction at hover). Nov 12: no longer uses the
        # learned MOT_THST_HOVER, which wasn't relearning in SITL (stale).
        h = self.metrics.get('hover_throttle_pct')
        e, lo, hi = params.get('MOT_THST_EXPO'), params.get('MOT_SPIN_MIN'), params.get('MOT_SPIN_MAX')
        tw = None
        if h and e is not None and lo is not None and hi and hi > lo:
            u = (h / 100.0 - lo) / (hi - lo)          # 0..1 across the usable spin range
            frac = (1 - e) * u + e * u * u            # ArduPilot's thrust curve
            tw = round(1.0 / frac, 2) if 0.05 < frac <= 1.0 else None
        self.metrics['thrust_weight'] = tw
        # Current and flight time only from a REAL current reading. The
        # Gazebo-backed sim reports 0 A, so these stay blank there.
        amps = self.metrics.get('hover_amps')
        cap = params.get('BATT_CAPACITY') or (cfg.get('loadout') or {}).get('battery_mah')
        # 80% of capacity is usable without hurting a LiPo
        self.metrics['flight_time_min'] = (
            round(cap / 1000.0 / amps * 60 * 0.8, 1) if cap and amps else None)


class StepResponseTrial(Trial):
    name = 'trial_step_response'
    timeout_sec = 30.0
    STEP_M = 1.0
    WINDOW_S = 10.0
    BAND_M = 0.05

    def start(self):
        super().start()
        # step along the nose direction, or backwards if that would leave the fence
        yaw = self.hold_yaw or 0.0
        self.u = (math.cos(yaw), math.sin(yaw))
        from shadow.limits import within_limits
        tx, ty = self.hold[0] + self.u[0] * self.STEP_M, self.hold[1] + self.u[1] * self.STEP_M
        if not within_limits((tx, ty, self.hold[2])):
            self.u = (-self.u[0], -self.u[1])
        self.target = (self.hold[0] + self.u[0] * self.STEP_M, self.hold[1] + self.u[1] * self.STEP_M, self.hold[2])
        self._step_t = None

    def update(self):
        t = self.elapsed()
        if t < SETTLE_S:
            self.phase = 'SETTLE'
            x, y, z = self.hold
            self.body.send_position_setpoint(x, y, z, yaw=self.hold_yaw)
            return False
        if self._step_t is None:
            self._step_t = t
        self.phase = 'STEP'
        x, y, z = self.target
        self.body.send_position_setpoint(x, y, z, yaw=self.hold_yaw)
        pos = self.body.current_position()
        ts = t - self._step_t
        if pos is not None:
            along = (pos[0] - self.hold[0]) * self.u[0] + (pos[1] - self.hold[1]) * self.u[1]
            self.samples.append((ts, along))
        if ts >= self.WINDOW_S:
            return self.finish()
        return False

    def measure(self):
        if not self.samples:
            return {}
        step = self.STEP_M
        response = next((ts for ts, a in self.samples if a >= 0.9 * step), None)
        peak = max(a for _, a in self.samples)
        settle = None
        for i, (ts, _) in enumerate(self.samples):
            if all(abs(a - step) <= self.BAND_M for _, a in self.samples[i:]):
                settle = ts
                break
        return {
            'response_s': round(response, 2) if response is not None else self.WINDOW_S,
            'overshoot_pct': round(max(0.0, peak - step) / step * 100, 1),
            'settle_s': round(settle, 2) if settle is not None else self.WINDOW_S,
            'settled': settle is not None,
        }


class TurnTrial(Trial):
    name = 'trial_turn'
    timeout_sec = 30.0
    WINDOW_S = 10.0

    def start(self):
        super().start()
        self.target_yaw = wrap((self.hold_yaw or 0.0) + math.pi)
        self._turn_t = None

    def update(self):
        t = self.elapsed()
        x, y, z = self.hold
        if t < 3.0:
            self.phase = 'SETTLE'
            self.body.send_position_setpoint(x, y, z, yaw=self.hold_yaw)
            return False
        if self._turn_t is None:
            self._turn_t = t
        self.phase = 'TURN'
        self.body.send_position_setpoint(x, y, z, yaw=self.target_yaw)
        yaw = self.body.current_yaw()
        ts = t - self._turn_t
        if yaw is not None:
            self.samples.append((ts, yaw))
        if ts >= self.WINDOW_S:
            return self.finish()
        return False

    def measure(self):
        if len(self.samples) < 4:
            return {}
        rates = []
        for i in range(3, len(self.samples)):
            t0, y0 = self.samples[i - 3]
            t1, y1 = self.samples[i]
            if t1 > t0:
                rates.append(abs(math.degrees(wrap(y1 - y0))) / (t1 - t0))
        done = next((ts for ts, y in self.samples if abs(math.degrees(wrap(y - self.target_yaw))) <= 5), None)
        return {
            'turn_rate_dps': round(max(rates), 1) if rates else None,
            'turn_180_s': round(done, 2) if done is not None else None,
        }


class ClimbTrial(Trial):
    name = 'trial_climb'
    timeout_sec = 45.0
    CLIMB_M = 2.5
    WINDOW_S = 12.0

    def start(self):
        super().start()
        from shadow.limits import SIM_LIMITS
        top = min(self.hold[2] + self.CLIMB_M, SIM_LIMITS['max_altitude_m'] - 0.8)
        self.top = (self.hold[0], self.hold[1], top)
        self._climb_t = None
        self._back = False

    def update(self):
        t = self.elapsed()
        pos = self.body.current_position()
        if t < 3.0:
            self.phase = 'SETTLE'
            self.body.send_position_setpoint(*self.hold, yaw=self.hold_yaw)
            return False
        if self._climb_t is None:
            self._climb_t = t
        ts = t - self._climb_t
        if not self._back:
            self.phase = 'CLIMB'
            self.body.send_position_setpoint(*self.top, yaw=self.hold_yaw)
            if pos is not None:
                self.samples.append((ts, pos[2]))
            if ts >= self.WINDOW_S or (pos is not None and abs(pos[2] - self.top[2]) < 0.15):
                self.metrics_pre = self.measure()
                self._back = True
            return False
        self.phase = 'RETURN'
        self.body.send_position_setpoint(*self.hold, yaw=self.hold_yaw)
        if pos is not None and abs(pos[2] - self.hold[2]) < 0.2:
            return self.finish()
        return False

    def measure(self):
        if getattr(self, 'metrics_pre', None) is not None:
            return self.metrics_pre
        rates = []
        for i in range(len(self.samples)):
            t0, z0 = self.samples[i]
            j = next((k for k in range(i + 1, len(self.samples)) if self.samples[k][0] - t0 >= 0.5), None)
            if j is not None:
                t1, z1 = self.samples[j]
                rates.append((z1 - z0) / (t1 - t0))
        return {'climb_rate_ms': round(max(rates), 2) if rates else None}


def make_trial_suite(body):
    from .delivery_rehearsal import SequenceSkill

    class TrialSuite(SequenceSkill):
        name = 'trial_suite'
        stages = _suite_stages()

    return TrialSuite(body)


# ---------------------------------------------------------------------
# Nov 12, 2026 - the rest of the stats.
# ---------------------------------------------------------------------

def _fly_phase(trial, target, yaw=None, tol=0.4, max_s=25.0):
    """Send target; True once within tol (3D)."""
    trial.body.send_position_setpoint(*target, yaw=yaw)
    pos = trial.body.current_position()
    return pos is not None and math.dist(pos, target) <= tol


def _inside(p):
    from shadow.limits import within_limits
    return within_limits(p)


class SpeedTrial(Trial):
    """Straight 12 m run along x at 2.5 m (y kept where Shadow is, clamped
    inside the zigzag area). Peak horizontal speed from position over 0.5 s
    windows. Capped by WP_SPD: measures Shadow as tuned."""
    name = 'trial_speed'
    timeout_sec = 60.0
    RUN_M = 12.0

    def start(self):
        super().start()
        y = max(-2.0, min(8.0, self.hold[1]))
        self.a = (-self.RUN_M / 2, y, 2.5)
        self.b = (self.RUN_M / 2, y, 2.5)
        self.state = 'TO START'
        self._hold_t = None

    def update(self):
        pos = self.body.current_position()
        yaw_ab = 0.0  # facing +x, along the run
        if self.state == 'TO START':
            self.phase = 'POSITION'
            if _fly_phase(self, self.a, yaw=yaw_ab):
                self._hold_t = self._hold_t or time.time()
                if time.time() - self._hold_t > 2.0:
                    self.state = 'RUN'
                    self._run_t = time.time()
            return False
        self.phase = 'RUN'
        self.body.send_position_setpoint(*self.b, yaw=yaw_ab)
        if pos is not None:
            self.samples.append((time.time() - self._run_t, pos[0], pos[1]))
        if (pos is not None and math.dist(pos, self.b) < 0.4) or time.time() - self._run_t > 20:
            return self.finish()
        return False

    def measure(self):
        sp = []
        for i in range(len(self.samples)):
            t0, x0, y0 = self.samples[i]
            j = next((k for k in range(i + 1, len(self.samples)) if self.samples[k][0] - t0 >= 0.5), None)
            if j is not None:
                t1, x1, y1 = self.samples[j]
                sp.append(math.hypot(x1 - x0, y1 - y0) / (t1 - t0))
        return {'top_speed_ms': round(max(sp), 2) if sp else None,
                'run_time_s': round(self.samples[-1][0], 2) if self.samples else None}


class GoToTrial(Trial):
    """Fly 4 m to a fixed point and hold. GO TO ERROR = mean distance from
    the commanded point over 2 s, once Shadow has stopped there."""
    name = 'trial_goto'
    timeout_sec = 45.0

    def start(self):
        super().start()
        tx = self.hold[0] + 4.0 if self.hold[0] < 4.0 else self.hold[0] - 4.0
        self.target = (tx, self.hold[1], 2.5)
        self._arrive_t = None
        self._err = []

    def update(self):
        self.body.send_position_setpoint(*self.target, yaw=self.hold_yaw)
        pos = self.body.current_position()
        if pos is None:
            return False
        d = math.dist(pos, self.target)
        if self._arrive_t is None:
            self.phase = 'FLY'
            if d < 0.5:
                self._arrive_t = time.time()
            return False
        t = time.time() - self._arrive_t
        if t < 3.0:
            self.phase = 'SETTLE'
            return False
        self.phase = 'MEASURE'
        self._err.append(d)
        if t >= 5.0:
            return self.finish()
        return False

    def measure(self):
        return {'goto_error_m': round(sum(self._err) / len(self._err), 3) if self._err else None}


class LapTrial(FollowPathSkill):
    """One lap of the 6-point circuit with follow_path. LAP TIME runs from
    passing the first waypoint to arriving at the last; PATH ERROR is the
    mean distance from the straight lines between waypoints."""
    name = 'trial_lap'
    timeout_sec = 150.0

    def __init__(self, body):
        super().__init__(body, path='circuit')
        self.failed = None
        self.phase = None
        self.samples = []
        self.metrics = None
        self._lap_t0 = None
        self._t_end = None

    def update(self):
        done = super().update()
        self.phase = f'WP {self.i + 1}/{len(self.waypoints)}'
        pos = self.body.current_position()
        if self.i >= 1 and self._lap_t0 is None:
            self._lap_t0 = time.time()
        if self._lap_t0 is not None and pos is not None:
            self.samples.append(pos[:2])
        if done:
            self._t_end = time.time()
            return Trial.finish(self)
        return False

    def measure(self):
        if self._lap_t0 is None or not self.samples:
            return {}
        segs = [(PATHS['circuit'][k], PATHS['circuit'][k + 1]) for k in range(len(PATHS['circuit']) - 1)]

        def seg_dist(p, a, b):
            ax, ay = a
            bx, by = b
            dx, dy = bx - ax, by - ay
            L = dx * dx + dy * dy
            u = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
            return math.hypot(p[0] - (ax + u * dx), p[1] - (ay + u * dy))
        errs = [min(seg_dist(p, a, b) for a, b in segs) for p in self.samples]
        return {'lap_time_s': round(self._t_end - self._lap_t0, 1),
                'path_error_m': round(sum(errs) / len(errs), 3)}

    def after_params(self, params):
        pass


class DetectionTrial(Trial):
    """Hover 5 m from payload_orange_1, facing it, and watch the colour
    tracker (/shadow/detection/track) for 10 s. Each tick: is the tracker
    locked, and does its box project onto the ground within 1 m of ANY real
    payload (sim truth)? The viewpoint uses the payload's known position;
    the measurement doesn't."""
    name = 'trial_detection'
    timeout_sec = 60.0
    WATCH_S = 10.0
    MATCH_M = 1.0

    def __init__(self, body):
        super().__init__(body)
        from std_msgs.msg import Float32MultiArray
        self._track = None
        body.create_subscription(Float32MultiArray, '/shadow/detection/track', self._cb, 10)

    def _cb(self, msg):
        self._track = (time.time(), list(msg.data))

    def start(self):
        super().start()
        from . import sim_truth
        px, py = sim_truth.PAYLOADS['payload_orange_1']
        self.view = (px - 5.0, py, 2.0)
        self.view_yaw = 0.0   # facing +x, at the payload
        self._watch_t = None
        self.ticks = self.hits = self.false = 0
        self.centres = []

    def update(self):
        from . import sim_truth
        from shadow.senses.ground_target import project_to_ground
        if self._watch_t is None:
            self.phase = 'POSITION'
            if _fly_phase(self, self.view, yaw=self.view_yaw, tol=0.3):
                yaw = self.body.current_yaw()
                if yaw is not None and abs(wrap(yaw - self.view_yaw)) < math.radians(5):
                    self._watch_t = time.time() + 2.0   # 2 s to settle first
            return False
        self.body.send_position_setpoint(*self.view, yaw=self.view_yaw)
        if time.time() < self._watch_t:
            self.phase = 'SETTLE'
            return False
        self.phase = 'WATCH'
        self.ticks += 1
        tr = self._track
        if tr and time.time() - tr[0] < 0.3 and len(tr[1]) >= 5 and tr[1][0] >= 2:
            cx, cy = tr[1][1], tr[1][2]
            self.centres.append((cx * 640, cy * 480))
            pos, q = self.body.current_position(), self.body.current_orientation_quat()
            hit = project_to_ground(2 * cx - 1, 2 * cy - 1, pos, q) if pos and q else None
            if hit and sim_truth.nearest(hit[0], hit[1])[1] <= self.MATCH_M:
                self.hits += 1
            else:
                self.false += 1
        if time.time() - self._watch_t >= self.WATCH_S:
            return self.finish()
        return False

    def measure(self):
        if not self.ticks:
            return {}
        jit = None
        if len(self.centres) > 5:
            mx = sum(c[0] for c in self.centres) / len(self.centres)
            my = sum(c[1] for c in self.centres) / len(self.centres)
            jit = round(math.sqrt(sum((c[0] - mx) ** 2 + (c[1] - my) ** 2 for c in self.centres) / len(self.centres)), 1)
        return {'detection_pct': round(100 * self.hits / self.ticks, 1),
                'false_alarm_pct': round(100 * self.false / self.ticks, 1),
                'jitter_px': jit}


class DisturbanceTrial(Trial):
    """Hold position, then Gazebo pushes Shadow sideways (ApplyLinkWrench,
    FORCE_N for PUSH_S). RECOVERY = time from the shove until back within
    0.15 m of the hold point and staying there. If the shove moved him
    less than 0.15 m it didn't land (plugin missing or wrong link name):
    the trial fails rather than report a fake recovery."""
    name = 'trial_disturbance'
    timeout_sec = 40.0
    FORCE_N = 3.0      # Nov 12: was 12 N, sized for the old 3.5 kg model (~1.2 m/s hit on the 1 kg MK1)
    PUSH_S = 0.4
    WINDOW_S = 12.0
    BAND_M = 0.15

    def _gz(self, topic, msgtype, payload):
        import os
        import subprocess
        world = os.environ.get('SHADOW_WORLD', 'shadow_runway')
        env = dict(os.environ, GZ_IP=os.environ.get('GZ_IP', '127.0.0.1'))
        try:
            subprocess.run(['gz', 'topic', '-t', f'/world/{world}/{topic}', '-m', msgtype, '-p', payload],
                           env=env, timeout=3.0, capture_output=True)
        except (OSError, subprocess.TimeoutExpired):
            pass

    def start(self):
        super().start()
        import os
        self.link = os.environ.get('SHADOW_WRENCH_LINK', 'shadow_with_ardupilot::shadow::base_link')
        self._push_t = None
        self._cleared = False

    def update(self):
        x, y, z = self.hold
        self.body.send_position_setpoint(x, y, z, yaw=self.hold_yaw)
        t = self.elapsed()
        if t < SETTLE_S:
            self.phase = 'SETTLE'
            return False
        if self._push_t is None:
            self.phase = 'SHOVE'
            self._gz('wrench/persistent', 'gz.msgs.EntityWrench',
                     f'entity: {{name: "{self.link}", type: LINK}}, wrench: {{force: {{y: {self.FORCE_N}}}}}')
            self._push_t = time.time()
            return False
        ts = time.time() - self._push_t
        if not self._cleared and ts >= self.PUSH_S:
            self._gz('wrench/clear', 'gz.msgs.Entity', f'name: "{self.link}", type: LINK')
            self._cleared = True
        self.phase = 'RECOVER'
        pos = self.body.current_position()
        if pos is not None:
            self.samples.append((ts, math.hypot(pos[0] - x, pos[1] - y)))
        if ts >= self.WINDOW_S:
            peak = max((d for _, d in self.samples), default=0.0)
            if peak < self.BAND_M:
                self.failed = (f'shove had no effect (moved {peak:.2f} m): check the ApplyLinkWrench '
                               f'plugin in the world and the link name {self.link!r}')
                return False
            return self.finish()
        return False

    def measure(self):
        peak = max((d for _, d in self.samples), default=0.0)
        rec = None
        for i, (ts, _) in enumerate(self.samples):
            if ts > self.PUSH_S and all(d <= self.BAND_M for _, d in self.samples[i:]):
                rec = ts
                break
        return {'recovery_s': round(rec, 2) if rec is not None else self.WINDOW_S,
                'shove_peak_m': round(peak, 2), 'recovered': rec is not None}


class LandingTrial(Trial):
    """Fly 4 m from home, then ArduPilot RTL. LANDING = distance from home
    at touchdown (disarm). Home = ArduPilot's home position if MAVROS
    provides it, otherwise the local origin, which is where Shadow spawned
    and armed in the suite. Ends the flight: run it last."""
    name = 'trial_landing'
    timeout_sec = 120.0

    def __init__(self, body):
        super().__init__(body)
        from mavros_msgs.msg import HomePosition
        from rclpy.qos import qos_profile_sensor_data
        self.home = None
        self.home_src = 'local origin (spawn / arm point)'
        body.create_subscription(HomePosition, '/mavros/home_position/home', self._home_cb, qos_profile_sensor_data)

    def _home_cb(self, msg):
        self.home = (msg.position.x, msg.position.y)
        self.home_src = 'ArduPilot home'

    def start(self):
        super().start()
        self.state = 'OUT'
        h = self.home or (0.0, 0.0)
        self.out = (h[0] + 4.0, h[1], 2.5)

    def update(self):
        pos = self.body.current_position()
        if self.state == 'OUT':
            self.phase = 'OUT'
            if _fly_phase(self, self.out, yaw=self.hold_yaw):
                self.body.set_mode('RTL')
                self.state = 'RTL'
            return False
        self.phase = 'RTL'
        if not self.body.is_armed() and pos is not None:
            h = self.home or (0.0, 0.0)
            self._land = math.hypot(pos[0] - h[0], pos[1] - h[1])
            return self.finish(notes=f'home from {self.home_src}')
        return False

    def limits(self):
        return True if getattr(self, 'state', None) == 'RTL' else super().limits()

    def measure(self):
        return {'landing_m': round(self._land, 3)}


def _suite_stages():
    from .arm import ArmSkill
    from .takeoff import TakeoffSkill
    return [
        ('arm', lambda b: ArmSkill(b)),
        ('takeoff', lambda b: TakeoffSkill(b, altitude_m=2.0)),
        ('hover hold', lambda b: HoverHoldTrial(b)),
        ('step response', lambda b: StepResponseTrial(b)),
        ('turn', lambda b: TurnTrial(b)),
        ('climb', lambda b: ClimbTrial(b)),
        ('disturbance', lambda b: DisturbanceTrial(b)),
        ('speed', lambda b: SpeedTrial(b)),
        ('go to', lambda b: GoToTrial(b)),
        ('lap', lambda b: LapTrial(b)),
        ('detection', lambda b: DetectionTrial(b)),
        ('landing', lambda b: LandingTrial(b)),
    ]
