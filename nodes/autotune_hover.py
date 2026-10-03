#!/usr/bin/env python3
"""
Shadow autotune runner.

Gets Shadow to a stable hover in the open (empty) world and triggers
PX4's built-in multicopter autotune over MAVROS, then lands.

Reuses the exact arm/OFFBOARD/setpoint-streaming sequence already proven
working in mission_planner.py for getting off the ground - same connection
wait, same "don't arm until a real position estimate exists" check, same
setpoint-before-mode-switch ordering PX4 requires.

Sep 20 fix: the first real run climbed cleanly to 8m and reached the
trigger step, but PX4 rejected MAV_CMD_DO_AUTOTUNE_ENABLE almost instantly
with MAV_RESULT_UNSUPPORTED (result=3) - too fast (~5ms) to be an actual
tuning attempt failing, more consistent with a synchronous mode check
failing immediately. Re-checked PX4's docs (docs.px4.io, config/autotune_mc
+ v1.14/config/autotune - fetched 2026-09-20 evening) and found the
original assumption below was wrong: PX4 documents autotune as requiring
"an altitude-stabilized mode (such as Altitude mode, Hold mode, or
Position mode)" - OFFBOARD is NOT listed as accepted. The previous version
of this script stayed in OFFBOARD the whole time, including through the
trigger - that's the leading suspect for the rejection.

Fix: after climbing and settling in OFFBOARD (proven to work for takeoff),
switch to POSCTL (Position mode) before triggering autotune. POSCTL holds
position using PX4's own internal controller, so no setpoint stream is
needed once switched - which also frees up the setpoint_raw topic instead
of fighting with whatever autotune itself needs to inject into the
attitude/rate control loop during the tune.

Preconditions (docs.px4.io, config/autotune_mc + v1.14/config/autotune,
fetched 2026-09-20):
  - Vehicle must be actively hovering, 4-20m above ground.
  - Must be in an altitude-stabilized mode: Altitude, Hold, or Position.
    (This script uses Position/POSCTL.)
  - Fully automatic once triggered - no stick/setpoint changes during the
    tune.
  - MAV_CMD_DO_AUTOTUNE_ENABLE = 212 (NOT 108 - see fix note below, 108 was
    wrong and is why every attempt this session got rejected). param1=1
    enables it, param2=0 asks PX4 to use its default axis bitmask (all axes).
  - Typical duration ~40s, documented range 19-64s. This holds for 90s
    after triggering to leave real margin, then lands regardless.

Run in the `empty` world, NOT living_room (no ceiling headroom for an
8m climb) - i.e. `run_full_sim.sh empty`, then this.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, ParamGet
import time


HOVER_ALT = 8.0          # metres - mid-point of PX4's documented 4-20m autotune range
CLIMB_SECONDS = 8.0      # time to stream the climb setpoint before considering it "at altitude"
SETTLE_SECONDS = 10.0    # extra hold time (still OFFBOARD) before switching to POSCTL
POSCTL_SETTLE_SECONDS = 3.0   # time to let POSCTL's own position hold take over before triggering
AUTOTUNE_HOLD_SECONDS = 90.0  # covers PX4's documented up to ~64s tune, plus real margin
STATUS_LOG_INTERVAL = 10.0    # how often to log mode/armed state during the hold, so a long silent
                               # wait doesn't look identical to a hang in the log

# Sep 21 (night, round 3) - THE actual root cause of every rejection this
# whole session. Confirmed via a live debug print added straight into PX4's
# mavlink_receiver.cpp: the binary this SITL build is actually running has
# MAV_CMD_DO_AUTOTUNE_ENABLE compiled as 212, not 108. 108 was wrong from
# the start (came from outdated docs/memory of an older MAVLink dialect
# revision) - it was never a mode, timing, MC_AT_EN, or airframe problem.
# Command 108 doesn't match the autotune case in mavlink_receiver.cpp OR
# Commander.cpp, so it fell through to the generic "unrecognized command"
# default on both, producing the instant MAV_RESULT_UNSUPPORTED we kept
# seeing - nothing about the drone or the module was ever actually checked.
MAV_CMD_DO_AUTOTUNE_ENABLE = 212


class AutotuneRunner(Node):
    def __init__(self):
        super().__init__('autotune_runner')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.current_state = State()
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self._state_cb, qos)

        self.have_local_position = False
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._local_pos_cb, qos)

        self.setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', qos)

        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')
        self.command_client = self.create_client(
            CommandLong, '/mavros/cmd/command')
        self.param_get_client = self.create_client(
            ParamGet, '/mavros/param/get')

        self.get_logger().info('Waiting for MAVROS services...')
        # Sep 21 fix: a bare wait_for_service() blocks forever with no
        # feedback if a service never comes up - the autotune.log evidence
        # from a real run showed exactly one line ("Waiting for MAVROS
        # services...") and nothing after, so there was no way to tell which
        # of the four clients (arming/set_mode/command all proven working in
        # every prior run, param_get brand new this session) was actually
        # stuck. Bound each wait and log per-service so a hang is diagnosable
        # instead of silent.
        for svc_name, client in (
            ('/mavros/cmd/arming', self.arming_client),
            ('/mavros/set_mode', self.set_mode_client),
            ('/mavros/cmd/command', self.command_client),
        ):
            self.get_logger().info(f'  waiting for {svc_name} ...')
            if not client.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(
                    f'  {svc_name} did not become available after 15s - '
                    'aborting instead of hanging forever.')
                raise RuntimeError(f'{svc_name} service never appeared')
            self.get_logger().info(f'  {svc_name} available.')

        # Sep 21 (later) fix: /mavros/param/get never appeared within 15s on
        # a real run even though "Plugin param created/initialized" logged
        # cleanly with no error, and mavros.log showed zero param-pull
        # traffic the whole session (every other plugin - mission, geofence,
        # rallypoint - logged continuous activity). That looks like a real,
        # separate problem with the param plugin/service in this MAVROS
        # build, not something this script can fix. It was also a hard
        # requirement in __init__, which meant one broken diagnostic service
        # was blocking every single flight test of the actual bug we care
        # about (does the autotune trigger get accepted). Made it soft: wait
        # a short bound, warn and continue without it if it's not there,
        # instead of aborting the whole run.
        self.param_get_available = False
        self.get_logger().info('  waiting for /mavros/param/get ...')
        if self.param_get_client.wait_for_service(timeout_sec=5.0):
            self.param_get_available = True
            self.get_logger().info('  /mavros/param/get available.')
        else:
            self.get_logger().warn(
                '  /mavros/param/get did not become available after 5s - '
                'skipping the live MC_AT_EN check (this looks like a param '
                'plugin/service issue independent of the autotune bug). '
                'Proceeding on the assumption the airframe param already '
                'confirmed in the build reached the running vehicle - watch '
                'the trigger result closely.')
        self.get_logger().info('MAVROS services available.')

    def _state_cb(self, msg):
        self.current_state = msg

    def _local_pos_cb(self, msg):
        self.have_local_position = True

    def _publish_setpoint(self, xyz):
        IGNORE_YAW = 0b100000000000       # bit 11
        IGNORE_YAW_RATE = 0b1000000000000  # bit 12

        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = IGNORE_YAW | IGNORE_YAW_RATE
        target.position.x = xyz[0]
        target.position.y = xyz[1]
        target.position.z = xyz[2]
        self.setpoint_pub.publish(target)

    def _stream_setpoint_for(self, xyz, seconds, note=None):
        # Sep 20 (night) fix: rclpy.spin_once(timeout_sec=X) is a MAXIMUM
        # wait, not a guaranteed one - it returns as soon as it processes any
        # ready callback. /mavros/state and /mavros/local_position/pose are
        # streaming continuously, so the old cycles-based loop (publish once,
        # spin_once, repeat `seconds * rate_hz` times) finished in well under
        # the intended wall-clock duration every time - confirmed from a real
        # autotune.log: an 8.0s climb actually took 5.13s, a 10.0s settle
        # actually took 6.48s. Track real elapsed time against a deadline
        # instead, and pad each cycle back up to the target period so the
        # setpoint publish rate for OFFBOARD stays ~rate_hz.
        if note:
            self.get_logger().info(note)
        rate_hz = 20
        period = 1.0 / rate_hz
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            loop_start = time.monotonic()
            self._publish_setpoint(xyz)
            rclpy.spin_once(self, timeout_sec=0.0)
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    def _set_mode(self, mode_name, confirm_timeout=3.0, keep_streaming=None):
        req = SetMode.Request()
        req.custom_mode = mode_name
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not (future.result() and future.result().mode_sent):
            self.get_logger().error(f'Failed to set {mode_name} mode (message not even sent).')
            return False

        # Sep 21 (night, round 4) - THE real root cause of the RTL that
        # survived every battery fix. `mode_sent` only means mavros
        # transmitted the SET_MODE request - it does NOT mean PX4 actually
        # switched modes. Live .ulg data (nav_state field) proved PX4 never
        # once entered POSCTL (nav_state stayed 14/OFFBOARD until it jumped
        # straight to 5/AUTO_RTL) even though this function was logging
        # "POSCTL mode set." and moving on. The instant that happened, the
        # script stopped calling _stream_setpoint_for (the only thing
        # publishing OFFBOARD setpoints) and moved into _wait_logging_status,
        # which publishes nothing - so PX4, still actually in OFFBOARD with
        # zero incoming setpoints, tripped its offboard-signal-loss failsafe
        # within about a second and forced AUTO_RTL, well before autotune was
        # ever triggered. Confirm the REAL reported mode instead of trusting
        # mode_sent, and keep streaming setpoints while waiting so we don't
        # trip that same failsafe during the confirmation window itself.
        deadline = time.monotonic() + confirm_timeout
        while time.monotonic() < deadline:
            if keep_streaming is not None:
                self._publish_setpoint(keep_streaming)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_state.mode == mode_name:
                self.get_logger().info(f'{mode_name} mode confirmed active.')
                return True
        self.get_logger().error(
            f'{mode_name} was sent but PX4 never actually switched into it '
            f'(still reporting mode={self.current_state.mode}) - aborting '
            f'instead of flying on into a mode that was never really set.')
        return False

    def _arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Armed.')
            return True
        self.get_logger().error('Arming failed.')
        return False

    def _check_mc_at_en(self):
        # Sep 20 (night) diagnostic: MC_AT_EN defaults to 0 and gates
        # whether PX4 even starts the mc_autotune_attitude_control module
        # (rc.mc_apps: `if param greater -s MC_AT_EN 0 then
        # mc_autotune_attitude_control start fi`). Added
        # `param set-default MC_AT_EN 1` to Shadow's airframe file and
        # confirmed the build picked it up, but the trigger was rejected
        # again identically - so read the value straight from the running
        # vehicle instead of assuming the airframe fix reached it.
        if not self.param_get_available:
            return 'unavailable'
        req = ParamGet.Request()
        req.param_id = 'MC_AT_EN'
        future = self.param_get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result and result.success:
            self.get_logger().info(
                f'MC_AT_EN on the running vehicle = {result.value.integer} '
                f'(success={result.success})')
            return result.value.integer
        self.get_logger().error(
            f'Could not read MC_AT_EN from the vehicle: {result}')
        return None

    def _trigger_autotune(self):
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_AUTOTUNE_ENABLE
        req.confirmation = 0
        req.param1 = 1.0   # enable
        req.param2 = 0.0   # axis bitmask: 0 = PX4 default (all axes)
        req.param3 = 0.0   # timeout: 0 = no limit
        req.param4 = 0.0   # pole estimate timeout: 0 = no limit
        future = self.command_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        # Sep 21 (night, round 3) fix: MAV_CMD_DO_AUTOTUNE_ENABLE is a
        # long-running command. Per mavlink_receiver.cpp's own handling
        # (read straight from PX4 source this session), PX4 acks it as
        # VEHICLE_CMD_RESULT_IN_PROGRESS (5) repeatedly while the tune is
        # actually running, and only sends ACCEPTED (0, success=True) once
        # the whole tune genuinely completes. mavros' `success` field is
        # only True for ACCEPTED, so treating success==False as a rejection
        # was wrong for this one command - it made a real, in-progress tune
        # look identical to an actual denial and landed immediately instead
        # of letting it run. Only DENIED/UNSUPPORTED/FAILED are real
        # rejections now.
        VEHICLE_CMD_RESULT_IN_PROGRESS = 5
        if result and (result.success or result.result == VEHICLE_CMD_RESULT_IN_PROGRESS):
            self.get_logger().info(
                f'Autotune command accepted/in progress (result code {result.result}).')
            return True
        self.get_logger().error(
            f'Autotune command NOT accepted: {result}')
        return False

    def _wait_logging_status(self, seconds, note=None):
        # Same real-elapsed-time fix as _stream_setpoint_for - `elapsed +=
        # step` assumed spin_once always blocks the full `step`, which it
        # doesn't. This was the worst-hit case: the 3.0s POSCTL_SETTLE_SECONDS
        # call actually returned in 0.13s per a real autotune.log, meaning
        # PX4 had essentially no time to complete the OFFBOARD->POSCTL
        # transition before autotune was triggered - the leading suspect for
        # the continued MAV_RESULT_UNSUPPORTED rejection even after adding
        # the POSCTL switch itself.
        if note:
            self.get_logger().info(note)
        start = time.monotonic()
        next_log = STATUS_LOG_INTERVAL
        while (time.monotonic() - start) < seconds and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - start
            if elapsed >= next_log:
                self.get_logger().info(
                    f'  ...{elapsed:.0f}s in - mode={self.current_state.mode}, '
                    f'armed={self.current_state.armed}')
                next_log += STATUS_LOG_INTERVAL
                if not self.current_state.armed:
                    self.get_logger().warn(
                        'Vehicle disarmed during the hold window - stopping wait early.')
                    return

    def run(self):
        home = (0.0, 0.0, HOVER_ALT)

        self.get_logger().info('Waiting for MAVROS connection to PX4...')
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self, timeout_sec=0.5)

        mc_at_en = self._check_mc_at_en()
        if mc_at_en == 'unavailable':
            self.get_logger().warn(
                'Could not verify MC_AT_EN live (param service unavailable) - '
                'flying anyway. The trigger result below is now the real signal.')
        elif mc_at_en is None or mc_at_en <= 0:
            self.get_logger().error(
                'MC_AT_EN is not enabled on this vehicle (mc_autotune_attitude_control '
                'never started, so nothing will ever accept the autotune command) - '
                'aborting before takeoff instead of wasting a flight. Fix the param on '
                'the running vehicle (not just the airframe source) and restart.')
            return

        self.get_logger().info(
            'Connected. Waiting for a valid position estimate before arming...')
        while rclpy.ok() and not self.have_local_position:
            rclpy.spin_once(self, timeout_sec=0.5)

        self._stream_setpoint_for(
            home, 2.0, note='Position estimate confirmed. Streaming initial setpoints...')

        if not self._set_mode('OFFBOARD', keep_streaming=home):
            return
        if not self._arm():
            return

        self._stream_setpoint_for(
            home, CLIMB_SECONDS,
            note=f'Climbing to {HOVER_ALT}m...')

        self._stream_setpoint_for(
            home, SETTLE_SECONDS,
            note=f'At {HOVER_ALT}m. Settling before switching to POSCTL...')

        # Switch out of OFFBOARD into POSCTL - PX4 documents autotune as
        # requiring an altitude-stabilized mode (Altitude/Hold/Position),
        # not OFFBOARD. POSCTL holds position with PX4's own controller,
        # so no more setpoint streaming is needed after this.
        if not self._set_mode('POSCTL', keep_streaming=home):
            self.get_logger().error('Could not reach POSCTL. Landing.')
            self._land()
            return

        self._wait_logging_status(
            POSCTL_SETTLE_SECONDS,
            note='POSCTL set - letting position hold take over before triggering autotune...')

        self.get_logger().info(
            'Triggering autotune (MAV_CMD_DO_AUTOTUNE_ENABLE)...')
        if not self._trigger_autotune():
            self.get_logger().error(
                'Aborting - autotune was not accepted. Landing.')
            self._land()
            return

        self._wait_logging_status(
            AUTOTUNE_HOLD_SECONDS,
            note=('Autotune running in POSCTL - not sending any other commands '
                  'until this completes. Watch the dashboard.'))

        self.get_logger().info('Autotune hold window complete. Landing.')
        self._land()

    def _land(self):
        self._set_mode('AUTO.LAND')
        self.get_logger().info('Landing commanded. Mission script done.')


def main(args=None):
    rclpy.init(args=args)
    node = AutotuneRunner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
