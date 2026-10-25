#!/usr/bin/env python3
"""
Senses layer - PayloadSensor wraps object_detector.py's output so a
Skill never has to subscribe to a topic or know what message type it is.

object_detector.py publishes geometry_msgs/Point on
/shadow/target_position:
  x, y - normalised image position, -1 to 1 (0,0 = dead centre of frame)
  z    - contour area in pixels; 0 means nothing detected

This class just remembers the latest one and answers plain questions
("is anything detected", "how far off-centre is it") instead of handing
a Skill the raw Point and making it re-derive that logic every time.

Oct 25, 2026 rewrite - this used to subscribe through whatever Node it
was handed (deliver_payload.py passes self.body, the ArduPilotBody),
sharing that node's spin budget with /mavros/state and
/mavros/local_position/pose. run_skill.py's tick loop originally called
rclpy.spin_once() exactly once per tick, which only ever services ONE
ready callback - with state/pose always busy and registered first,
this subscription basically never got serviced after its first lucky
message (see areas/search-and-deliver-mission.md, Run 7). Widening
run_skill.py's tick loop to drain up to 10 spin_once calls per tick
helped (it's no longer frozen solid for an entire run) but still only
lets a handful of fresh Point messages through per run instead of a
smooth stream every tick, because it's still competing for a shared,
limited budget against two other subscriptions that are busier than it
is.

The real fix: don't share. This now owns its OWN rclpy Node and spins
it continuously on its own dedicated background thread - the exact
same pattern already proven working in sim_camera_stream.py's
DetectionListener. That decouples this subscription completely from
however often (or rarely) whatever tick loop happens to call
spin_once() on some other node - it's never waiting its turn. The
`node` constructor argument is kept, but unused, purely so existing
call sites (deliver_payload.py's `PayloadSensor(self.body)`) don't need
editing.
"""

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class PayloadSensor:
    """Owns its own Node, spun on its own background thread - see the
    module docstring for why this stopped sharing whatever Node/spin
    budget the caller has. `node` is accepted but unused (kept for
    call-site compatibility)."""

    def __init__(self, node=None, qos=10):
        self._point = Point()  # z=0 default means "nothing detected"
        self._lock = threading.Lock()

        self._node = Node('payload_sensor_listener')
        self._node.create_subscription(
            Point, '/shadow/target_position', self._cb, qos)

        # Daemon thread, same reasoning sim_camera_stream.py's
        # DetectionListener already documents: this process exits via
        # run_skill.py's own run (or Stop Simulation's pkill) long
        # before any clean shutdown of this one extra node would
        # matter, so there's no separate shutdown path to wire up.
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    def _cb(self, msg):
        with self._lock:
            self._point = msg

    def has_target(self):
        """True once object_detector.py has actually found the payload
        in the current frame. z is contour area in pixels; 0 is its
        explicit "not detected" value, not a real reading."""
        with self._lock:
            z = self._point.z
        return z > 0

    def target_offset(self):
        """(x, y) normalised -1..1, or None if nothing is detected right
        now. (0, 0) would mean dead-centre - don't confuse that with
        "no target", which is why this returns None instead of (0, 0)."""
        with self._lock:
            point = self._point
        if point.z <= 0:
            return None
        return (point.x, point.y)

    def target_area(self):
        """Raw contour area in pixels - 0 if nothing detected, otherwise
        bigger roughly means closer/larger in view. No calibrated
        distance conversion exists yet; treat this as relative, not a
        real distance measurement."""
        with self._lock:
            return self._point.z
