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
"""

from geometry_msgs.msg import Point


class PayloadSensor:
    """Not a Node itself - subscribes through whatever Body/Node instance
    it's given, the same pattern as everything else in this package
    staying out of the rclpy node-ownership business except in Body."""

    def __init__(self, node, qos=10):
        self._point = Point()  # z=0 default means "nothing detected"
        node.create_subscription(
            Point, '/shadow/target_position', self._cb, qos)

    def _cb(self, msg):
        self._point = msg

    def has_target(self):
        """True once object_detector.py has actually found the payload
        in the current frame. z is contour area in pixels; 0 is its
        explicit "not detected" value, not a real reading."""
        return self._point.z > 0

    def target_offset(self):
        """(x, y) normalised -1..1, or None if nothing is detected right
        now. (0, 0) would mean dead-centre - don't confuse that with
        "no target", which is why this returns None instead of (0, 0)."""
        if not self.has_target():
            return None
        return (self._point.x, self._point.y)

    def target_area(self):
        """Raw contour area in pixels - 0 if nothing detected, otherwise
        bigger roughly means closer/larger in view. No calibrated
        distance conversion exists yet; treat this as relative, not a
        real distance measurement."""
        return self._point.z
