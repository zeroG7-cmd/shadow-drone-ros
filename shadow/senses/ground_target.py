#!/usr/bin/env python3
"""
GroundTargetSensor - turns "a box on the camera image" into "a point on
the ground in metres". Senses layer. Added Nov 9, 2026 for search,
approach_target and follow_target.

This is the camera + attitude + altitude fusion step: the detector gives
a direction (pixel position), MAVROS gives where the camera is and how
it is tilted (full quaternion, pitch and roll included, because a flying
Shadow is never level), and the ground plane gives the distance. The ray
from the camera through the pixel is followed until it hits the height
of the payload's centre.

Camera model (model.sdf front_camera): rigidly mounted 0.16 m ahead of
base_link, looking straight forward (no pitch), 640x480, horizontal FOV
80 deg (1.3962634 rad). If the camera mount is ever tilted down (the real
build has a pan/tilt gimbal), set CAM_PITCH_DEG to match or every
estimate will be wrong.

Consequence worth knowing: with the camera level, at 2.5 m altitude the
nearest visible ground is ~4 m ahead (half vertical FOV is ~32 deg).
Anything closer is under the camera's view. approach_target is shaped
around that.

Sources:
  'orange' - /shadow/target_position (geometry_msgs/Point, x/y in -1..1
             from the image centre, x right, y down, z = blob area,
             0 = nothing). Orange only, so the distractors are ignored.
             What the delivery mission uses.
  'track'  - /shadow/detection/track ([state, cx, cy, w, h, hits,
             misses], 0..1). Whatever the colour-blind tracker is
             locked on.
"""

import math
import time

CAM_OFFSET_M = (0.16, 0.0, 0.0)
CAM_PITCH_DEG = 0.0                 # positive = tilted down
TAN_HALF_HFOV = math.tan(1.3962634 / 2)
TAN_HALF_VFOV = TAN_HALF_HFOV * 480 / 640
TARGET_Z_M = 0.15                   # payload boxes are 0.3 m, centred at 0.15
MAX_RANGE_M = 20.0
MIN_AREA_PX = 50
SIGHTING_KEEP_S = 30.0


def quat_rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def project_to_ground(nx, ny, pos, quat, target_z=TARGET_Z_M):
    """nx, ny: image position from the centre, -1..1 (x right, y down).
    pos: drone (x, y, z) ENU. quat: body-to-world (x, y, z, w).
    Returns (x, y, range_m) on the plane z = target_z, or None if the ray
    doesn't come down to it (at or above the horizon, or too far)."""
    # ray in the camera frame (x forward, y left, z up), then the mount pitch
    d = (1.0, -nx * TAN_HALF_HFOV, -ny * TAN_HALF_VFOV)
    if CAM_PITCH_DEG:
        p = math.radians(CAM_PITCH_DEG)
        d = (d[0] * math.cos(p) + d[2] * math.sin(p), d[1],
             -d[0] * math.sin(p) + d[2] * math.cos(p))
    dw = quat_rotate(quat, d)
    off = quat_rotate(quat, CAM_OFFSET_M)
    cx, cy, cz = pos[0] + off[0], pos[1] + off[1], pos[2] + off[2]
    if dw[2] > -0.02 or cz <= target_z:
        return None
    t = (cz - target_z) / -dw[2]
    gx, gy = cx + t * dw[0], cy + t * dw[1]
    rng = math.hypot(gx - cx, gy - cy)
    if rng > MAX_RANGE_M:
        return None
    return gx, gy, rng


class GroundTargetSensor:
    def __init__(self, body, source='orange'):
        self.body = body
        self.source = source
        self.sightings = []          # (t, x, y, range)
        self.last_seen_t = None
        self.last_image = None       # (nx, ny)
        if source == 'orange':
            from geometry_msgs.msg import Point
            body.create_subscription(Point, '/shadow/target_position', self._point_cb, 10)
        elif source == 'track':
            from std_msgs.msg import Float32MultiArray
            body.create_subscription(Float32MultiArray, '/shadow/detection/track', self._track_cb, 10)
        else:
            raise ValueError(f'unknown source {source!r}')

    # ---- inputs ------------------------------------------------------
    def _point_cb(self, msg):
        if msg.z < MIN_AREA_PX:
            return
        self._add(msg.x, msg.y)

    def _track_cb(self, msg):
        d = list(msg.data)
        if len(d) < 5 or d[0] < 2:      # locked only
            return
        self._add(2 * d[1] - 1, 2 * d[2] - 1)

    def _add(self, nx, ny):
        now = time.time()
        self.last_seen_t = now
        self.last_image = (nx, ny)
        pos = self.body.current_position()
        quat = self.body.current_orientation_quat()
        if pos is None or quat is None:
            return
        hit = project_to_ground(nx, ny, pos, quat)
        if hit is None:
            return
        self.sightings.append((now, hit[0], hit[1], hit[2]))
        cutoff = now - SIGHTING_KEEP_S
        while self.sightings and self.sightings[0][0] < cutoff:
            self.sightings.pop(0)

    # ---- queries -----------------------------------------------------
    def clear(self):
        self.sightings = []

    def seen_within(self, seconds):
        return self.last_seen_t is not None and time.time() - self.last_seen_t <= seconds

    def latest(self, max_age_s=0.5):
        if self.sightings and time.time() - self.sightings[-1][0] <= max_age_s:
            return self.sightings[-1][1], self.sightings[-1][2]
        return None

    def estimate(self, window_s=10.0, radius_m=0.8):
        """Robust position from recent sightings: the median, then the
        mean of the sightings within radius_m of it (drops outliers and a
        second payload seen briefly). Returns (x, y, n, spread) or None."""
        now = time.time()
        pts = [(x, y) for t, x, y, _ in self.sightings if now - t <= window_s]
        if not pts:
            return None
        xs = sorted(p[0] for p in pts)
        ys = sorted(p[1] for p in pts)
        mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
        near = [p for p in pts if math.hypot(p[0] - mx, p[1] - my) <= radius_m]
        if not near:
            return None
        ex = sum(p[0] for p in near) / len(near)
        ey = sum(p[1] for p in near) / len(near)
        spread = max(math.hypot(p[0] - ex, p[1] - ey) for p in near)
        return ex, ey, len(near), spread
