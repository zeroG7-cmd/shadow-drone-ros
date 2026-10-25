#!/usr/bin/env python3
"""
Simulation camera stream - subscribes to Shadow's real front camera
sensor in Gazebo and serves it as an MJPEG stream over HTTP, so it can
be embedded directly in the Zero Command dashboard with a plain <img>
tag, no browser plugins or special video codecs needed.

Runs as its own small Flask app on port 5002, separate from the main
Zero Command app.

Oct 24, 2026 rewrite - the real root cause of "FPV shows LIVE but
stays solid black": this script used to go through rclpy + cv_bridge,
subscribing to a ROS2 topic, /shadow/front_camera/image_raw. NOTHING
IN THIS STACK HAS EVER PUBLISHED TO THAT ROS2 TOPIC. Gazebo's camera
sensor only ever published on its own native gz-transport topic,
shadow/front_camera/image_raw (no leading slash - a completely
different transport, not auto-bridged into ROS2) - see the comment on
the <sensor name="front_camera"> block in model.sdf, which already
called this out: bridging it to a real ROS2 topic needs ros_gz built
from source against Harmonic (ROS2 Humble's official ros_gz binaries
target Gazebo Fortress, not Harmonic - see
areas/shadow-ardupilot-migration-roadmap.md). That bridge work was
explicitly deferred during the Oct 2-4 ArduPilot/Harmonic migration
and never circled back to. So image_callback() was never once
invoked - latest_frame stayed None forever, and generate_mjpeg()
looped sending zero actual frame bytes. The <img> tag still connects
fine (200 OK, correct multipart headers - which is exactly why the
dashboard's "LIVE" badge/gating logic, keyed off a different signal,
correctly lit up), it just never receives a frame to paint.

This rewrite drops rclpy and cv_bridge from this script entirely and
subscribes straight to Gazebo's own gz-transport layer instead, using
Gazebo's own Python bindings (apt package python3-gz-transport13 -
matches this machine's Gazebo Harmonic / gz-sim8 install; gz-msgs10 is
the matching message-generation package, e.g. python3-gz-msgs10 if
not already pulled in as a dependency). No ROS2 bridge build needed.
Bonus: since this no longer touches rclpy or cv_bridge, it's also
unaffected either way by the separate numpy 1.x/2.x ABI issue cv_bridge
has (see object_detector.py's own log for that one - still open there).
And since this script no longer needs ROS2 node discovery to come up
at all, it can start the moment Gazebo itself is running, instead of
waiting behind ArduCopter SITL/MAVROS in run_full_sim.sh's launch
order - see the reordering in that script, same date.

Oct 25, 2026 - added the detection overlay (box + dot + "LOCKED ON")
Zer0 asked for after watching a mission live in the dashboard and not
seeing it: object_detector.py was already drawing this exact overlay,
onto its OWN separate topic (/shadow/detection_debug) that nothing
ever displayed - the dashboard's FPV panel only ever shows THIS
script's /video feed, the raw gz-transport frame with nothing drawn
on it. Rather than duplicate object_detector.py's HSV detection logic
here (two detectors that could drift out of sync), this adds one
small rclpy subscription to its lightweight output topic,
/shadow/target_position (one geometry_msgs/Point per frame, not the
image), spun on its own background thread so it never blocks Flask or
the gz-transport callback thread. The box drawn here is an
approximation - object_detector.py's Point only gives centre + area,
not real width/height - using sqrt(area) as an assumed-square side,
good enough to visually confirm "yes, it sees the target and roughly
where," which is all this is for. If that subscription never connects
(ROS2/object_detector.py not up yet), the raw feed still streams with
no overlay, same graceful-degradation behaviour this script already
had.
"""

import os
os.environ.pop("WERKZEUG_SERVER_FD", None)
os.environ.pop("WERKZEUG_RUN_MAIN", None)

# Oct 24 - apt's gz-msgs10 python bindings (image_pb2.py etc.) were
# generated against an older protobuf and use the old-style direct
# FieldDescriptor() construction. The protobuf package actually
# resolving at runtime on this machine (~/.local/lib/python3.10/
# site-packages/google/protobuf, likely pulled in by some unrelated
# pip package) is new enough to use the C++/upb implementation, which
# refuses that old-style construction outright ("Descriptors cannot
# be created directly... TypeError"). Forcing the pure-Python
# implementation here sidesteps it without touching any installed
# package version - scoped to just this one process's env, not a
# system-wide pip downgrade (see the numpy<2 fix elsewhere in this
# stack for why that's worth avoiding when there's a narrower option:
# it quietly put opencv-python, which declares numpy>=2, into a
# broken-metadata state instead). Must be set before gz.msgs10 is
# ever imported - protobuf locks in which implementation it's using
# the first time any _pb2 module loads.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import threading
import time

import numpy as np
import cv2
from flask import Flask, Response
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

import rclpy
from rclpy.node import Node as RclpyNode
from geometry_msgs.msg import Point

app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()

# No leading slash - this is a gz-transport topic name, not a ROS2
# one, even though it happens to share the same string Shadow's model
# was always configured with (see the <topic> element in model.sdf's
# front_camera sensor block).
GZ_TOPIC = "shadow/front_camera/image_raw"

# Must match deliver_payload.py's own LOCK_AREA_PX - this is purely a
# display threshold (when to show the "LOCKED ON" badge), deliberately
# kept as its own constant rather than importing deliver_payload.py
# itself (that module pulls in gz-transport entity-factory bindings
# and mission-specific state this display-only script has no business
# depending on). If the mission's real lock threshold is retuned,
# update this to match or the badge will show at a different point
# than the mission actually commits at.
LOCK_AREA_PX = 4000.0

latest_detection = Point()  # z=0 default -> "nothing detected", same as PayloadSensor
detection_lock = threading.Lock()
_overlay_error_count = 0  # see the throttle in image_callback's except block

# Oct 25, 2026 (Run 9, second half) - the overlay's own feel, separate
# from the Run 8/9 sensor-freshness and approach-control fixes
# (payload_sensor.py / deliver_payload.py). Zer0's own words after Run
# 8's fix: the box is genuinely live now but "looks a bit off and
# weird" - drifting rather than looking locked-on. Checked how
# production tracking pipelines handle this (e.g. Roboflow's own
# "detections stabilizer" block, inference.roboflow.com/workflows/
# blocks/detections_stabilizer): raw per-frame detector output is
# jittery by nature - cx/cy/area swing a little every frame even when
# the underlying detection is perfectly healthy - and the standard fix
# is an exponential moving average (EMA) on the box geometry actually
# drawn, plus holding the last known box for a short grace window if a
# detection briefly drops out, rather than redrawing raw per-frame noise
# or snapping the box to nothing and back. Smoothing lives entirely
# here, in what's drawn - it does NOT touch latest_detection itself, so
# deliver_payload.py's PayloadSensor (and its flight-control math) still
# reads the raw, unsmoothed Point. Smoothing a flight-control input
# would add lag nobody asked for; this is a display-only concern.
_EMA_ALPHA = 0.35          # weight on the new frame each tick - lower = smoother/slower, higher = snappier/more jitter
_smoothed_cx = None
_smoothed_cy = None
_smoothed_half_side = None

# Grace window for a detection that drops out and comes back quickly
# (object_detector.py losing the target for a frame or two isn't the
# same as actually losing track) - hold the last smoothed box rather
# than instantly vanishing it. Past this, drop it and show a dedicated
# "REACQUIRING" state instead of a confident-looking box that's
# secretly stale - this is what actually answers "is it losing track or
# not" from the FPV panel itself, instead of the box just silently
# going wrong.
STALE_AFTER_SEC = 0.75
_last_detection_time = 0.0


class DetectionListener(RclpyNode):
    """Tiny rclpy node whose only job is keeping latest_detection fresh
    for image_callback to draw. Spun on its own background thread (see
    main()) - deliberately separate from gz-transport's own delivery
    thread and from Flask's request threads, so a slow or missing ROS2
    side never blocks the raw video feed."""

    def __init__(self):
        super().__init__('sim_camera_stream_detection_listener')
        self.create_subscription(
            Point, '/shadow/target_position', self._cb, 10)

    def _cb(self, msg):
        global latest_detection, _last_detection_time
        with detection_lock:
            latest_detection = msg
            _last_detection_time = time.monotonic()


def _draw_detection_overlay(frame, point, age_sec):
    """Smoothed box + centre dot + lock-on/reacquiring badge, drawn
    straight onto the raw camera frame. point.z <= 0 means
    object_detector.py isn't currently seeing the target at all - draw
    nothing and reset the smoother (nothing real to smooth toward).
    age_sec is how long ago the underlying ROS message actually arrived
    (see _last_detection_time) - once that's stale past
    STALE_AFTER_SEC, show REACQUIRING instead of trusting an old box.
    Wrapped in try/except by the caller so a drawing mistake here can
    never take down the actual video feed."""
    global _smoothed_cx, _smoothed_cy, _smoothed_half_side

    if point.z <= 0:
        _smoothed_cx = _smoothed_cy = _smoothed_half_side = None
        return frame

    # Oct 25, 2026 fix - `frame` arrives as rgb[:, :, ::-1] from
    # image_callback: a VIEW over the read-only buffer np.frombuffer()
    # handed back, with a reversed (negative-stride) last axis on top
    # of that. cv2.rectangle/circle/putText all draw in place, and
    # OpenCV correctly refuses to do that on a read-only/non-contiguous
    # array ("img marked as output argument, but provided NumPy array
    # marked as readonly"). This was failing on every single frame -
    # confirmed live: camera_stream.log had over 13,000 copies of that
    # exact traceback from one run, which is also almost certainly why
    # the overlay never appeared no matter what the detector saw.
    # .copy() gives a fresh, writable, contiguous array to draw on.
    frame = frame.copy()

    height, width = frame.shape[:2]
    raw_cx = (point.x + 1.0) / 2.0 * width
    raw_cy = (point.y + 1.0) / 2.0 * height
    # object_detector.py only gives us centre + contour area, not a
    # real width/height - sqrt(area) as an assumed-square half-side is
    # an approximation, not the detector's actual bounding box.
    raw_half_side = max(point.z ** 0.5 / 2, 6)

    if _smoothed_cx is None:
        # First frame of a fresh lock (or right after a reset above) -
        # snap straight to the raw reading rather than easing in from
        # whatever the box's default would otherwise be.
        _smoothed_cx, _smoothed_cy, _smoothed_half_side = raw_cx, raw_cy, raw_half_side
    else:
        _smoothed_cx += _EMA_ALPHA * (raw_cx - _smoothed_cx)
        _smoothed_cy += _EMA_ALPHA * (raw_cy - _smoothed_cy)
        _smoothed_half_side += _EMA_ALPHA * (raw_half_side - _smoothed_half_side)

    cx, cy, half_side = int(_smoothed_cx), int(_smoothed_cy), int(_smoothed_half_side)
    locked = point.z >= LOCK_AREA_PX

    if age_sec > STALE_AFTER_SEC:
        # Held/stale - draw the last known smoothed box thinner and
        # say so plainly, instead of a confident-looking box that's
        # secretly out of date.
        cv2.rectangle(frame, (cx - half_side, cy - half_side),
                      (cx + half_side, cy + half_side), (0, 165, 255), 1)
        cv2.putText(frame, 'REACQUIRING...', (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 165, 255), 2, cv2.LINE_AA)
        return frame

    box_color = (0, 255, 0) if locked else (0, 165, 255)  # green when locked, orange while tracking

    cv2.rectangle(frame, (cx - half_side, cy - half_side),
                  (cx + half_side, cy + half_side), box_color, 2)
    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

    if locked:
        cv2.putText(frame, 'LOCKED ON', (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, f'TARGET area={point.z:.0f}', (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)

    return frame


def image_callback(msg: Image):
    global latest_frame
    # msg.step is the real per-row byte stride - may include row
    # padding, so don't assume it always equals width * 3. Slice each
    # row down to its real width*3 pixel bytes before reshaping.
    # pixel_format_type on this topic is RGB_INT8 (3 bytes/pixel),
    # matching model.sdf's <format>R8G8B8</format> - not re-checked
    # per-frame here since the sensor's format is fixed at model load,
    # not something that changes at runtime.
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    rgb = raw[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    bgr = rgb[:, :, ::-1]  # Gazebo sends RGB; cv2.imencode wants BGR

    try:
        with detection_lock:
            point = latest_detection
            age_sec = time.monotonic() - _last_detection_time
        bgr = _draw_detection_overlay(bgr, point, age_sec)
    except Exception as exc:  # overlay is an enhancement - never let it kill the feed
        # Oct 25, 2026 - throttled after the read-only-array bug above
        # logged this exact line 13,720 times in one run (5.4MB of
        # traceback text, once per frame, for the whole session) -
        # print it a few times so it's still visible, then go quiet
        # instead of flooding camera_stream.log indefinitely.
        global _overlay_error_count
        _overlay_error_count += 1
        if _overlay_error_count <= 5:
            print(f"WARNING: detection overlay draw failed: {exc}")
            if _overlay_error_count == 5:
                print("WARNING: detection overlay draw failed 5 times - "
                      "suppressing further copies of this message.")

    ok, jpeg = cv2.imencode('.jpg', bgr)
    if ok:
        with frame_lock:
            latest_frame = jpeg.tobytes()


def generate_mjpeg():
    while True:
        time.sleep(0.05)
        with frame_lock:
            frame = latest_frame
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video')
def video():
    return Response(generate_mjpeg(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    node = Node()
    # subscribe() registers the callback and returns - gz-transport
    # delivers messages on its own background threads from here (no
    # spin()/wait loop needed, unlike rclpy). It's fine to subscribe
    # before Gazebo has actually advertised the topic yet (e.g. right
    # after "gz sim" is launched) - discovery connects the two once
    # the sensor comes up, same as any other pub/sub match.
    subscribed = node.subscribe(Image, GZ_TOPIC, image_callback)
    if not subscribed:
        print(f"ERROR: failed to subscribe to gz-transport topic "
              f"'{GZ_TOPIC}' - node.subscribe() returned false.")

    # Detection overlay subscriber - rclpy, not gz-transport, so it
    # needs its own init + a background thread to spin it. Daemon
    # thread: this process already exits via Stop Simulation's pkill,
    # same as every other long-running node in this stack, so there's
    # no separate shutdown path to wire up here.
    rclpy.init(args=None)
    detection_node = DetectionListener()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(detection_node,), daemon=True)
    spin_thread.start()

    app.run(host='0.0.0.0', port=5002, threaded=True, debug=False, use_reloader=False)


if __name__ == '__main__':
    main()
