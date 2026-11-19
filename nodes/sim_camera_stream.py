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
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image as RosImage
from flask import jsonify

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import object_tracker as ot  # noqa: E402  (same folder; one drawing implementation shared with the detector)

app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()

# No leading slash - this is a gz-transport topic name, not a ROS2
# one, even though it happens to share the same string Shadow's model
# was always configured with (see the <topic> element in model.sdf's
# front_camera sensor block).
GZ_TOPIC = "shadow/front_camera/image_raw"

# Nov 8, 2026 - overlay rebuilt. The old version drew a square sized
# sqrt(contour area) around a centre point (the detector never sent a
# real box), turned green when area passed 4000 px (i.e. "close", not
# "locked"), and had no notion of sticking to one object - which is why
# it looked random. object_detector.py now runs object_tracker.py and
# publishes the real tracked box on /shadow/detection/track; this file
# just draws it with the same ot.draw_overlay() the detector uses, so
# there is one drawing implementation, not two that can drift apart.
# Smoothing and lock-on now live in the tracker, not here.
latest_track = []                 # flat list from /shadow/detection/track
track_lock = threading.Lock()
_last_track_time = 0.0
TRACK_STALE_SEC = 1.0             # past this, say the detector is offline rather than draw an old box
_overlay_error_count = 0          # see the throttle in image_callback's except block

latest_raw = None                 # last raw BGR frame, for /snapshot
raw_lock = threading.Lock()
latest_mask_jpeg = None           # last detector mask, for /mask
mask_lock = threading.Lock()
FRAMES_DIR = os.path.expanduser("~/shadow_sim_runtime/frames")


class DetectionListener(RclpyNode):
    """Tiny rclpy node: keeps the latest tracker output and mask fresh.
    Spun on its own background thread (see main()) so a slow or missing
    ROS2 side never blocks the raw video feed."""

    def __init__(self):
        super().__init__('sim_camera_stream_detection_listener')
        self.create_subscription(
            Float32MultiArray, '/shadow/detection/track', self._track_cb, 10)
        self.create_subscription(
            RosImage, '/shadow/detection_mask', self._mask_cb, 2)

    def _track_cb(self, msg):
        global latest_track, _last_track_time
        with track_lock:
            latest_track = list(msg.data)
            _last_track_time = time.monotonic()

    def _mask_cb(self, msg):
        global latest_mask_jpeg
        try:
            mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
            ok, jpeg = cv2.imencode('.jpg', mask)
            if ok:
                with mask_lock:
                    latest_mask_jpeg = jpeg.tobytes()
        except Exception:
            pass  # debug view only - never let it matter


def _draw_detection_overlay(frame, flat, age_sec):
    height, width = frame.shape[:2]
    if age_sec > TRACK_STALE_SEC:
        return ot.draw_overlay(frame, {"state": ot.STATE_NONE}, note="DETECTOR OFFLINE")
    return ot.draw_overlay(frame, ot.status_from_flat(flat, width, height))


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

    global latest_raw
    with raw_lock:
        latest_raw = bgr  # read-only view is fine; /snapshot copies before writing

    try:
        with track_lock:
            flat = list(latest_track)
            age_sec = time.monotonic() - _last_track_time
        bgr = _draw_detection_overlay(bgr, flat, age_sec)
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


def generate_mask_mjpeg():
    while True:
        time.sleep(0.1)
        with mask_lock:
            frame = latest_mask_jpeg
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/mask')
def mask():
    """What the detector actually sees: white = counts as part of an
    object, black = ignored. Open http://localhost:5002/mask next to the
    FPV view. Speckle everywhere = thresholds wrong; clean white blob on
    each prop = detector healthy."""
    return Response(generate_mask_mjpeg(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/snapshot')
def snapshot():
    """Saves the current RAW camera frame (no overlay) as a PNG in
    ~/shadow_sim_runtime/frames/, for test_detector.py to replay
    offline. Hit http://localhost:5002/snapshot once per test view."""
    with raw_lock:
        frame = None if latest_raw is None else np.ascontiguousarray(latest_raw).copy()
    if frame is None:
        return jsonify(ok=False, error='no camera frame received yet'), 503
    os.makedirs(FRAMES_DIR, exist_ok=True)
    path = os.path.join(FRAMES_DIR, time.strftime('frame_%Y%m%d_%H%M%S') + f'_{int(time.time()*1000)%1000:03d}.png')
    cv2.imwrite(path, frame)
    return jsonify(ok=True, path=path)


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
