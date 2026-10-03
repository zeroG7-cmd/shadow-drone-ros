#!/usr/bin/env python3
"""
Simulation camera stream - subscribes to Shadow's real front camera
topic in Gazebo and serves it as an MJPEG stream over HTTP, so it can
be embedded directly in the Zero Command dashboard with a plain <img>
tag, no browser plugins or special video codecs needed.

Runs as its own small Flask app on port 5002, separate from the main
Zero Command app - it needs its own ROS2 subscriber thread running
alongside Flask's own server thread.

Topic name confirmed against Shadow's real, live sensor setup:
/shadow/front_camera/image_raw (not the old placeholder
/camera_sensor/image_raw from earlier in the project).
"""

import os
os.environ.pop("WERKZEUG_SERVER_FD", None)
os.environ.pop("WERKZEUG_RUN_MAIN", None)

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response

app = Flask(__name__)
bridge = CvBridge()
latest_frame = None
frame_lock = threading.Lock()


class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('sim_camera_stream')

        # Match gazebo_ros_camera's publisher QoS (SensorDataQoS / best-effort)
        # - a plain integer depth here defaults to reliable QoS, which is
        # incompatible with a best-effort publisher and silently never
        # connects (same root cause as object_detector.py's dead HSV log).
        self.subscription = self.create_subscription(
            Image,
            "/shadow/front_camera/image_raw",
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("Subscribed to /shadow/front_camera/image_raw")

    def image_callback(self, msg):
        global latest_frame
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, jpeg = cv2.imencode('.jpg', frame)
        if ok:
            with frame_lock:
                latest_frame = jpeg.tobytes()


def generate_mjpeg():
    import time
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


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = CameraStreamNode()

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    app.run(host='0.0.0.0', port=5002, threaded=True, debug=False, use_reloader=False)


if __name__ == '__main__':
    main()
