#!/usr/bin/env python3
"""
Object tracker - the colour-blind "is there an object in front of me,
and keep the box on it" core used by object_detector.py (live), the FPV
overlay in sim_camera_stream.py (drawing), and test_detector.py (offline
test on saved frames). Pure numpy + OpenCV: no ROS, no Gazebo, so it can
be tested anywhere.

Nov 8, 2026 - written because the old overlay looked random. Three real
causes, found by reading the code, not guessed:
  1. The FPV box was never the detector's real box. sim_camera_stream.py
     only received a centre point + contour area and drew a square of
     side sqrt(area) - so the box grew and shrank with the blob's area,
     not the object's shape.
  2. "LOCKED ON" meant area >= 4000 px (i.e. close), not "seen steadily".
  3. Nothing stuck to one object - every frame took the largest orange
     blob, so the box could hop between blobs.

What this does instead:
  - MODE_ANY (default): any strongly coloured blob counts as an object.
    Works in shadow_runway.sdf because the background (0.8 grey,
    saturation ~8/255 in the real logs) and the delivery pad are almost
    colourless, while every payload prop is strongly coloured. It knows
    THAT something is there, not WHAT it is - which is why the label
    says "object", not "payload". Telling payloads apart is YOLO's job.
  - Shape filter: blobs must be chunky (aspect <= MAX_ASPECT, fill >=
    MIN_FILL). This rejects the world's long thin red/green/blue "axes"
    lines and any thin runway markings.
  - Tracking: once it has an object, each frame it picks the candidate
    nearest to where the box was (within a gate), so it sticks to the
    same object instead of hopping. Box position/size is smoothed (EMA).
  - Lock-on: green only after LOCK_FRAMES consecutive good frames. Brief
    misses (up to MAX_MISSES frames) hold the box rather than dropping
    it; past that, the track resets to searching.

MODE_ORANGE is kept for the mission (deliver_payload.py must keep
ignoring the blue/green/purple distractors until YOLO exists).
"""

import cv2
import numpy as np

MODE_ANY = "any"
MODE_ORANGE = "orange"

# "Strongly coloured" for MODE_ANY. Background logs at S=8; the props are
# near-fully saturated, so 90 leaves a wide margin either side.
S_MIN = 90
V_MIN = 50
ORANGE_LO = np.array([5, 100, 100])
ORANGE_HI = np.array([25, 255, 255])

MIN_AREA_PX = 120      # ignore specks
MAX_ASPECT = 3.0       # longest side / shortest side - rejects thin lines
MIN_FILL = 0.45        # blob area / its bounding box area - rejects scraggly shapes

# Tracking (the detector runs at roughly 11 fps on this machine, per the
# logged HSV line cadence - these are tuned for that, not 30 fps).
LOCK_FRAMES = 8        # ~0.7 s of steady sighting before green
MAX_MISSES = 5         # ~0.45 s of hold before giving up
GATE_FRAC = 0.25       # max jump between frames, as a fraction of frame size
POS_ALPHA = 0.65       # box position smoothing: higher = snappier (follows motion), lower = smoother
SIZE_ALPHA = 0.35      # box size smoothing: kept heavier, size flicker was the old complaint

STATE_NONE, STATE_TRACKING, STATE_LOCKED = 0, 1, 2


def make_mask(bgr, mode=MODE_ANY):
    """Black/white mask of pixels that could belong to an object."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if mode == MODE_ORANGE:
        mask = cv2.inRange(hsv, ORANGE_LO, ORANGE_HI)
    else:
        mask = cv2.inRange(hsv, np.array([0, S_MIN, V_MIN]), np.array([179, 255, 255]))
    # Opening with a 5x5 kernel wipes out anything thinner than ~5 px -
    # specks AND the world's thin axis lines - so a line touching a prop
    # can't merge into it and make one long blob the shape filter then
    # rejects (caught in synthetic testing, Nov 8). A 0.3 m prop at 10 m
    # is still ~11 px across on this 640 px / 80 deg camera, so real
    # props survive it.
    open_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_k, iterations=2)  # fill small holes
    return mask


def find_candidates(mask):
    """Chunky blobs only. Returns a list of (cx, cy, w, h, area) in pixels."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(c)
        aspect = max(w, h) / float(max(1, min(w, h)))
        fill = area / float(max(1, w * h))
        if aspect > MAX_ASPECT or fill < MIN_FILL:
            continue
        out.append((x + w / 2.0, y + h / 2.0, float(w), float(h), area))
    return out


class Tracker:
    """Sticks a smoothed box to one object across frames."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.box = None    # [cx, cy, w, h] in pixels, smoothed
        self.hits = 0      # consecutive frames the object was seen
        self.misses = 0    # consecutive frames it wasn't
        self.locked = False

    def update(self, candidates, frame_w, frame_h):
        if self.box is None:
            if candidates:
                best = max(candidates, key=lambda c: c[4])  # start on the biggest
                self.box = list(best[:4])
                self.hits, self.misses = 1, 0
            return self.status(frame_w, frame_h)

        gate = GATE_FRAC * max(frame_w, frame_h)
        best, best_d = None, None
        for c in candidates:
            d = ((c[0] - self.box[0]) ** 2 + (c[1] - self.box[1]) ** 2) ** 0.5
            if best_d is None or d < best_d:
                best, best_d = c, d

        if best is not None and best_d <= gate:
            for i in range(4):
                a = POS_ALPHA if i < 2 else SIZE_ALPHA
                self.box[i] += a * (best[i] - self.box[i])
            self.hits += 1
            self.misses = 0
            if self.hits >= LOCK_FRAMES:
                self.locked = True
        else:
            self.misses += 1
            if self.misses > MAX_MISSES:
                self.reset()
        return self.status(frame_w, frame_h)

    def status(self, frame_w, frame_h):
        """Plain dict, plus a flat list for publishing over ROS."""
        if self.box is None:
            return {"state": STATE_NONE, "box": None, "hits": 0, "misses": 0,
                    "flat": [float(STATE_NONE), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        state = STATE_LOCKED if self.locked else STATE_TRACKING
        cx, cy, w, h = self.box
        flat = [float(state), cx / frame_w, cy / frame_h, w / frame_w, h / frame_h,
                float(self.hits), float(self.misses)]
        return {"state": state, "box": (cx, cy, w, h), "hits": self.hits,
                "misses": self.misses, "flat": flat}


def status_from_flat(flat, frame_w, frame_h):
    """Inverse of status()['flat'] - used by the FPV stream, which gets the
    track over ROS as normalised numbers and may draw on a different size."""
    if not flat or int(flat[0]) == STATE_NONE:
        return {"state": STATE_NONE, "box": None, "hits": 0, "misses": 0}
    return {"state": int(flat[0]),
            "box": (flat[1] * frame_w, flat[2] * frame_h, flat[3] * frame_w, flat[4] * frame_h),
            "hits": int(flat[5]), "misses": int(flat[6])}


YELLOW = (0, 215, 255)   # BGR
GREEN = (80, 220, 60)
GREY = (170, 170, 170)


def draw_overlay(bgr, status, label="object", note=None):
    """One box: yellow while acquiring, green once locked, small label on
    top. No crosshair, no dot. Draws on a copy and returns it."""
    frame = np.ascontiguousarray(bgr).copy()
    font = cv2.FONT_HERSHEY_SIMPLEX

    if note:
        cv2.putText(frame, note, (12, 26), font, 0.55, GREY, 1, cv2.LINE_AA)
        return frame

    if status["state"] == STATE_NONE:
        cv2.putText(frame, "SEARCHING", (12, 26), font, 0.6, YELLOW, 2, cv2.LINE_AA)
        return frame

    cx, cy, w, h = status["box"]
    x0, y0 = int(cx - w / 2), int(cy - h / 2)
    x1, y1 = int(cx + w / 2), int(cy + h / 2)
    locked = status["state"] == STATE_LOCKED
    colour = GREEN if locked else YELLOW
    thickness = 1 if status.get("misses", 0) > 0 else 2  # thinner while holding through a miss

    cv2.rectangle(frame, (x0, y0), (x1, y1), colour, thickness)

    text = label + ("  LOCKED" if locked else "")
    (tw, th), base = cv2.getTextSize(text, font, 0.5, 1)
    ty = y0 - 6 if y0 - th - 10 > 0 else y1 + th + 8
    cv2.rectangle(frame, (x0, ty - th - 4), (x0 + tw + 8, ty + base), colour, -1)
    cv2.putText(frame, text, (x0 + 4, ty - 1), font, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
    return frame
