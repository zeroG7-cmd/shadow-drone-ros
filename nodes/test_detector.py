#!/usr/bin/env python3
"""
Offline detector test - replays saved camera frames through the exact
same tracker the live detector uses (object_tracker.py), writes a copy of
each frame with the box drawn on, and prints numbers instead of vibes.

Get frames: with the sim running, open http://localhost:5002/snapshot
once per view you want to test (they land in ~/shadow_sim_runtime/frames).

Run:
    python3 ~/shadow_ws/src/shadow_drone/nodes/test_detector.py
    python3 test_detector.py /some/folder --mode orange

Annotated copies go in <folder>/annotated/. No ROS or Gazebo needed.

Frames are replayed in filename (= capture time) order as if they were a
video, so the tracker's lock-on and stickiness are exercised too. For
frames captured far apart in time that's only a rough check - a burst of
snapshots of one scene is the fair test for jitter.
"""

import argparse
import glob
import os
import statistics
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import object_tracker as ot  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder', nargs='?', default=os.path.expanduser('~/shadow_sim_runtime/frames'))
    ap.add_argument('--mode', choices=[ot.MODE_ANY, ot.MODE_ORANGE], default=ot.MODE_ANY)
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.folder, '*.png')) + glob.glob(os.path.join(args.folder, '*.jpg')))
    if not files:
        print(f'No .png/.jpg frames in {args.folder}. Grab some with http://localhost:5002/snapshot first.')
        return 1

    out_dir = os.path.join(args.folder, 'annotated')
    os.makedirs(out_dir, exist_ok=True)
    tracker = ot.Tracker()

    seen = locked = resets = 0
    centres = []
    was_tracking = False
    print(f'{"frame":<40} {"objects":>7} {"state":>9}  box (cx, cy, w, h) px')
    for path in files:
        bgr = cv2.imread(path)
        if bgr is None:
            print(f'{os.path.basename(path):<40} unreadable, skipped')
            continue
        h, w = bgr.shape[:2]
        cands = ot.find_candidates(ot.make_mask(bgr, args.mode))
        st = tracker.update(cands, w, h)
        name = ['none', 'tracking', 'LOCKED'][st['state']]
        box = '' if st['box'] is None else '(%.0f, %.0f, %.0f, %.0f)' % st['box']
        print(f'{os.path.basename(path):<40} {len(cands):>7} {name:>9}  {box}')

        if st['state'] != ot.STATE_NONE:
            seen += 1
            centres.append(st['box'][:2])
            was_tracking = True
        elif was_tracking:
            resets += 1
            was_tracking = False
        if st['state'] == ot.STATE_LOCKED:
            locked += 1
        cv2.imwrite(os.path.join(out_dir, os.path.basename(path)), ot.draw_overlay(bgr, st))

    n = len(files)
    print('\nSummary')
    print(f'  frames             {n}')
    print(f'  box shown          {seen}/{n} ({100.0 * seen / n:.0f}%)')
    print(f'  locked (green)     {locked}/{n} ({100.0 * locked / n:.0f}%)')
    print(f'  lost track         {resets} time(s)')
    if len(centres) >= 3:
        jx = statistics.pstdev([c[0] for c in centres])
        jy = statistics.pstdev([c[1] for c in centres])
        print(f'  box jitter         {jx:.1f} px horizontal, {jy:.1f} px vertical '
              '(only meaningful for a burst of the SAME still scene)')
    print(f'  annotated copies   {out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
