# MIT License
#
# Copyright (c) 2026
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Bottle Capping Gesture Detector  -  FIWARE IoTA-UL Edition
===========================================================
Detects three states based on index + middle finger direction held for 13 s:

  NO_HAND    - No hand visible.
  CAP_PLACED - Index + middle pointing DOWN, stationary for 1 s.
  SIDE_GRIP  - Index + middle pointing HORIZONTAL, stationary for 1 s.

On every state change the detector POSTs to the FIWARE:

    PATCH /v2/entities/GestureDetector:operator-1/attrs
    Content-Type: application/json
    Fiware-Service:     openiot
    Fiware-Servicepath: /

    {"command": {"type": "Text", "value": "CAP_PLACED"}}

FIWARE settings
---------------
    broker             = "http://localhost:1026"
    fiware_service     = "openiot"
    fiware_servicepath = "/"

Dependencies:
    pip install opencv-python mediapipe numpy requests

On first run the MediaPipe hand landmark model (~9 MB) is auto-downloaded.

Usage:
    python bottle_gesture_detector.py
    python bottle_gesture_detector.py --no-fiware   # run detector only
    python bottle_gesture_detector.py --no-gui      # headless, no OpenCV window
    python bottle_gesture_detector.py --no-gui --no-fiware  # pure stdout mode
    Press 'q' to quit (GUI mode only). Use Ctrl-C in headless mode.
"""

import argparse
import math
import os
import time
import urllib.request
from collections import deque
import cv2
import mediapipe as mp
import numpy as np
import requests
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

# ──────────────────────────────────────────────────────────────────────────────
# CLI  –  mirrors the constants you'd set in an M5Stick sketch
# ──────────────────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser(description='Bottle Capping Gesture Detector')
parser.add_argument('--broker',             default='http://localhost:1026',
                    help='Orion Context Broker base URL (default: http://localhost:1026)')
parser.add_argument('--fiware-service',     default='openiot',
                    help='Fiware-Service header     (default: openiot)')
parser.add_argument('--fiware-servicepath', default='/',
                    help='Fiware-Servicepath header (default: /)')
parser.add_argument('--no-fiware',          action='store_true',
                    help='Disable FIWARE publishing (run detector only)')
parser.add_argument('--no-gui',             action='store_true',
                    help='Headless mode: skip OpenCV window and all drawing')
parser.add_argument('--camera',             type=int, default=0,
                    help='Camera device index (default: 0)')
ARGS = parser.parse_args()

# ──────────────────────────────────────────────────────────────────────────────
# Gesture / detection configuration
# ──────────────────────────────────────────────────────────────────────────────

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/"
             "hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task")
MODEL_PATH = "hand_landmarker.task"

CONFIRM_SECONDS = 0.5    # hold duration required
STILL_THRESHOLD_PX = 25.0   # px/s max wrist speed to count as stationary
DOWN_ANGLE_TOLERANCE = 20.0   # degrees from vertical  → "pointing down"
HORIZ_ANGLE_TOLERANCE = 20.0   # degrees from horizontal → "pointing horizontal"

# ──────────────────────────────────────────────────────────────────────────────
# States
# ──────────────────────────────────────────────────────────────────────────────

NO_HAND = 'NO_HAND'
CAP_PLACED = 'CAP_PLACED'
SIDE_GRIP = 'SIDE_GRIP'

# ──────────────────────────────────────────────────────────────────────────────
# Landmark indices
# ──────────────────────────────────────────────────────────────────────────────

WRIST = 0
INDEX_MCP = 5
INDEX_TIP = 8
MIDDLE_MCP = 9
MIDDLE_TIP = 12

# ──────────────────────────────────────────────────────────────────────────────
# FIWARE publisher  –  NGSIv2 REST (same pattern as the speech recognizer)
#   POST /v2/entities              to create entity on first call
#   PATCH /v2/entities/<id>/attrs  on every subsequent update
#   Headers: Fiware-Service / Fiware-Servicepath
# ──────────────────────────────────────────────────────────────────────────────

ENTITY_ID = 'GestureDetector:operator-1'
ENTITY_TYPE = 'GestureDetector'


class FiwarePublisher:
    def __init__(self, broker_url, fiware_service, fiware_servicepath, enabled=True):
        self.enabled = enabled
        self._broker = broker_url.rstrip('/')
        self._entity_url = f"{self._broker}/v2/entities/{ENTITY_ID}"
        self._attrs_url = f"{self._entity_url}/attrs"
        self._headers = {
            'Content-Type':       'application/json',
            'Accept':             'application/json',
            'Fiware-Service':     fiware_service,
            'Fiware-Servicepath': fiware_servicepath,
        }
        self._session = requests.Session()
        self._created = False
        self._last_ok = False

    def _ensure_entity(self):
        if self._created:
            return
        body = {
            'id':   ENTITY_ID,
            'type': ENTITY_TYPE,
            'command': {'type': 'Text', 'value': ''},
        }
        try:
            r = self._session.post(
                f"{self._broker}/v2/entities",
                headers=self._headers,
                json=body,
                timeout=5,
            )
            if r.status_code == 201:
                print(f"[FIWARE] Entity created: {ENTITY_ID}")
            elif r.status_code == 422:
                print(f"[FIWARE] Entity already exists, will PATCH attributes.")
            else:
                print(
                    f"[FIWARE] WARNING: entity creation returned {r.status_code}: {r.text[:80]}")
                return
        except requests.exceptions.ConnectionError:
            print(f"[FIWARE] ERROR: cannot reach broker at {self._broker}")
            print("         Is Orion running?  See docker-compose.yml")
            return
        self._created = True

    def publish(self, state):
        if not self.enabled:
            return True
        self._ensure_entity()
        if not self._created:
            self._last_ok = False
            return False
        body = {'command': {'type': 'Text', 'value': state}}
        try:
            r = self._session.patch(
                self._attrs_url,
                headers=self._headers,
                json=body,
                timeout=5,
            )
            self._last_ok = r.status_code == 204
            if not self._last_ok:
                print(
                    f"[FIWARE] WARNING: PATCH returned {r.status_code}: {r.text[:80]}")
            return self._last_ok
        except requests.exceptions.ConnectionError:
            if self._last_ok:
                print(
                    f"[FIWARE] ERROR: lost connection to broker at {self._broker}")
            self._last_ok = False
            return False
        except requests.exceptions.Timeout:
            self._last_ok = False
            return False

    def check_connection(self):
        try:
            r = self._session.get(f"{self._broker}/v2/entities", timeout=3)
            return r.status_code < 500
        except requests.exceptions.RequestException:
            return False

    @property
    def last_ok(self):
        return self._last_ok

    def close(self):
        self._session.close()


# ──────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ──────────────────────────────────────────────────────────────────────────────

def lm_xy(landmarks, idx, w, h):
    lm = landmarks[idx]
    return np.array([lm.x * w, lm.y * h], dtype=float)


def finger_direction_angle(mcp, tip):
    """Returns (angle_from_vertical, angle_from_horizontal) in degrees."""
    vec = tip - mcp
    length = np.linalg.norm(vec) + 1e-6
    uv = vec / length
    # +y is downward in image coords, so straight down = (0, 1)
    angle_v = math.degrees(
        math.acos(np.clip(np.dot(uv, [0.0, 1.0]), -1.0, 1.0)))
    angle_h = min(
        math.degrees(math.acos(np.clip(np.dot(uv, [1.0, 0.0]), -1.0, 1.0))),
        math.degrees(math.acos(np.clip(np.dot(uv, [-1.0, 0.0]), -1.0, 1.0))),
    )
    return angle_v, angle_h


def get_raw_pose(landmarks, w, h):
    """Returns 'down', 'horizontal', or None."""
    down_count = horiz_count = 0
    for mcp_i, tip_i in [(INDEX_MCP, INDEX_TIP), (MIDDLE_MCP, MIDDLE_TIP)]:
        av, ah = finger_direction_angle(
            lm_xy(landmarks, mcp_i, w, h),
            lm_xy(landmarks, tip_i, w, h))
        if av <= DOWN_ANGLE_TOLERANCE:
            down_count += 1
        if ah <= HORIZ_ANGLE_TOLERANCE:
            horiz_count += 1
    if down_count == 2:
        return 'down'
    if horiz_count == 2:
        return 'horizontal'
    return None


# ──────────────────────────────────────────────────────────────────────────────
# Wrist velocity tracker
# ──────────────────────────────────────────────────────────────────────────────

class VelocityTracker:
    def __init__(self, window=8):
        self.positions = deque(maxlen=window)
        self.times = deque(maxlen=window)

    def update(self, pos, t):
        self.positions.append(np.array(pos))
        self.times.append(t)

    def speed(self):
        if len(self.positions) < 2:
            return 0.0
        dp = self.positions[-1] - self.positions[0]
        dt = self.times[-1] - self.times[0] + 1e-6
        return float(np.linalg.norm(dp / dt))

    def is_still(self):
        return self.speed() < STILL_THRESHOLD_PX


# ──────────────────────────────────────────────────────────────────────────────
# Hold timer  –  3-second confirmation gate
# ──────────────────────────────────────────────────────────────────────────────

class HoldTimer:
    def __init__(self, required_seconds=CONFIRM_SECONDS):
        self.required = required_seconds
        self._pose = None
        self._start = None
        self._done = False

    def update(self, pose, is_still):
        if pose is None or not is_still:
            self._pose = None
            self._start = None
            self._done = False
            return
        if pose != self._pose:
            self._pose = pose
            self._start = time.time()
            self._done = False
        elif not self._done and (time.time() - self._start) >= self.required:
            self._done = True

    def confirmed(self): return self._done
    def active_pose(self): return self._pose

    def progress(self):
        if self._start is None:
            return 0.0
        if self._done:
            return 1.0
        return min((time.time() - self._start) / self.required, 1.0)


# ──────────────────────────────────────────────────────────────────────────────
# State machine
# ──────────────────────────────────────────────────────────────────────────────

class StateMachine:
    def __init__(self):
        self.state = NO_HAND

    def update(self, hand_present, hold_timer):
        if not hand_present:
            self.state = NO_HAND
            return self.state
        if hold_timer.confirmed():
            p = hold_timer.active_pose()
            if p == 'down':
                self.state = CAP_PLACED
            elif p == 'horizontal':
                self.state = SIDE_GRIP
        return self.state


# ──────────────────────────────────────────────────────────────────────────────
# Skeleton drawing
# ──────────────────────────────────────────────────────────────────────────────

CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12), (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20), (5, 9), (9, 13), (13, 17),
]

POSE_COLOR = {
    'down':       (0, 180, 255),   # blue-ish
    'horizontal': (0, 210,  80),   # green
    None:         (80,  80,  80),  # gray
}


def draw_skeleton(frame, landmarks, w, h, highlight_color):
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for a, b in CONNECTIONS:
        cv2.line(frame, pts[a], pts[b], (70, 70, 70), 1, cv2.LINE_AA)
    for chain in [(5, 6, 7, 8), (9, 10, 11, 12)]:
        for i in range(len(chain)-1):
            cv2.line(frame, pts[chain[i]], pts[chain[i+1]],
                     highlight_color, 3, cv2.LINE_AA)
    for i, p in enumerate(pts):
        r = 7 if i in (INDEX_TIP, MIDDLE_TIP, INDEX_MCP, MIDDLE_MCP) else 4
        c = highlight_color if r == 7 else (70, 70, 70)
        cv2.circle(frame, p, r, (255, 255, 255), -1)
        cv2.circle(frame, p, r, c, 2)
    for mcp_i, tip_i in [(INDEX_MCP, INDEX_TIP), (MIDDLE_MCP, MIDDLE_TIP)]:
        cv2.arrowedLine(frame, pts[mcp_i], pts[tip_i],
                        highlight_color, 2, tipLength=0.35, line_type=cv2.LINE_AA)


# ──────────────────────────────────────────────────────────────────────────────
# HUD
# ──────────────────────────────────────────────────────────────────────────────

STATE_COLOR = {NO_HAND: (120, 120, 120), CAP_PLACED: (
    0, 180, 255), SIDE_GRIP: (0, 210, 80)}
STATE_LABEL = {
    NO_HAND:    'No hand detected',
    CAP_PLACED: 'Cap placed  -> robot screwing cap',
    SIDE_GRIP:  'Side grip ready  ->  robot releasing gripper',
}
STATE_DESC = {
    NO_HAND:    'Point index+middle DOWN (1s)  or  HORIZONTAL (1s)',
    CAP_PLACED: 'Sent to FIWARE',
    SIDE_GRIP:  'Sent to FIWARE',
}


def draw_hud(frame, state, pose, hold_progress, fps,
             wrist_pos, ang_iv, ang_ih, ang_mv, ang_mh, speed,
             fiware_ok, fiware_enabled, entity_id, broker_url):
    fh, fw = frame.shape[:2]
    sc = STATE_COLOR.get(state, (200, 200, 200))
    pc = POSE_COLOR.get(pose, (80, 80, 80))

    # Top banner
    bg = frame.copy()
    cv2.rectangle(bg, (0, 0), (fw, 105), (15, 15, 15), -1)
    cv2.addWeighted(bg, 0.62, frame, 0.38, 0, frame)

    cv2.putText(frame, STATE_LABEL.get(state, state),
                (16, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.72, sc, 2, cv2.LINE_AA)
    cv2.putText(frame, STATE_DESC.get(state, ''),
                (16, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(frame, f'FPS: {fps:.0f}',
                (fw-105, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (130, 130, 130), 1, cv2.LINE_AA)
    cv2.circle(frame, (fw-28, 52), 14, sc, -1)
    cv2.circle(frame, (fw-28, 52), 14, (255, 255, 255), 1)

    # FIWARE / IoTA badge
    if fiware_enabled:
        badge_col = (0, 180, 80) if fiware_ok else (0, 60, 200)
        badge_txt = f'FIWARE OK' if fiware_ok else 'FIWARE ERR'
        cv2.putText(frame, badge_txt,
                    (fw-260, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.40, badge_col, 1, cv2.LINE_AA)

    # IoT Agent URL watermark
    cv2.putText(frame, broker_url,
                (16, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (80, 80, 80), 1, cv2.LINE_AA)

    # Hold-progress bar
    bx, by, bw_, bh = 16, 112, fw-32, 20
    cv2.rectangle(frame, (bx, by), (bx+bw_, by+bh), (40, 40, 40), -1)
    if pose is not None:
        cv2.rectangle(frame, (bx, by),
                      (bx+int(bw_*hold_progress), by+bh), pc, -1)
        elapsed = hold_progress * CONFIRM_SECONDS
        txt = 'CONFIRMED' if hold_progress >= 1.0 else f'{elapsed:.1f} / {CONFIRM_SECONDS:.0f} s'
        cv2.putText(frame, txt,
                    (bx+8, by+14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, f'Pose: {pose if pose else "none"}',
                (bx+bw_-180, by+14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, pc, 1, cv2.LINE_AA)

    # Angle readouts
    ox, oy = 16, fh-130

    def aline(txt, val, thr, y):
        ok = val <= thr
        cv2.putText(frame, f'{txt}: {val:5.1f} deg  [{"OK" if ok else "--"}]',
                    (ox, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    (0, 200, 80) if ok else (80, 80, 200), 1, cv2.LINE_AA)

    cv2.putText(frame, 'Index:', (ox, oy), cv2.FONT_HERSHEY_SIMPLEX,
                0.40, (160, 160, 160), 1, cv2.LINE_AA)
    aline('  vert ', ang_iv, DOWN_ANGLE_TOLERANCE,  oy+18)
    aline('  horiz', ang_ih, HORIZ_ANGLE_TOLERANCE, oy+36)
    cv2.putText(frame, 'Middle:', (ox, oy+56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160, 160, 160), 1, cv2.LINE_AA)
    aline('  vert ', ang_mv, DOWN_ANGLE_TOLERANCE,  oy+74)
    aline('  horiz', ang_mh, HORIZ_ANGLE_TOLERANCE, oy+92)

    still = speed < STILL_THRESHOLD_PX
    cv2.putText(frame, f'Wrist speed: {speed:.0f} px/s  [{"STILL" if still else "MOVING"}]',
                (ox, fh-28), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                (0, 200, 80) if still else (80, 80, 200), 1, cv2.LINE_AA)
    cv2.putText(frame, "Press 'q' to quit",
                (ox, fh-10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80, 80, 80), 1, cv2.LINE_AA)

    if wrist_pos is not None:
        cv2.circle(frame, (int(wrist_pos[0]), int(wrist_pos[1])), 12, pc, 2)


# ──────────────────────────────────────────────────────────────────────────────
# Model download
# ──────────────────────────────────────────────────────────────────────────────

def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print(f"Downloading hand landmark model (~9 MB) → '{MODEL_PATH}' ...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Done.\n")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    ensure_model()

    publisher = FiwarePublisher(
        broker_url=ARGS.broker,
        fiware_service=ARGS.fiware_service,
        fiware_servicepath=ARGS.fiware_servicepath,
        enabled=not ARGS.no_fiware,
    )

    options = mp_vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.60,
        min_hand_presence_confidence=0.55,
        min_tracking_confidence=0.55,
    )

    cap = cv2.VideoCapture(ARGS.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open webcam (index {ARGS.camera}).")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

    vel = VelocityTracker(window=8)
    timer = HoldTimer(CONFIRM_SECONDS)
    sm = StateMachine()

    prev_time = time.time()
    prev_state = None

    ang_iv = ang_ih = ang_mv = ang_mh = 90.0
    wrist_pos = None
    speed = 0.0
    pose = None

    print("Bottle Capping Gesture Detector")
    print(
        f"  Broker:    {ARGS.broker}  ({'disabled' if ARGS.no_fiware else 'enabled'})")
    print(f"  Service:   {ARGS.fiware_service}  {ARGS.fiware_servicepath}")
    print(f"  Entity:    {ENTITY_ID}")
    print(f"  Hold time: {CONFIRM_SECONDS:.0f} s")
    print(
        f"  GUI:       {'disabled (headless)' if ARGS.no_gui else 'enabled'}")
    if not ARGS.no_fiware:
        if publisher.check_connection():
            print("  Broker status: reachable")
        else:
            print("  Broker status: NOT reachable — detections will be skipped")
            print("                 (start Orion with:  docker compose up -d)")
    print()
    print(f"STATE:{NO_HAND}")

    if not ARGS.no_gui:
        cv2.namedWindow('Bottle Capping Gesture Detector', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Bottle Capping Gesture Detector', 800, 600)

    with mp_vision.HandLandmarker.create_from_options(options) as detector:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, 1)
            fh, fw = frame.shape[:2]
            now = time.time()

            result = detector.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB,
                         data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)),
                int(now * 1000),
            )

            hand_present = bool(result.hand_landmarks)

            if hand_present:
                lms = result.hand_landmarks[0]
                wrist_pos = lm_xy(lms, WRIST, fw, fh)
                vel.update(wrist_pos, now)
                speed = vel.speed()

                ang_iv, ang_ih = finger_direction_angle(
                    lm_xy(lms, INDEX_MCP, fw, fh), lm_xy(lms, INDEX_TIP, fw, fh))
                ang_mv, ang_mh = finger_direction_angle(
                    lm_xy(lms, MIDDLE_MCP, fw, fh), lm_xy(lms, MIDDLE_TIP, fw, fh))

                pose = get_raw_pose(lms, fw, fh)
                timer.update(pose, vel.is_still())
                if not ARGS.no_gui:
                    draw_skeleton(frame, lms, fw, fh,
                                  POSE_COLOR.get(pose, (80, 80, 80)))
            else:
                wrist_pos = None
                speed = 0.0
                pose = None
                ang_iv = ang_ih = ang_mv = ang_mh = 90.0
                timer.update(None, False)

            state = sm.update(hand_present, timer)

            # Publish to FIWARE only when a confirmed state is reached
            if state != prev_state:
                print(f"STATE:{state}", flush=True)
                if state in (NO_HAND, CAP_PLACED, SIDE_GRIP):
                    publisher.publish(state)
                prev_state = state

            fps = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now

            if not ARGS.no_gui:
                draw_hud(frame, state, pose, timer.progress(), fps,
                         wrist_pos, ang_iv, ang_ih, ang_mv, ang_mh, speed,
                         publisher.last_ok, not ARGS.no_fiware,
                         ENTITY_ID, ARGS.broker)
                cv2.imshow('Bottle Capping Gesture Detector', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Quitting.")
                    break

    publisher.close()
    cap.release()
    if not ARGS.no_gui:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
