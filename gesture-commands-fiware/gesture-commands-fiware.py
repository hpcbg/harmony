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

Backends (GESTURE_BACKEND environment variable):
    GESTURE_BACKEND=fiware_v2  (default)     → FIWARE Orion over NGSI-v2 (/v2/entities)
    GESTURE_BACKEND=ros2       (experimental)→ ROS 2 topic /user_inputs/gesture_command
                                               (std_msgs/String); Orion-LD's -wip dds
                                               bridge maps it to NGSI-LD. No /v2 calls.
    TEST_GESTURE="SIDE_GRIP"   → no-camera self-test: publish the gesture(s) through
                                 the selected backend and exit (no OpenCV/MediaPipe).

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

# Heavy perception deps (camera/landmarking) and the HTTP client are imported
# softly: the DDS-native ros2 backend and the TEST_GESTURE self-test publish
# gestures without a camera, OpenCV, MediaPipe or requests, and may run under a
# sourced ROS 2 / Vulcanexus python that lacks them. They are only required for
# live webcam detection (cv2/mediapipe/numpy) or the fiware_v2 backend (requests).
try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None
    _HAS_NUMPY = False

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    cv2 = None
    _HAS_CV2 = False

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    _HAS_MEDIAPIPE = True
except ImportError:
    mp = mp_python = mp_vision = None
    _HAS_MEDIAPIPE = False

try:
    import requests
    _HAS_REQUESTS = True
except ImportError:
    requests = None
    _HAS_REQUESTS = False

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
parser.add_argument('--camera',             type=int, default=None,
                    help='Camera device index. Overrides config/config.json; '
                         'falls back to that file\'s CAMERA, else 0.')
ARGS = parser.parse_args()


# ── Camera source: --camera (CLI) > config/config.json CAMERA > 0 ──────────────
# Mirrors the AI detector's config/config.json convention (a tracked
# config/config.json.tpl template; the real config/config.json is gitignored and
# machine-local). Missing file or key is never fatal — we fall back to 0.
def _load_camera_config():
    import json
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "config", "config.json")
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except Exception as exc:                       # malformed JSON, etc.
        print(f"[CONFIG] Ignoring {path}: {exc}")
        return {}


if ARGS.camera is None:
    ARGS.camera = int(_load_camera_config().get("CAMERA", 0))
    print(f"[CONFIG] camera device {ARGS.camera} "
          "(from config/config.json; override with --camera)")

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
# ROS 2 publisher  (DDS-native experimental backend)  -  GESTURE_BACKEND=ros2
# ──────────────────────────────────────────────────────────────────────────────
# Instead of writing NGSI-v2 entities over HTTP, publish each confirmed gesture
# (NO_HAND / CAP_PLACED / SIDE_GRIP) as a std_msgs/String on the ROS 2 topic
# /user_inputs/gesture_command. When Orion-LD runs with `-wip dds`, its built-in
# DDS bridge discovers that topic (DDS name rt/user_inputs/gesture_command) and
# maps it to the NGSI-LD entity urn:ngsi-ld:GestureDetector:operator-1, attribute
# `command`. So in this mode the gesture module never touches /v2/entities and
# never triggers the HTTP 501 the -mongocOnly DDS broker returns for NGSI-v2.
#
# Mirrors FiwarePublisher's interface (publish/check_connection/last_ok/close) so
# the detection loop stays backend-agnostic.

GESTURE_TOPIC = '/user_inputs/gesture_command'


class Ros2GesturePublisher:
    """Publish recognised gestures to a ROS 2 topic (DDS-native path)."""

    def __init__(self, topic=GESTURE_TOPIC):
        try:
            import rclpy
            from std_msgs.msg import String
        except ImportError:
            print("[GESTURE] ERROR: ROS 2 (rclpy / std_msgs) not found.")
            print("          The 'ros2' gesture backend needs a sourced ROS 2 / "
                  "Vulcanexus workspace, e.g.:")
            print("            source /opt/vulcanexus/jazzy/setup.bash")
            print("            source <ws>/install/setup.bash")
            raise SystemExit(1)

        self._rclpy = rclpy
        self._String = String
        self.topic = topic
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node('gesture_command_publisher')
        self._pub = self._node.create_publisher(String, topic, 10)
        self._last_ok = True
        print("[GESTURE] ROS 2 backend active")

    def publish(self, state):
        # Gesture values are published unchanged (NO_HAND / CAP_PLACED / SIDE_GRIP).
        msg = self._String()
        msg.data = state
        self._pub.publish(msg)
        self._last_ok = True
        print(f"[GESTURE] published {self.topic}: {state}")
        return True

    def wait_for_subscribers(self, timeout=5.0):
        """Spin until at least one subscriber matches (or timeout).

        DDS delivery is best-effort, so a one-shot publish fired before the
        subscriber (the DDS broker, or `ros2 topic echo`) has finished discovery
        is silently dropped. The live camera loop never hits this — it streams
        for minutes — but the TEST_GESTURE self-test does, so wait for a match.
        """
        deadline = time.time() + timeout
        count = self._pub.get_subscription_count()
        while count == 0 and time.time() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.1)
            count = self._pub.get_subscription_count()
        return count

    def check_connection(self):
        # rclpy is initialised and the publisher created; nothing to ping.
        return True

    @property
    def last_ok(self):
        return self._last_ok

    def close(self):
        """Cleanly tear down the node and rclpy (idempotent)."""
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if self._rclpy.ok():
            self._rclpy.shutdown()


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
    # ── select backend ──────────────────────────────────────────────────────
    # GESTURE_BACKEND=fiware_v2 (default) → publish NGSI-v2 entities to Orion
    #                                       (the integrated node-bridge demo).
    # GESTURE_BACKEND=ros2                → publish std_msgs/String to ROS 2 topic
    #                                       /user_inputs/gesture_command; Orion-LD's
    #                                       -wip dds bridge maps it to NGSI-LD.
    backend = os.environ.get('GESTURE_BACKEND', 'fiware_v2').strip().lower()
    if backend not in ('fiware_v2', 'ros2'):
        print(f"ERROR: unknown GESTURE_BACKEND='{backend}'. "
              "Use 'fiware_v2' (default) or 'ros2'.")
        raise SystemExit(1)

    # GESTURE_MODE=real (default) → open the configured camera and run MediaPipe.
    # GESTURE_MODE=stub           → no camera / no MediaPipe; publish an initial
    #                               NO_HAND and idle (analogous to the AI
    #                               detector's AI_DETECTION_MODE=stub). TEST_GESTURE
    #                               still works in either mode.
    gesture_mode = os.environ.get('GESTURE_MODE', 'real').strip().lower()
    if gesture_mode not in ('real', 'stub'):
        print(f"ERROR: unknown GESTURE_MODE='{gesture_mode}'. "
              "Use 'real' (default) or 'stub'.")
        raise SystemExit(1)

    # ── set up publisher ────────────────────────────────────────────────────
    if backend == 'ros2':
        publisher = Ros2GesturePublisher()
        print(f"Backend:   ros2 (DDS-native → {GESTURE_TOPIC})")
        print("           Orion-LD -wip dds maps this to "
              "urn:ngsi-ld:GestureDetector:operator-1.command")
    else:
        if not ARGS.no_fiware and not _HAS_REQUESTS:
            print("ERROR: 'requests' is required for the fiware_v2 backend.")
            print("       Install it with:  pip install requests")
            raise SystemExit(1)
        publisher = FiwarePublisher(
            broker_url=ARGS.broker,
            fiware_service=ARGS.fiware_service,
            fiware_servicepath=ARGS.fiware_servicepath,
            enabled=not ARGS.no_fiware,
        )

    # ── headless self-test (no camera) ──────────────────────────────────────
    # TEST_GESTURE pushes gesture(s) through the selected backend without a
    # camera, OpenCV or MediaPipe, e.g.:
    #     GESTURE_BACKEND=ros2 TEST_GESTURE=SIDE_GRIP ./run.sh
    #     GESTURE_BACKEND=ros2 TEST_GESTURE="NO_HAND SIDE_GRIP" ./run.sh
    test_gesture = os.environ.get('TEST_GESTURE')
    if test_gesture:
        states = [s for s in test_gesture.replace(',', ' ').split() if s]
        valid = {NO_HAND, CAP_PLACED, SIDE_GRIP}
        unknown = [s for s in states if s not in valid]
        if unknown:
            print(f"[TEST] WARNING: unrecognised gesture(s) {unknown}; valid: "
                  f"{sorted(valid)}. Publishing as given.")
        print(f"[TEST] no-camera mode — publishing {len(states)} gesture(s): "
              f"{' '.join(states)}")
        try:
            if isinstance(publisher, Ros2GesturePublisher):
                n = publisher.wait_for_subscribers()
                if n == 0:
                    print("[TEST] WARNING: no DDS subscriber matched within timeout "
                          "— is the DDS broker (or 'ros2 topic echo') running on the "
                          "same ROS_DOMAIN_ID? Publishing anyway.")
                else:
                    print(f"[TEST] {n} DDS subscriber(s) matched.")
            for s in states:
                publisher.publish(s)
                time.sleep(0.5)
            # give DDS discovery / NGSI write a moment to settle before teardown
            time.sleep(1.0)
        finally:
            publisher.close()
        return

    # ── stub mode (no camera, no MediaPipe) ─────────────────────────────────
    # GESTURE_MODE=stub keeps the backend running without opening any camera:
    # publish an initial NO_HAND and idle so the DDS/FIWARE side still sees a
    # live gesture detector. (TEST_GESTURE above already ran and returned if set.)
    if gesture_mode == 'stub':
        print("[STUB] GESTURE_MODE=stub — no camera opened; idling in NO_HAND.")
        print("       (set TEST_GESTURE=... to publish gestures without a camera.)")
        try:
            if isinstance(publisher, Ros2GesturePublisher):
                n = publisher.wait_for_subscribers()
                print(f"[STUB] {n} DDS subscriber(s) matched." if n else
                      "[STUB] no DDS subscriber matched yet — publishing anyway.")
            publisher.publish(NO_HAND)
            print(f"STATE:{NO_HAND}", flush=True)
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nStopping stub gesture detector.")
        finally:
            publisher.close()
        return

    # ── live webcam detection ───────────────────────────────────────────────
    if not (_HAS_CV2 and _HAS_MEDIAPIPE and _HAS_NUMPY):
        missing = [n for n, ok in (('opencv-python', _HAS_CV2),
                                   ('mediapipe', _HAS_MEDIAPIPE),
                                   ('numpy', _HAS_NUMPY)) if not ok]
        print(f"ERROR: live gesture detection needs: {', '.join(missing)}.")
        print("       Install them with:  pip install opencv-python mediapipe numpy")
        print("       (or use TEST_GESTURE=... for a no-camera self-test)")
        publisher.close()
        raise SystemExit(1)

    ensure_model()

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
        # Real mode uses only the configured camera — no automatic fallback to
        # another index. Report the error so a wrong camera ID is caught, not
        # silently masked by opening camera 0.
        cap.release()
        raise RuntimeError(
            f"Cannot open camera index {ARGS.camera}. In real mode the configured "
            f"camera is used as-is (no automatic fallback). Fix the camera ID in "
            f"config/config.json (or pass --camera), or run with GESTURE_MODE=stub.")
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
    if backend == 'ros2':
        print(f"  Backend:   ros2 (DDS-native → {GESTURE_TOPIC})")
        print(f"  Entity:    {ENTITY_ID}  (mapped by Orion-LD -wip dds)")
    else:
        print(
            f"  Broker:    {ARGS.broker}  ({'disabled' if ARGS.no_fiware else 'enabled'})")
        print(f"  Service:   {ARGS.fiware_service}  {ARGS.fiware_servicepath}")
        print(f"  Entity:    {ENTITY_ID}")
    print(f"  Hold time: {CONFIRM_SECONDS:.0f} s")
    print(
        f"  GUI:       {'disabled (headless)' if ARGS.no_gui else 'enabled'}")
    if backend == 'fiware_v2' and not ARGS.no_fiware:
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
