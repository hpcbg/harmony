import cv2
import numpy as np
from ultralytics import YOLO
import json
import os
import math

import utils.json_config

CONFIG = utils.json_config.load("config/detect_obb.json")

scale_factor = CONFIG['WINDOW_SCALE_FACTOR']
coords_diff = (CONFIG['WORKAREA_POSE']['x'], CONFIG['WORKAREA_POSE']['y'])
orientation_diff = CONFIG['WORKAREA_POSE']['orientation_degrees']

RTSP_URL = CONFIG['RTSP_URL']
MODEL_PATH = CONFIG['MODEL_PATH']

model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(RTSP_URL)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# ArUco markers and correspondig points for plane definition
CORNER_MARKERS = CONFIG['CORNER_MARKERS']
PLANE_POINTS = np.array(CONFIG['CORNER_COORDINATES'], dtype=np.float32)

BOTTLE_STAND_POSE = CONFIG['BOTTLE_STAND_POSE']

HOVER_COLOR = (0, 255, 255)   # yellow
NORMAL_BOTTLE = (0, 255, 0)
NORMAL_CAP = (0, 0, 255)
SELECTED_COLOR = (255, 0, 255)
hover_obj = None
selected_obj = None
mouse_x, mouse_y = 0, 0
coord_mode = "GLOBAL"
H = None


def normalize_angle(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi


def local_to_global_pose(x_l, y_l, theta_l, x0, y0, theta0):
    x_g = x0 + x_l * math.cos(theta0) - y_l * math.sin(theta0)
    y_g = y0 + x_l * math.sin(theta0) + y_l * math.cos(theta0)

    theta_g = normalize_angle(theta0 + theta_l)

    return x_g, y_g, theta_g


def detect_all_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    return corners, ids


def get_plane_markers(corners, ids):
    if ids is None:
        return None

    ids = ids.flatten()
    pts = {}

    for corner, id_ in zip(corners, ids):
        if id_ in CORNER_MARKERS:
            c = corner[0]
            center = np.mean(c, axis=0)
            pts[id_] = center

    if len(pts) != 4:
        return None

    ordered = np.array([pts[i] for i in CORNER_MARKERS], dtype=np.float32)
    return ordered


def toggle_coord_mode():
    global coord_mode
    coord_mode = "LOCAL" if coord_mode == "GLOBAL" else "GLOBAL"


def get_robot_status():
    path = "robot_status.json"
    if not os.path.exists(path):
        return {"status": "UNKNOWN", "pos": None}

    try:
        with open(path, "r") as f:
            data = json.load(f)
        return data
    except:
        return {"status": "ERROR", "pos": None}


def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y, selected_obj, bottles, caps

    x = int(x / scale_factor)
    y = int(y / scale_factor)
    mouse_x, mouse_y = x, y

    if event == cv2.EVENT_LBUTTONDOWN:

        if 10 <= y <= 10 + 280:  # UI Panel
            # Coordinates mode button
            bx, by, bw, bh = 10+10, 10+240, 150, 35
            if bx <= x <= bx + bw and by <= y <= by + bh:
                toggle_coord_mode()
                return

        robot_state = get_robot_status()
        robot_status = robot_state.get("status", "UNKNOWN")

        # Check the robot
        if robot_status != "OK":
            print("ERROR: Robot is busy or not available!")
            selected_obj = None
            return

        selected_obj = None

        for b in bottles:
            if b["mask"][y, x] > 0:
                selected_obj = b
                break

        if selected_obj is None:
            return

        # Write pick and place command if the robot is available
        x0, y0 = coords_diff[0], coords_diff[1]
        theta0 = math.radians(orientation_diff)
        x_l, y_l = float(selected_obj['coords'][0]), float(
            selected_obj['coords'][1])
        theta_l = math.radians(float(selected_obj.get('orientation', 0)))
        x, y, theta = local_to_global_pose(x_l, y_l, theta_l, x0, y0, theta0)
        theta = math.degrees(theta)
        data = {
            "pick_pose": {
                "x": x,
                "y": y,
                "z": 0,
                "roll_degrees": 0,
                "pitch_degrees": 180,
                "yaw_degrees": theta
            },
            "place_pose": {
                "x": BOTTLE_STAND_POSE['x'],
                "y": BOTTLE_STAND_POSE['y'],
                "z": BOTTLE_STAND_POSE['z'],
                "roll_degrees": 0,
                "pitch_degrees": 90,
                "yaw_degrees": 0,
            }
        }

        with open("pick_and_place.json", "w") as f:
            json.dump(data, f, indent=4)

        print("Created pick_and_place.json:", data)


def draw_gui_overlay(frame, hover_obj, selected_obj, robot_state):
    h, w = frame.shape[:2]

    panel_w = 350
    panel_h = 280
    x0, y0 = 10, 10
    x1, y1 = x0 + panel_w, y0 + panel_h

    # UI Panel
    panel = frame.copy()
    cv2.rectangle(panel, (x0, y0), (x1, y1), (40, 40, 40), -1)
    frame[:] = cv2.addWeighted(panel, 0.4, frame, 0.6, 0)

    cv2.putText(frame, "BOTTLE DETECTOR", (x0 + 10, y0 + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 220, 255), 2)

    y = y0 + 60

    # Hover
    if hover_obj:
        cv2.putText(frame, f"Hover conf: {hover_obj['conf']:.2f}",
                    (x0 + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 0), 2)
        y += 28

    # Selected
    if selected_obj:
        cv2.putText(frame, f"Local:",
                    (x0 + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 100, 255), 2)
        y += 28

        cv2.putText(frame, f"XY: {selected_obj['coords'][0]:.1f}, {selected_obj['coords'][1]:.1f}",
                    (x0 + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (255, 150, 255), 2)
        y += 28

        cv2.putText(frame, f"Orientation: {selected_obj.get('orientation', 0):.1f} deg",
                    (x0 + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (255, 150, 255), 2)
        y += 28

    # Robot status
    y += 10
    cv2.putText(frame, "Robot",
                (x0 + 10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 200), 2)

    y += 35

    status = robot_state.get("status", "UNKNOWN")
    pos = robot_state.get("pos")

    # Status
    color = (0, 255, 0) if status == "IDLE" else (0, 150, 255)
    if status == "BUSY":
        color = (0, 165, 255)
    if status == "ERROR":
        color = (0, 0, 255)

    cv2.putText(frame, f"Status: {status}",
                (x0 + 10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
    y += 30

    # Robot Position
    if pos:
        cv2.putText(frame,
                    f"Robot XYZ: {pos['x']:.1f}, {pos['y']:.1f}, {pos['z']:.1f}",
                    (x0 + 10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    else:
        cv2.putText(frame,
                    "Robot XYZ: no data",
                    (x0 + 10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (150, 150, 150), 2)

    # BUSY warning overlay
    if status != "OK":
        warn = frame.copy()
        cv2.rectangle(warn, (400, 0), (w-400, 70), (0, 165, 255), -1)
        frame = cv2.addWeighted(warn, 0.7, frame, 0.3, 0)
        cv2.putText(frame, "ROBOT IS BUSY OR NOT AVAILABLE",
                    (540, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 3)

    # Coords mode switch button
    btn_w = 150
    btn_h = 35
    bx = x0 + 10
    by = y0 + 240
    cv2.rectangle(frame, (bx, by), (bx + btn_w, by + btn_h),
                  (180, 180, 180), 2)
    cv2.putText(frame,
                f"MODE: {coord_mode}",
                (bx + 10, by + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 2)

    return frame


cv2.namedWindow("Bottle Detector")
cv2.setMouseCallback("Bottle Detector", mouse_callback)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.8, agnostic_nms=True, iou=0.3)

    annotated = results[0].plot()

    # cv2.imshow("OBB Detection", annotated)

    corners, ids = detect_all_markers(frame)

    # Show the ArUco markers
    if ids is not None:
        for corner, id_ in zip(corners, ids.flatten()):
            pts = corner[0].astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 255), 2)
            center = tuple(np.mean(pts, axis=0).astype(int))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"{id_}", (pts[0][0], pts[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 150, 255), 2)
    # compute homography
    plane_markers = get_plane_markers(corners, ids)
    if plane_markers is not None:
        H, _ = cv2.findHomography(plane_markers, PLANE_POINTS)

        pts = np.float32(PLANE_POINTS).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, np.linalg.inv(H))
        dst = np.int32(dst)

        cv2.polylines(frame, [dst], True, (0, 255, 0), 2)

    r = results[0]

    bottles = []
    caps = []

    img_h, img_w = r.orig_shape

    # Separate objects by classes
    for i in range(len(r.obb)):
        box_obj = r.obb[i]
        cls_name = results[0].names[int(box_obj.cls[0])]

        x, y, w, h, angle = box_obj.xywhr[0].cpu().numpy()  # normalized
        pts = box_obj.xyxyxyxy[0].cpu().numpy().astype(np.int32)
        mask = np.zeros((img_h, img_w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)

        try:
            pix = np.array([[x, y, 1]], dtype=np.float32).T
            pt = H @ pix
            pt /= pt[2]
            coords = (pt[0][0], pt[1][0])
        except:
            coords = (1, 1)

        obj = {
            "center": (int(x), int(y)),
            "angle": angle,
            "pts": pts,
            "mask": mask,
            "conf": r.obb[i].conf[0].item(),
            "coords": coords
        }

        if cls_name == 'bottle':
            bottles.append(obj)
        elif cls_name == 'cap':
            caps.append(obj)

    # Vizualisation
    for i in range(len(bottles)):
        b = bottles[i]
        b['cap'] = None
        best_coverage = 0.5
        for c in caps:
            count = cv2.countNonZero(cv2.bitwise_and(b['mask'], c['mask']))
            coverage = count / cv2.countNonZero(c['mask'])

            if coverage > best_coverage:
                b['cap'] = c['center']
                b['cap_coords'] = c['coords']

        b['orientation'] = 0
        if b['cap'] is not None:
            bottle_center = np.array(b['coords'])
            cap_center = np.array(b['cap_coords'])
            vec = cap_center - bottle_center
            angle = np.degrees(np.arctan2(vec[1], vec[0])) + 90
            orientation = angle % 360 - 180
            b['orientation'] = orientation

    for b in bottles:
        if b["mask"][mouse_y, mouse_x] > 0:
            hover_obj = b
            break

    if hover_obj is not None:
        overlay = frame.copy()
        mask = hover_obj["mask"]
        overlay[mask > 0] = (255, 50, 255)
        frame = cv2.addWeighted(overlay, 0.35, frame, 0.65, 0)

    for b in bottles:
        if b['cap'] is None:
            continue

        color = NORMAL_BOTTLE

        if hover_obj is b:
            color = HOVER_COLOR

        if selected_obj is b:
            color = SELECTED_COLOR

        cv2.drawContours(frame, [b["pts"]], -1, color, 3)
        cv2.line(frame, b["center"], b["cap"], color=(0, 255, 0), thickness=2)
        cv2.circle(frame, b["center"], 5, (0, 255, 0), -1)
        coords = b['coords']
        orientation = b['orientation']
        if coord_mode == 'GLOBAL':
            x0, y0 = coords_diff[0], coords_diff[1]
            theta0 = math.radians(orientation_diff)
            x_l, y_l = coords[0], coords[1]
            theta_l = math.radians(orientation)
            x, y, theta = local_to_global_pose(
                x_l, y_l, theta_l, x0, y0, theta0)
            theta = math.degrees(theta)
        else:
            x, y, theta = coords[0], coords[1], orientation

        cv2.putText(frame, f"({x:.1f}, {y:.1f})",
                    (b["center"][0] + 5, b["center"][1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"{int(theta)} deg",
                    (b["center"][0] + 5, b["center"][1] + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Bottle {b['conf']:.2f}",
                    (b["center"][0] + 5, b["center"][1] + 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    for c in caps:
        cv2.drawContours(frame, [c["pts"]], -1, (0, 0, 255), 2)
        cv2.circle(frame, c["center"], 5, (0, 0, 255), -1)

    hover_obj = None
    robot_state = get_robot_status()
    frame = draw_gui_overlay(frame, hover_obj, selected_obj, robot_state)

    cv2.imshow("Bottle Detector", cv2.resize(
        frame, None, fx=scale_factor, fy=scale_factor))

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()

if os.path.exists("pick_and_place.json"):
    try:
        os.remove("pick_and_place.json")
    except:
        pass
