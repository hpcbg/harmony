import cv2
import numpy as np

# RTSP stream URL
RTSP_URL = "rtsp://192.168.1.113:8554/mjpeg/1"

# ArUco markers
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Plane definition markers
CORNER_MARKERS = [8, 2, 1, 7]

# 2D plane points
plane_points = np.array([
    [0, 0],
    [150, 0],
    [120, 150],
    [0, 150]
], dtype=np.float32)

H = None  # homography

clicked_point = None


def mouse_callback(event, x, y, flags, param):
    global H, clicked_point, clicked_global
    if event == cv2.EVENT_LBUTTONDOWN and H is not None:
        pix = np.array([[x, y, 1]], dtype=np.float32).T
        pt = H @ pix
        pt /= pt[2]
        print("Plane coordinates:", (pt[0][0], pt[1][0]))
        clicked_point = (x, y)
        clicked_global = (pt[0][0], pt[1][0])


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


def main():
    global H, clicked_global, clicked_point

    cap = cv2.VideoCapture(RTSP_URL)

    if not cap.isOpened():
        print("Cannot open RTSP stream:", RTSP_URL)
        return

    cv2.namedWindow("RTSP")
    cv2.setMouseCallback("RTSP", mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame not captured.")
            continue

        corners, ids = detect_all_markers(frame)

        if ids is not None:
            for corner, id_ in zip(corners, ids.flatten()):
                pts = corner[0].astype(int)
                cv2.polylines(frame, [pts], True, (0, 255, 255), 2)
                center = tuple(np.mean(pts, axis=0).astype(int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID {id_}", (pts[0][0], pts[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 150, 255), 2)

        # calculate homography
        plane_markers = get_plane_markers(corners, ids)
        if plane_markers is not None:
            H, _ = cv2.findHomography(plane_markers, plane_points)

            pts = np.float32(plane_points).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, np.linalg.inv(H))
            dst = np.int32(dst)

            cv2.polylines(frame, [dst], True, (0, 255, 0), 2)

        if clicked_point is not None:
            print(clicked_point, clicked_point)
            cv2.circle(frame, clicked_point, 5, (0, 0, 255), -1)
            cv2.putText(frame,
                        f"({clicked_global[0]:.1f}, {clicked_global[1]:.1f})",
                        (clicked_point[0] + 10, clicked_point[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2)

        cv2.imshow("RTSP", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
