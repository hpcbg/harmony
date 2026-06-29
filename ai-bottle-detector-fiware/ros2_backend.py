#!/usr/bin/env python3
"""DDS-native ROS 2 backend for the bottle detector — experimental first step.

This is the AI_BACKEND=ros2 entry point. It is intentionally minimal and
self-contained: it speaks ROS 2 / DDS only and does NOT import FastAPI, OpenCV,
PyTorch, the detection pipeline, the model weights, or the FIWARE NGSI-v2 client
(fiware.py / main.py). The full NGSI-v2 detector (`uvicorn main:app`) is left
completely unchanged and remains the default backend.

Flow (incremental decomposition):
    subscribe  /bottle_detection/command       (std_msgs/String)  ← accepts "START"
    publish    /bottle_detection/status_json    (std_msgs/String)  → simple status
    publish    /bottle_detection/bottle_count   (std_msgs/Int32)   → bottle count
    publish    /bottle_detection/pick_pose_json (std_msgs/String)  → pick pose (JSON)
    publish    /bottle_detection/result_json    (std_msgs/String)  → full result (JSON)

When Orion-LD runs with `-wip dds` and the mappings are present (each rt/bottle_detection/*
topic -> an attribute of urn:ngsi-ld:BottleDetectionJob:processor-01), the outputs are
mapped to NGSI-LD. No /v2/entities calls happen here, so the -mongocOnly DDS broker
never returns HTTP 501 to this module.

The detector result is decomposed into DDS-native outputs one topic at a time so the
refactor stays controlled and never breaks the existing NGSI-v2 dashboard demo. Still
NOT migrated (kept on the NGSI-v2 backend): processed-image URLs and base64 payloads.
The pose here is published as a single JSON string (pick_pose_json); the NGSI-v2 path's
decomposed pickX/pickY/pickRotation floats are unchanged and stay on that backend.

Detection mode (`AI_DETECTION_MODE`), behind the SAME DDS interface:
    stub *(default)*  — no camera/GPU/PyTorch; emits representative values.
    real              — captures a frame and runs the existing detector pipeline
                        (camera.py + pipeline.py: Faster R-CNN + ArUco homography),
                        then publishes the real result on the same four topics.

Only `run_detection_stub()` is replaced by the real perception call; the topics,
message types and Orion-LD mappings are unchanged. Real mode needs torch/torchvision/
cv2 and the model weights, so launch it through run.sh (which uses the detector venv
python, able to import both rclpy and torch in one interpreter):
    AI_BACKEND=ros2 AI_DETECTION_MODE=real ./run.sh

Self-test (always the stub / no-camera path, even when AI_DETECTION_MODE=real):
    AI_BACKEND=ros2 TEST_DETECTION_COMMAND=START ./run.sh
"""
import json
import os
import time

try:
    import rclpy
    from std_msgs.msg import Int32, String
except ImportError:
    print("[AI] ERROR: ROS 2 (rclpy / std_msgs) not found.")
    print("     The 'ros2' AI backend needs a sourced ROS 2 / Vulcanexus workspace, e.g.:")
    print("       source /opt/vulcanexus/jazzy/setup.bash")
    print("       source <ws>/install/setup.bash")
    raise SystemExit(1)

COMMAND_TOPIC = "/bottle_detection/command"
STATUS_TOPIC = "/bottle_detection/status_json"
COUNT_TOPIC = "/bottle_detection/bottle_count"
PICK_POSE_TOPIC = "/bottle_detection/pick_pose_json"
RESULT_TOPIC = "/bottle_detection/result_json"


class Ros2DetectorBackend:
    """Minimal ROS 2 node: receive detector commands, publish detector status."""

    def __init__(self, detection_mode="stub"):
        self.detection_mode = "real" if detection_mode == "real" else "stub"
        self._camera = None          # lazily opened in real mode
        self._process_frame = None   # lazily imported pipeline fn (real mode)
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("bottle_detector_ros2")
        self._status_pub = self._node.create_publisher(String, STATUS_TOPIC, 10)
        self._count_pub = self._node.create_publisher(Int32, COUNT_TOPIC, 10)
        self._pick_pose_pub = self._node.create_publisher(String, PICK_POSE_TOPIC, 10)
        self._result_pub = self._node.create_publisher(String, RESULT_TOPIC, 10)
        self._pubs = [self._status_pub, self._count_pub,
                      self._pick_pose_pub, self._result_pub]
        self._sub = self._node.create_subscription(
            String, COMMAND_TOPIC, self._on_command, 10)
        print("[AI] ROS 2 backend active")
        print(f"[AI] detection mode: {self.detection_mode}"
              f"{' (camera + pipeline)' if self.detection_mode == 'real' else ' (no camera)'}")
        print(f"[AI] subscribed {COMMAND_TOPIC}  (accepts START)")
        for t in (STATUS_TOPIC, COUNT_TOPIC, PICK_POSE_TOPIC, RESULT_TOPIC):
            print(f"[AI] publishing  {t}")

    # ── publishing ──────────────────────────────────────────────────────────
    def _publish_status(self, **fields):
        msg = String()
        msg.data = json.dumps(fields)
        self._status_pub.publish(msg)
        print(f"[AI] published {STATUS_TOPIC}: {msg.data}")

    def _publish_count(self, count):
        msg = Int32()
        msg.data = int(count)
        self._count_pub.publish(msg)
        print(f"[AI] published {COUNT_TOPIC}: {msg.data}")

    def _publish_pick_pose(self, pose):
        msg = String()
        msg.data = json.dumps(pose)
        self._pick_pose_pub.publish(msg)
        print(f"[AI] published {PICK_POSE_TOPIC}: {msg.data}")

    def _publish_result(self, result):
        msg = String()
        msg.data = json.dumps(result)
        self._result_pub.publish(msg)
        print(f"[AI] published {RESULT_TOPIC}: {msg.data}")

    def run_detection_stub(self):
        """DDS-native decomposition step: no real perception yet.

        Emit the detector lifecycle (status) plus the decomposed result outputs
        (bottle count, pick pose, full result JSON) so each DDS-native AI output
        can be validated end-to-end through Orion-LD before any real perception is
        migrated. Real frame capture / inference / image URLs stay on the NGSI-v2
        backend until a later migration step.
        """
        self._publish_status(status="PROCESSING")
        time.sleep(0.5)

        bottle_count = 1
        pick_pose = {"x": 120.0, "y": -45.0, "rotation": 30.0}
        result = {
            "status": "DONE",
            "bottleCount": bottle_count,
            "pickPose": pick_pose,
        }
        self._publish_count(bottle_count)
        self._publish_pick_pose(pick_pose)
        self._publish_result(result)
        self._publish_status(status="DONE", bottleCount=bottle_count)

    def _ensure_pipeline(self):
        """Lazily import the real detector pipeline and open the camera.

        The heavy deps (torch/torchvision/cv2) and the model weights are loaded
        only the first time real detection runs, so the stub/test paths never need
        them. `pipeline.process_frame` loads the Faster R-CNN model at import.
        """
        if self._process_frame is None:
            try:
                from pipeline import process_frame
            except Exception as e:
                raise RuntimeError(
                    "real detection needs the detector pipeline (torch, torchvision, "
                    "cv2) and the model weights. Launch via run.sh with "
                    "AI_DETECTION_MODE=real so the detector venv python is used "
                    f"(import error: {e})")
            self._process_frame = process_frame
        if self._camera is None:
            import utils.json_config
            from camera import Camera
            cfg = utils.json_config.load("config/config.json")
            cam_env = os.environ.get("AI_CAMERA", "").strip()
            cam_index = int(cam_env) if cam_env else cfg["CAMERA"]
            print(f"[AI] opening camera {cam_index} …")
            self._camera = Camera(cam_index, cfg.get("SET_RESOLUTION", False),
                                  cfg.get("WIDTH", 1600), cfg.get("HEIGHT", 1200))

    def run_detection_real(self):
        """Real perception behind the same DDS interface.

        Capture one frame and run the existing detector pipeline, then publish the
        real result on the same four topics. Image URLs / base64 stay on the NGSI-v2
        backend; only the scalar outputs are bridged over DDS.
        """
        self._publish_status(status="CAPTURING")
        self._ensure_pipeline()

        frame = self._camera.get_frame()
        deadline = time.time() + 5.0
        while frame is None and time.time() < deadline:
            time.sleep(0.1)
            frame = self._camera.get_frame()
        if frame is None:
            raise RuntimeError("no camera frame available")

        self._publish_status(status="PROCESSING")
        result = self._process_frame(frame.copy())
        bottles = result.get("bottles") or []
        pick_pose = result.get("pick_pose") or {}
        bottle_count = len(bottles)

        self._publish_count(bottle_count)
        self._publish_pick_pose(pick_pose)
        self._publish_result({
            "status": "DONE",
            "bottleCount": bottle_count,
            "pickPose": pick_pose,
            "bottles": bottles,
        })
        self._publish_status(status="DONE", bottleCount=bottle_count)

    def run_detection(self):
        """Dispatch a START to the configured detection mode (stub or real)."""
        if self.detection_mode == "real":
            try:
                self.run_detection_real()
            except Exception as e:
                print(f"[AI] real detection FAILED: {e}")
                self._publish_status(status="FAILED", error=str(e))
        else:
            self.run_detection_stub()

    # ── subscribing ─────────────────────────────────────────────────────────
    def _on_command(self, msg):
        cmd = (msg.data or "").strip().upper()
        print(f"[AI] received {COMMAND_TOPIC}: {cmd}")
        if cmd == "START":
            self.run_detection()
        # Other commands are ignored in this step.

    # ── helpers ─────────────────────────────────────────────────────────────
    def wait_for_subscribers(self, timeout=5.0, settle=0.8):
        """Spin until a subscriber matches at least one output topic (or timeout).

        DDS delivery is best-effort, so a one-shot publish fired before the
        subscriber (the DDS broker, or `ros2 topic echo`) has finished discovery
        is silently dropped. The long-running spin mode never hits this; the
        TEST_DETECTION_COMMAND self-test does, so wait for a match first.

        With multiple output topics the DDS broker discovers them at roughly the
        same time; once any one is matched, a short `settle` lets discovery of the
        remaining topics complete before the one-shot burst is published.
        """
        deadline = time.time() + timeout
        count = max(p.get_subscription_count() for p in self._pubs)
        while count == 0 and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            count = max(p.get_subscription_count() for p in self._pubs)
        if count > 0 and settle > 0:
            settle_deadline = time.time() + settle
            while time.time() < settle_deadline:
                rclpy.spin_once(self._node, timeout_sec=0.1)
        return count

    def spin(self):
        try:
            rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass

    def close(self):
        """Cleanly tear down the camera, node and rclpy (idempotent)."""
        if self._camera is not None:
            try:
                self._camera.release()
            except Exception:
                pass
            self._camera = None
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main():
    mode = os.environ.get("AI_DETECTION_MODE", "stub").strip().lower()
    backend = Ros2DetectorBackend(detection_mode=mode)
    test_cmd = os.environ.get("TEST_DETECTION_COMMAND")
    try:
        if test_cmd:
            # TEST_DETECTION_COMMAND is always the stub / no-camera path, even when
            # AI_DETECTION_MODE=real, so the self-test never needs a camera or torch.
            cmd = test_cmd.strip().upper()
            print(f"[TEST] no-camera mode — injecting command: {cmd}")
            n = backend.wait_for_subscribers()
            if n == 0:
                print("[TEST] WARNING: no DDS subscriber matched within timeout — "
                      "is the DDS broker (or 'ros2 topic echo') running on the same "
                      "ROS_DOMAIN_ID? Publishing anyway.")
            else:
                print(f"[TEST] {n} DDS subscriber(s) matched.")
            if cmd == "START":
                backend.run_detection_stub()
            else:
                print(f"[TEST] command '{cmd}' not recognised "
                      "(only START is handled in this first step).")
            time.sleep(1.0)
        else:
            print("[AI] waiting for commands … (Ctrl-C to stop)")
            backend.spin()
    finally:
        backend.close()


if __name__ == "__main__":
    main()
