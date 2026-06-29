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

Self-test (no camera / GPU / PyTorch / model weights / FIWARE NGSI-v2):
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

    def __init__(self):
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

    # ── subscribing ─────────────────────────────────────────────────────────
    def _on_command(self, msg):
        cmd = (msg.data or "").strip().upper()
        print(f"[AI] received {COMMAND_TOPIC}: {cmd}")
        if cmd == "START":
            self.run_detection_stub()
        # Other commands are ignored in this first step.

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
        """Cleanly tear down the node and rclpy (idempotent)."""
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main():
    backend = Ros2DetectorBackend()
    test_cmd = os.environ.get("TEST_DETECTION_COMMAND")
    try:
        if test_cmd:
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
