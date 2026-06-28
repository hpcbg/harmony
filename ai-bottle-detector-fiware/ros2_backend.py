#!/usr/bin/env python3
"""DDS-native ROS 2 backend for the bottle detector — experimental first step.

This is the AI_BACKEND=ros2 entry point. It is intentionally minimal and
self-contained: it speaks ROS 2 / DDS only and does NOT import FastAPI, OpenCV,
PyTorch, the detection pipeline, the model weights, or the FIWARE NGSI-v2 client
(fiware.py / main.py). The full NGSI-v2 detector (`uvicorn main:app`) is left
completely unchanged and remains the default backend.

Flow (first step):
    subscribe  /bottle_detection/command      (std_msgs/String)  ← accepts "START"
    publish    /bottle_detection/status_json   (std_msgs/String)  → simple status

When Orion-LD runs with `-wip dds` and the mapping is present (rt/bottle_detection/
status_json -> urn:ngsi-ld:BottleDetectionJob:processor-01.status), the status is
mapped to NGSI-LD. No /v2/entities calls happen here, so the -mongocOnly DDS broker
never returns HTTP 501 to this module.

NOT migrated in this first step (kept on the NGSI-v2 backend): the full result JSON,
processed-image URLs, base64 payloads and the pick pose. Only a simple detector
status string is published.

Self-test (no camera / GPU / PyTorch / model weights / FIWARE NGSI-v2):
    AI_BACKEND=ros2 TEST_DETECTION_COMMAND=START ./run.sh
"""
import json
import os
import time

try:
    import rclpy
    from std_msgs.msg import String
except ImportError:
    print("[AI] ERROR: ROS 2 (rclpy / std_msgs) not found.")
    print("     The 'ros2' AI backend needs a sourced ROS 2 / Vulcanexus workspace, e.g.:")
    print("       source /opt/vulcanexus/jazzy/setup.bash")
    print("       source <ws>/install/setup.bash")
    raise SystemExit(1)

COMMAND_TOPIC = "/bottle_detection/command"
STATUS_TOPIC = "/bottle_detection/status_json"


class Ros2DetectorBackend:
    """Minimal ROS 2 node: receive detector commands, publish detector status."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("bottle_detector_ros2")
        self._status_pub = self._node.create_publisher(String, STATUS_TOPIC, 10)
        self._sub = self._node.create_subscription(
            String, COMMAND_TOPIC, self._on_command, 10)
        print("[AI] ROS 2 backend active")
        print(f"[AI] subscribed {COMMAND_TOPIC}  (accepts START)")
        print(f"[AI] publishing  {STATUS_TOPIC}")

    # ── publishing ──────────────────────────────────────────────────────────
    def _publish_status(self, **fields):
        msg = String()
        msg.data = json.dumps(fields)
        self._status_pub.publish(msg)
        print(f"[AI] published {STATUS_TOPIC}: {msg.data}")

    def run_detection_stub(self):
        """First DDS-native step: no real perception yet.

        Emit a simple status sequence so downstream consumers can observe the
        detector lifecycle over DDS. Real frame capture / inference / pick-pose
        stay on the NGSI-v2 backend until a later migration step.
        """
        self._publish_status(status="PROCESSING")
        time.sleep(0.5)
        self._publish_status(status="DONE", bottleCount=0)

    # ── subscribing ─────────────────────────────────────────────────────────
    def _on_command(self, msg):
        cmd = (msg.data or "").strip().upper()
        print(f"[AI] received {COMMAND_TOPIC}: {cmd}")
        if cmd == "START":
            self.run_detection_stub()
        # Other commands are ignored in this first step.

    # ── helpers ─────────────────────────────────────────────────────────────
    def wait_for_subscribers(self, timeout=5.0):
        """Spin until at least one subscriber matches the status topic (or timeout).

        DDS delivery is best-effort, so a one-shot publish fired before the
        subscriber (the DDS broker, or `ros2 topic echo`) has finished discovery
        is silently dropped. The long-running spin mode never hits this; the
        TEST_DETECTION_COMMAND self-test does, so wait for a match first.
        """
        deadline = time.time() + timeout
        count = self._status_pub.get_subscription_count()
        while count == 0 and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            count = self._status_pub.get_subscription_count()
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
