#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# scripts/measure_dds_fiware_latency.py
#
# Measures the DDS-native FIWARE *control-loop communication latency*: how long a
# control signal takes to travel ROS 2 → Orion-LD/FIWARE → back to ROS 2.
#
#     ROS 2 publisher  (/user_inputs/gesture_command, std_msgs/String)
#       → Orion-LD DDS enabler maps it to NGSI-LD
#           urn:ngsi-ld:GestureDetector:operator-1  command.value.data
#       → this script PATCHes the *output* entity attribute (the minimal TEST
#         RELAY — this is measurement overhead, NOT perception/robot logic)
#           urn:ngsi-ld:BottleDetectionJob:processor-01  command.value.data
#       → Orion-LD DDS enabler writes it back to ROS 2
#           /bottle_detection/command (std_msgs/String)
#       → ROS 2 subscriber
#
# The primary metric is the closed ROS → FIWARE → ROS loop (t3 - t0). It excludes
# camera processing, AI inference, gesture logic, robot motion, and dashboard
# rendering: the only work between the two DDS hops is the minimal test relay.
#
# Per-sample timestamps (time.monotonic_ns):
#   t0   publish to input ROS topic
#   t1   input NGSI-LD attribute observed == payload   (ROS → FIWARE = t1 - t0)
#   t2   relay PATCH of output attribute initiated
#   t2b  relay PATCH HTTP response received             (relay/PATCH  = t2b - t2)
#   t3   payload received on output ROS topic           (FIWARE → ROS = t3 - t2)
#                                                        (loop         = t3 - t0)
#
# NOTE: the relay PATCH and the FIWARE → ROS notification overlap. Orion-LD writes
# the update to DDS *while still processing the PATCH*, so the ROS subscriber can
# receive the message before the PATCH HTTP response returns (t3 < t2b). The
# FIWARE → ROS interval is therefore measured from PATCH *initiation* (t2): it
# captures Orion's push + DDS delivery and never goes negative. The relay/PATCH
# overhead (t2b - t2) is the concurrent client-side HTTP cost of the test relay —
# reported as a diagnostic, not perception or robot logic. The loop total is
# measured directly (t3 - t0) and does not depend on the breakdown.
#
# stdlib + rclpy + std_msgs only (no external HTTP deps).
# ─────────────────────────────────────────────────────────────────────────────

import argparse
import csv
import json
import math
import os
import statistics
import sys
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
except Exception as exc:  # pragma: no cover - environment guard
    sys.stderr.write(
        "ERROR: could not import rclpy/std_msgs (%s).\n"
        "       Source a ROS 2 / Vulcanexus environment first, e.g.\n"
        "       source /opt/vulcanexus/jazzy/setup.bash\n" % exc
    )
    sys.exit(2)


# ── configuration (env-overridable; the shell wrapper forwards these) ─────────
def env_int(name, default):
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


ORION = os.environ.get("ORION", "http://localhost:1026").rstrip("/")
# Output directory for the JSON/CSV artifacts. Defaults to /tmp (as documented);
# override with DDS_LATENCY_OUTDIR — e.g. to a mounted path when running inside
# the Vulcanexus Docker wrapper, where the container's own /tmp is ephemeral.
OUTDIR = os.environ.get("DDS_LATENCY_OUTDIR", "/tmp")
WARMUP = env_int("DDS_LATENCY_WARMUP", 3)
SAMPLES = env_int("DDS_LATENCY_SAMPLES", 20)
TIMEOUT_SEC = env_int("DDS_LATENCY_TIMEOUT_SEC", 5)
POLL_INTERVAL_MS = env_int("DDS_LATENCY_POLL_INTERVAL_MS", 5)

INPUT_TOPIC = "/user_inputs/gesture_command"
OUTPUT_TOPIC = "/bottle_detection/command"
INPUT_ENTITY = "urn:ngsi-ld:GestureDetector:operator-1"
INPUT_TYPE = "GestureDetector"
OUTPUT_ENTITY = "urn:ngsi-ld:BottleDetectionJob:processor-01"
OUTPUT_TYPE = "BottleDetectionJob"
ATTR = "command"


# ── tiny ANSI helpers ─────────────────────────────────────────────────────────
_TTY = sys.stdout.isatty()
BOLD = "\033[1m" if _TTY else ""
CYN = "\033[36m" if _TTY else ""
GRN = "\033[32m" if _TTY else ""
YEL = "\033[33m" if _TTY else ""
RED = "\033[31m" if _TTY else ""
RST = "\033[0m" if _TTY else ""


def hr():
    print("─" * 60)


def head(msg):
    print("\n%s%s%s" % (BOLD + CYN, msg, RST))
    hr()


# ── Orion-LD NGSI-LD helpers (stdlib urllib) ──────────────────────────────────
def _request(method, url, body=None, timeout=5):
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, resp.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()
    except Exception:
        return None, None


def broker_is_orionld():
    status, body = _request("GET", "%s/version" % ORION, timeout=3)
    if status is None or body is None:
        return None  # unreachable
    return b"orionld" in body.lower()


def read_input_attr():
    """Return the current command.value.data of the input entity, or None."""
    url = "%s/ngsi-ld/v1/entities/%s?local=true&attrs=%s" % (ORION, INPUT_ENTITY, ATTR)
    status, body = _request("GET", url, timeout=3)
    if status != 200 or not body:
        return None
    try:
        d = json.loads(body.decode("utf-8"))
    except Exception:
        return None
    a = d.get(ATTR)
    if not isinstance(a, dict):
        return None
    v = a.get("value")
    if isinstance(v, dict):
        return v.get("data")
    return v


def ensure_entity(entity_id, entity_type):
    """Idempotently make sure the entity + command attribute exist (for PATCH/poll)."""
    url = "%s/ngsi-ld/v1/entities/%s?local=true" % (ORION, entity_id)
    status, _ = _request("GET", url, timeout=3)
    if status == 200:
        return True
    body = {
        "id": entity_id,
        "type": entity_type,
        ATTR: {"type": "Property", "value": {"data": "init"}},
    }
    status, _ = _request("POST", "%s/ngsi-ld/v1/entities" % ORION, body=body, timeout=5)
    return status in (201, 204, 409)


def patch_output_attr(payload):
    """Relay the payload onto the output entity attribute (DDS nested String shape)."""
    url = "%s/ngsi-ld/v1/entities/%s/attrs/%s" % (ORION, OUTPUT_ENTITY, ATTR)
    body = {"type": "Property", "value": {"data": payload}}
    status, _ = _request("PATCH", url, body=body, timeout=TIMEOUT_SEC)
    return status in (204, 207)


# ── ROS 2 node: publish to input, subscribe to output ─────────────────────────
class LoopProbe(Node):
    def __init__(self):
        super().__init__("dds_fiware_latency_probe")
        self._pub = self.create_publisher(String, INPUT_TOPIC, 10)
        self._sub = self.create_subscription(
            String, OUTPUT_TOPIC, self._on_output, 10
        )
        self._lock = threading.Lock()
        self._expected = None
        self._received_ns = None
        self._event = threading.Event()

    def _on_output(self, msg):
        now = time.monotonic_ns()
        with self._lock:
            if self._expected is not None and msg.data == self._expected:
                self._received_ns = now
                self._event.set()

    def arm(self, payload):
        with self._lock:
            self._expected = payload
            self._received_ns = None
            self._event.clear()

    def publish_input(self, payload):
        self._pub.publish(String(data=payload))

    def wait_output(self, timeout):
        got = self._event.wait(timeout)
        with self._lock:
            return self._received_ns if got else None

    def input_subscription_count(self):
        return self._pub.get_subscription_count()

    def output_publisher_count(self):
        return self._sub.get_publisher_count()


def wait_for_discovery(node, timeout):
    """Wait until the Orion-LD DDS enabler is matched on both sides (best-effort)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if node.input_subscription_count() >= 1 and node.output_publisher_count() >= 1:
            return True
        time.sleep(0.1)
    return False


# ── one sample ────────────────────────────────────────────────────────────────
def run_sample(node, index):
    """Run one loop; return a dict with ns timings and ok flag."""
    payload = "PERF_LOOP_%d_%d" % (time.monotonic_ns(), index)
    poll_s = POLL_INTERVAL_MS / 1000.0

    node.arm(payload)
    t0 = time.monotonic_ns()
    node.publish_input(payload)

    # ROS → FIWARE: poll the input attribute until it reflects our payload
    t1 = None
    deadline = time.monotonic() + TIMEOUT_SEC
    while time.monotonic() < deadline:
        if read_input_attr() == payload:
            t1 = time.monotonic_ns()
            break
        time.sleep(poll_s)
    if t1 is None:
        return {"index": index, "ok": False, "error": "input attr not observed (ROS→FIWARE timeout)"}

    # relay: PATCH the output attribute with the same payload (the test relay).
    # t2 marks relay initiation; Orion pushes to DDS during PATCH processing, so the
    # FIWARE → ROS interval is referenced from t2 (never negative), while t2b - t2
    # is the concurrent client-side HTTP PATCH cost.
    t2 = time.monotonic_ns()
    if not patch_output_attr(payload):
        return {"index": index, "ok": False, "error": "output PATCH rejected"}
    t2b = time.monotonic_ns()

    # FIWARE → ROS: wait for the payload on the output ROS topic
    t3 = node.wait_output(TIMEOUT_SEC)
    if t3 is None:
        return {"index": index, "ok": False, "error": "output not received (FIWARE→ROS timeout)"}

    return {
        "index": index,
        "ok": True,
        "error": None,
        "loop_ns": t3 - t0,
        "ros_to_fiware_ns": t1 - t0,
        "patch_overhead_ns": t2b - t2,
        "fiware_to_ros_ns": t3 - t2,
    }


# ── statistics ────────────────────────────────────────────────────────────────
def percentile(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_vals[int(k)]
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


def summarise_ms(values_ns):
    vals = sorted(v / 1e6 for v in values_ns)
    return {
        "samples": len(vals),
        "min": vals[0],
        "median": statistics.median(vals),
        "mean": statistics.fmean(vals),
        "p95": percentile(vals, 95),
        "max": vals[-1],
    }


def fmt_ms(x):
    return "%.3f ms" % x


# ── reporting ─────────────────────────────────────────────────────────────────
def print_block(title, s, full=True):
    print("  %s:" % title)
    print("    samples: %d" % s["samples"])
    if full:
        print("    min:    %s" % fmt_ms(s["min"]))
    print("    median: %s" % fmt_ms(s["median"]))
    if full:
        print("    mean:   %s" % fmt_ms(s["mean"]))
    print("    p95:    %s" % fmt_ms(s["p95"]))
    if full:
        print("    max:    %s" % fmt_ms(s["max"]))


def write_outputs(stamp, config, samples, summary):
    ok = [s for s in samples if s["ok"]]
    try:
        os.makedirs(OUTDIR, exist_ok=True)
    except Exception:
        pass
    json_path = os.path.join(OUTDIR, "harmony-dds-latency-%s.json" % stamp)
    csv_path = os.path.join(OUTDIR, "harmony-dds-latency-%s.csv" % stamp)
    with open(json_path, "w") as f:
        json.dump(
            {"timestamp": stamp, "config": config, "summary": summary, "samples": samples},
            f,
            indent=2,
        )
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["index", "ok", "loop_ms", "ros_to_fiware_ms", "patch_overhead_ms", "fiware_to_ros_ms", "error"]
        )
        for s in samples:
            if s["ok"]:
                w.writerow([
                    s["index"], 1,
                    "%.3f" % (s["loop_ns"] / 1e6),
                    "%.3f" % (s["ros_to_fiware_ns"] / 1e6),
                    "%.3f" % (s["patch_overhead_ns"] / 1e6),
                    "%.3f" % (s["fiware_to_ros_ns"] / 1e6),
                    "",
                ])
            else:
                w.writerow([s["index"], 0, "", "", "", "", s["error"]])
    return json_path, csv_path, len(ok)


def main():
    argparse.ArgumentParser(
        description="Measure DDS-native ROS→FIWARE→ROS control-loop latency."
    ).parse_args()

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    config = {
        "orion": ORION,
        "warmup": WARMUP,
        "samples": SAMPLES,
        "timeout_sec": TIMEOUT_SEC,
        "poll_interval_ms": POLL_INTERVAL_MS,
        "input_topic": INPUT_TOPIC,
        "output_topic": OUTPUT_TOPIC,
        "input_entity": INPUT_ENTITY,
        "output_entity": OUTPUT_ENTITY,
        "attribute": "%s.value.data" % ATTR,
    }

    head("DDS/FIWARE control-loop latency measurement")
    print("    Orion-LD        : %s" % ORION)
    print("    input  (ROS→FIWARE): %s → %s.%s" % (INPUT_TOPIC, INPUT_ENTITY, ATTR))
    print("    output (FIWARE→ROS): %s.%s → %s" % (OUTPUT_ENTITY, ATTR, OUTPUT_TOPIC))
    print("    warmup=%d samples=%d timeout=%ds poll=%dms"
          % (WARMUP, SAMPLES, TIMEOUT_SEC, POLL_INTERVAL_MS))

    # ── refuse the wrong broker ───────────────────────────────────────────────
    is_ld = broker_is_orionld()
    if is_ld is None:
        print("  %s✗%s Orion-LD not reachable at %s" % (RED, RST, ORION))
        return 2
    if not is_ld:
        print("  %s✗%s Broker at %s is not Orion-LD (NGSI-v2-only stack?)." % (RED, RST, ORION))
        print("      This measurement requires the DDS/NGSI-LD Orion-LD broker.")
        return 2
    print("  %s✓%s Orion-LD (DDS/NGSI-LD) broker confirmed" % (GRN, RST))

    # ── make sure both entities/attributes exist for PATCH + poll ─────────────
    ensure_entity(INPUT_ENTITY, INPUT_TYPE)
    ensure_entity(OUTPUT_ENTITY, OUTPUT_TYPE)

    rclpy.init()
    node = LoopProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except Exception:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    rc = 0
    try:
        if wait_for_discovery(node, TIMEOUT_SEC):
            print("  %s✓%s DDS enabler matched on input and output topics" % (GRN, RST))
        else:
            print("  %s!%s DDS enabler not fully matched yet (in=%d out=%d); warmup will prime it"
                  % (YEL, RST, node.input_subscription_count(), node.output_publisher_count()))

        head("Warmup (%d, not recorded)" % WARMUP)
        for i in range(WARMUP):
            r = run_sample(node, -(i + 1))
            state = "ok" if r["ok"] else ("FAILED: " + r["error"])
            print("    warmup %d/%d: %s" % (i + 1, WARMUP, state))

        head("Sampling (%d)" % SAMPLES)
        samples = []
        for i in range(SAMPLES):
            r = run_sample(node, i)
            samples.append(r)
            if r["ok"]:
                print("    sample %2d/%d: loop %s  (R→F %s | relay %s | F→R %s)"
                      % (i + 1, SAMPLES,
                         fmt_ms(r["loop_ns"] / 1e6),
                         fmt_ms(r["ros_to_fiware_ns"] / 1e6),
                         fmt_ms(r["patch_overhead_ns"] / 1e6),
                         fmt_ms(r["fiware_to_ros_ns"] / 1e6)))
            else:
                print("    sample %2d/%d: %sFAILED%s — %s" % (i + 1, SAMPLES, RED, RST, r["error"]))

        ok = [s for s in samples if s["ok"]]
        if not ok:
            print("\n  %s✗%s No successful loop samples — broken DDS mapping or broker not in DDS mode."
                  % (RED, RST))
            summary = {}
            json_path, csv_path, _ = write_outputs(stamp, config, samples, summary)
            print("    wrote %s" % json_path)
            print("    wrote %s" % csv_path)
            return 1

        summary = {
            "loop": summarise_ms([s["loop_ns"] for s in ok]),
            "ros_to_fiware": summarise_ms([s["ros_to_fiware_ns"] for s in ok]),
            "patch_overhead": summarise_ms([s["patch_overhead_ns"] for s in ok]),
            "fiware_to_ros": summarise_ms([s["fiware_to_ros_ns"] for s in ok]),
        }

        head("DDS/FIWARE control-loop communication latency")
        print("\nROS → FIWARE → ROS loop:")
        s = summary["loop"]
        print("  samples: %d" % s["samples"])
        print("  min:    %s" % fmt_ms(s["min"]))
        print("  median: %s" % fmt_ms(s["median"]))
        print("  mean:   %s" % fmt_ms(s["mean"]))
        print("  p95:    %s" % fmt_ms(s["p95"]))
        print("  max:    %s" % fmt_ms(s["max"]))
        print("\nBreakdown:")
        print_block("ROS → FIWARE", summary["ros_to_fiware"], full=False)
        print_block("FIWARE relay/PATCH overhead (test relay, not perception/robot logic)",
                    summary["patch_overhead"], full=False)
        print_block("FIWARE → ROS", summary["fiware_to_ros"], full=False)

        failed = len(samples) - len(ok)
        if failed:
            print("\n  %s!%s %d/%d samples failed (communication timeouts) — see files."
                  % (YEL, RST, failed, len(samples)))

        json_path, csv_path, n_ok = write_outputs(stamp, config, samples, summary)
        print("\nWrote:")
        print("  %s" % json_path)
        print("  %s" % csv_path)
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    return rc


if __name__ == "__main__":
    sys.exit(main())
