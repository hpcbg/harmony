#!/usr/bin/env python3
"""Pack Bottle Setup Assistant — DDS-native variant.

Counterpart of setup.py for the ARISE **DDS-broker-only** path. Same assistant,
but the Daily Startup wires the NGSI-LD / DDS runtime instead of NGSI-v2:

  • FIWARE side  : the Orion-LD DDS broker (docker-compose.dds.yml, -wip dds
                   -mongocOnly, NGSI-LD only) — NOT the fiware-analytics stack.
  • Voice/Gesture: their DDS-native `ros2` backends (VOICE_BACKEND=ros2 /
                   GESTURE_BACKEND=ros2).
  • AI detector  : AI_BACKEND=ros2 AI_DETECTION_MODE=real (real Faster-R-CNN
                   perception behind the four DDS bottle_detection topics).
  • ROS 2        : BRIDGE_BACKEND=dds (the default) — Orion-LD bridges directly.

The React dashboard is deliberately NOT offered here: it reads NGSI-v2, which the
-mongocOnly DDS broker returns HTTP 501 for. Everything else has been validated
end-to-end on this path (see run_dds_regression_tests.sh).

Requires a Vulcanexus Jazzy ROS side and Docker. The DDS broker and the standard
Orion stack both bind host port 1026 — do not run both at once.

First-time installation reuses setup.py's install flow; pick the `dds` bridge
backend (the default) when prompted.
"""

import sys

# Reuse the shared assistant machinery from setup.py (same directory).
import setup as base
from setup import (
    colorize, BOLD, CYAN, DIM, GREEN, YELLOW,
    step, ok, warn, info, ask, ask_yes_no, ask_choice,
    detect_terminal, load_state, save_state, SCRIPT_DIR,
)

# Source Vulcanexus (ARISE-compliant) first, then plain Jazzy, plus the project
# workspace, so the venv python running gesture/voice can import rclpy from the
# sourced PYTHONPATH (the single-interpreter approach the AI torch_venv uses).
# NB: braces are doubled ({{ }}) so they survive str.format() below and become a
# real bash group `{ ... }` at launch time (matching launch_pack_bottle_dds.sh).
ROS_SRC = (
    "{{ [ -f /opt/vulcanexus/jazzy/setup.bash ] "
    "&& source /opt/vulcanexus/jazzy/setup.bash "
    "|| source /opt/ros/jazzy/setup.bash; "
    "source ros2-xarm-pack-bottle/ros2_ws/install/setup.bash 2>/dev/null || true; }}"
)

# DDS-native services, in launch order. Same cmd_tpl placeholders as setup.py:
# {root} = SCRIPT_DIR, {camera} = gesture camera device ID. The React dashboard
# is intentionally absent (NGSI-v2 only).
DDS_STARTUP_SERVICES = [
    {
        "key":     "fiware",   # keep key "fiware" so the docker readiness check applies
        "label":   "FIWARE DDS Broker (Orion-LD, NGSI-LD)",
        "checks":  [],
        "cmd_tpl": ("cd {root}/ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds "
                    "&& docker compose -f docker-compose.dds.yml up -d "
                    "&& echo 'DDS Orion-LD broker up on :1026 (NGSI-LD only).'"),
    },
    {
        "key":     "ai",
        "label":   "AI Bottle Detector (DDS-native, real perception)",
        "checks":  [
            "torch_venv/bin/activate",
            "ai-bottle-detector-fiware/config/config.json",
            "ai-bottle-detector-fiware/models/best_model.pth",
        ],
        "cmd_tpl": ("sleep 5 && cd {root}/ai-bottle-detector-fiware "
                    "&& AI_BACKEND=ros2 AI_DETECTION_MODE=real ./run.sh"),
    },
    {
        "key":     "gesture",
        "label":   "Hand Gesture Recognition (DDS-native)",
        "checks":  ["venv/bin/activate"],
        "cmd_tpl": ("sleep 8 && cd {root} && source venv/bin/activate && " + ROS_SRC +
                    " && cd gesture-commands-fiware "
                    "&& GESTURE_BACKEND=ros2 python gesture-commands-fiware.py "
                    "--camera {camera} --no-gui"),
    },
    {
        "key":     "voice",
        "label":   "Voice Commands (DDS-native)",
        "checks":  ["venv/bin/activate"],
        "cmd_tpl": ("sleep 8 && cd {root} && source venv/bin/activate && " + ROS_SRC +
                    " && cd voice-commands-fiware "
                    "&& VOICE_BACKEND=ros2 python voice-commands-fiware.py --keywords"),
    },
    {
        "key":     "ros2",
        "label":   "ROS 2 + xArm Control (DDS bridge)",
        "checks":  [
            "ros2-xarm-pack-bottle/ros2_ws/config/xarm_pack_bottle.json",
            "ros2-xarm-pack-bottle/ros2_ws/install",
        ],
        "cmd_tpl": ("sleep 20 && cd {root}/ros2-xarm-pack-bottle "
                    "&& BRIDGE_BACKEND=dds ./run.sh"),
    },
]


def mode_startup_dds():
    step("Daily Startup — DDS-native (NGSI-LD)")
    state = load_state()

    # ── Readiness table ───────────────────────────────────────────────────────
    print(f"\n  {colorize('Service readiness:', BOLD)}\n")
    docker_ok = base._docker_available()
    ready_keys = set()

    for svc in DDS_STARTUP_SERVICES:
        key, label = svc["key"], svc["label"]
        issues = base._service_issues(svc)
        if key == "fiware":
            if docker_ok:
                ok(label); ready_keys.add(key)
            else:
                base.err(f"{label}  — Docker / Docker Compose not available")
        elif issues:
            warn(f"{label}  — not configured ({', '.join(issues)})")
        else:
            ok(label); ready_keys.add(key)

    # ── Service selection ─────────────────────────────────────────────────────
    step("Select Services to Start")
    print("  Services that are not yet configured are deselected by default.")
    print(f"  {colorize('The React dashboard is not part of the DDS path (NGSI-v2 only).', DIM)}\n")
    selected = set(ready_keys)

    while True:
        print()
        for i, svc in enumerate(DDS_STARTUP_SERVICES):
            key = svc["key"]
            mark = colorize("[x]", GREEN) if key in selected else colorize("[ ]", DIM)
            note = "" if key in ready_keys else colorize("  (not configured)", YELLOW)
            print(f"   {mark} {i + 1}) {svc['label']}{note}")
        val = input(colorize(
            f"\n  Toggle 1-{len(DDS_STARTUP_SERVICES)} · A=all-ready · N=none · Enter=launch: ",
            DIM)).strip().lower()
        if not val:
            break
        if val == "a":
            selected = set(ready_keys)
        elif val == "n":
            selected = set()
        else:
            try:
                idx = int(val) - 1
                if 0 <= idx < len(DDS_STARTUP_SERVICES):
                    selected ^= {DDS_STARTUP_SERVICES[idx]["key"]}
            except ValueError:
                pass

    if not selected:
        warn("Nothing selected — exiting.")
        return

    # ── Camera ID (gesture only) ───────────────────────────────────────────────
    camera = state.get("camera", 0)
    if "gesture" in selected:
        print()
        try:
            camera = int(ask("Camera device ID for gesture recognition", default=str(camera)))
        except ValueError:
            camera = 0

    # ── Terminal detection ─────────────────────────────────────────────────────
    terminal = detect_terminal()
    print()
    if terminal == "none":
        warn("No supported terminal emulator found (gnome-terminal, konsole, xterm, tmux).")
        warn("Commands will be printed for you to run manually.")
    else:
        info(f"Terminal: {terminal}")

    # ── Pre-flight: warn if a non-LD Orion holds :1026 ─────────────────────────
    if "fiware" in selected:
        import subprocess
        ver = subprocess.run(["curl", "-s", "--max-time", "3",
                              "http://localhost:1026/version"],
                             capture_output=True, text=True).stdout.lower()
        if "orion" in ver and "orionld" not in ver:
            warn("A non-LD Orion (NGSI-v2) is already on :1026 — stop it first; "
                 "the DDS Orion-LD broker needs that port.")

    # ── Confirm and launch ─────────────────────────────────────────────────────
    labels = [s["label"] for s in DDS_STARTUP_SERVICES if s["key"] in selected]
    print(f"\n  {colorize('Will start:', BOLD)} {', '.join(labels)}\n")
    if not ask_yes_no("Launch now?", default=True):
        info("Aborted.")
        return

    root = str(SCRIPT_DIR)
    for svc in DDS_STARTUP_SERVICES:
        if svc["key"] not in selected:
            continue
        cmd = svc["cmd_tpl"].format(root=root, camera=camera)
        if terminal == "none":
            print(f"\n  {colorize(svc['label'], BOLD)}:\n    {cmd}")
        else:
            info(f"Launching {svc['label']}...")
            base._launch_in_terminal(terminal, svc["label"], cmd)

    state["camera"] = camera
    save_state(state)

    if terminal != "none":
        print()
        ok("All selected DDS-native services launched in separate terminal windows/tabs.")
        info("DDS broker starts immediately; AI after 5 s; Gesture/Voice after 8 s; ROS 2 after 20 s.")
    else:
        print(f"\n  {colorize('Copy the commands above and run each in its own terminal.', YELLOW)}")


def main():
    print(f"""
{colorize('═' * 60, BOLD, CYAN)}
{colorize('  Pack Bottle  —  Setup Assistant (DDS-native / NGSI-LD)', BOLD, CYAN)}
{colorize('  Orion-LD DDS broker · ros2 backends · no dashboard', DIM)}
{colorize('═' * 60, BOLD, CYAN)}
""")
    mode = ask_choice(
        "What would you like to do?",
        [
            "Daily Startup (DDS)  — check readiness and launch the DDS-native runtime",
            "First-time Install   — install dependencies and generate config files",
        ],
        default=0,
    )
    print()
    if mode == 0:
        mode_startup_dds()
    else:
        info("Reusing the shared installer — choose the 'dds' bridge backend (the default) "
             "when prompted for the FIWARE bridge backend.")
        print()
        base.mode_install()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n\n{colorize('  Cancelled.', YELLOW)}\n")
        sys.exit(0)
