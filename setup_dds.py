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

import contextlib
import io
import json
import sys

# Reuse the shared assistant machinery from setup.py (same directory).
import setup as base
from setup import (
    colorize, BOLD, CYAN, DIM, GREEN, YELLOW,
    step, ok, warn, info, ask, ask_yes_no, ask_choice,
    detect_terminal, SCRIPT_DIR,
)

# Source Vulcanexus (ARISE-compliant) first, then plain Jazzy, plus the project
# workspace, so the venv python running gesture/voice can import rclpy from the
# sourced PYTHONPATH (the single-interpreter approach the AI torch_venv uses).
# The venvs are isolated (no system site-packages), so rclpy's pure-Python deps
# (yaml, lark, catkin_pkg) — which live in the system dist-packages — are appended
# to PYTHONPATH; appended (not prepended) so the venv's mediapipe/vosk still win.
# NB: braces are doubled ({{ }}) so they survive str.format() below and become a
# real bash group `{ ... }` at launch time (matching launch_pack_bottle_dds.sh).
ROS_SRC = (
    "{{ [ -f /opt/vulcanexus/jazzy/setup.bash ] "
    "&& source /opt/vulcanexus/jazzy/setup.bash "
    "|| source /opt/ros/jazzy/setup.bash; "
    "source ros2-xarm-pack-bottle/ros2_ws/install/setup.bash 2>/dev/null || true; "
    "export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages; }}"
)

# ── Persistent gesture camera ID ──────────────────────────────────────────────
# The gesture camera lives in gesture-commands-fiware/config/config.json (created
# from config.json.tpl; machine-local and gitignored). This assistant is the
# canonical place to set it; gesture-commands-fiware.py reads the same file at
# launch (a direct `--camera` on that script still overrides it for one run).
GESTURE_CONFIG_DIR = SCRIPT_DIR / "gesture-commands-fiware" / "config"
GESTURE_CONFIG = GESTURE_CONFIG_DIR / "config.json"
GESTURE_CONFIG_TPL = GESTURE_CONFIG_DIR / "config.json.tpl"


def _tpl_camera_default():
    """Gesture camera default from config.json.tpl, else 0."""
    try:
        return int(json.loads(GESTURE_CONFIG_TPL.read_text()).get("CAMERA", 0))
    except Exception:
        return 0


def _write_gesture_camera(camera):
    GESTURE_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    GESTURE_CONFIG.write_text(json.dumps({"CAMERA": int(camera)}, indent=2) + "\n")


def read_gesture_camera():
    """Return (camera_id, status): status is 'ok' | 'missing' | 'malformed'."""
    if not GESTURE_CONFIG.exists():
        return None, "missing"
    try:
        return int(json.loads(GESTURE_CONFIG.read_text())["CAMERA"]), "ok"
    except Exception:
        return None, "malformed"


def _ask_gesture_camera(default):
    try:
        return int(ask("Gesture camera device ID", default=str(default)))
    except (ValueError, TypeError):
        return int(default)


def ensure_gesture_camera(force=False):
    """Resolve the persistent gesture camera ID, prompting only when necessary.

      • missing    → ask (default from config.json.tpl, else 0), create the file
      • malformed  → warn clearly, ask for a replacement, rewrite the file
      • ok         → return silently (no prompt) unless force=True (reconfigure)
    """
    rel = GESTURE_CONFIG.relative_to(SCRIPT_DIR)
    camera, status = read_gesture_camera()

    if status == "ok" and not force:
        return camera
    if status == "malformed":
        warn(f"{rel} is malformed — its camera ID could not be read. Reconfiguring.")

    default = camera if status == "ok" else _tpl_camera_default()
    print()
    camera = _ask_gesture_camera(default)
    _write_gesture_camera(camera)
    ok(f"Gesture camera {camera} saved to {rel}")
    return camera


# DDS-native services, in launch order. cmd_tpl placeholder {root} = SCRIPT_DIR.
# The gesture module reads its camera from config/config.json (see above), so no
# --camera is passed here. The React dashboard is intentionally absent (NGSI-v2).
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
                    "&& GESTURE_BACKEND=ros2 python gesture-commands-fiware.py --no-gui"),
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


# The shared installer's summary hard-codes `./launch_pack_bottle.sh`; on the DDS
# path the one-shot launcher is `./launch_pack_bottle_dds.sh`. Wrap the summary,
# capture its output, and rewrite just that command. (mode_install() looks up
# `print_install_summary` in setup's globals at call time, so patching it on `base`
# is enough — the original is captured here first to avoid recursion.)
_orig_install_summary = base.print_install_summary


def _dds_install_summary(components, cfg):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        _orig_install_summary(components, cfg)
    sys.stdout.write(
        buf.getvalue().replace("./launch_pack_bottle.sh", "./launch_pack_bottle_dds.sh")
    )


def mode_startup_dds():
    step("Daily Startup — DDS-native (NGSI-LD)")

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

    # ── Gesture camera (persisted in config/config.json; prompt only if needed) ─
    camera = None
    if "gesture" in selected:
        camera = ensure_gesture_camera()

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
    print(f"\n  {colorize('Will start:', BOLD)} {', '.join(labels)}")
    if camera is not None:
        print(f"  {colorize('Gesture camera:', BOLD)} device {camera} "
              f"{colorize('(from config/config.json)', DIM)}")
    print()
    if not ask_yes_no("Launch now?", default=True):
        info("Aborted.")
        return

    root = str(SCRIPT_DIR)
    for svc in DDS_STARTUP_SERVICES:
        if svc["key"] not in selected:
            continue
        cmd = svc["cmd_tpl"].format(root=root)
        if terminal == "none":
            print(f"\n  {colorize(svc['label'], BOLD)}:\n    {cmd}")
        else:
            info(f"Launching {svc['label']}...")
            base._launch_in_terminal(terminal, svc["label"], cmd)

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
    cam, cam_status = read_gesture_camera()
    cam_hint = (f"currently device {cam}" if cam_status == "ok"
                else "malformed — needs fixing" if cam_status == "malformed"
                else "not set yet")
    mode = ask_choice(
        "What would you like to do?",
        [
            "Daily Startup (DDS)     — check readiness and launch the DDS-native runtime",
            "First-time Install      — install dependencies and generate config files",
            f"Set gesture camera ID   — reconfigure config/config.json ({cam_hint})",
        ],
        default=0,
    )
    print()
    if mode == 0:
        mode_startup_dds()
    elif mode == 2:
        step("Configure Gesture Camera")
        ensure_gesture_camera(force=True)
    else:
        info("Reusing the shared installer — choose the 'dds' bridge backend (the default) "
             "when prompted for the FIWARE bridge backend.")
        print()
        base.print_install_summary = _dds_install_summary  # show the DDS launcher
        base.mode_install()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n\n{colorize('  Cancelled.', YELLOW)}\n")
        sys.exit(0)
