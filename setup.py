#!/usr/bin/env python3
"""Pack Bottle Setup Assistant — Installation wizard and daily startup launcher."""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# ─── ANSI output helpers ──────────────────────────────────────────────────────

RESET  = "\033[0m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
BLUE   = "\033[94m"
CYAN   = "\033[96m"

def colorize(text: str, *codes: str) -> str:
    return "".join(codes) + str(text) + RESET

def ok(text: str):   print(colorize(f"  ✓ {text}", GREEN))
def warn(text: str): print(colorize(f"  ! {text}", YELLOW))
def err(text: str):  print(colorize(f"  ✗ {text}", RED))
def info(text: str): print(colorize(f"  → {text}", CYAN))

def step(title: str):
    bar = "═" * 60
    print(f"\n{colorize(bar, BOLD, BLUE)}")
    print(colorize(f"  {title}", BOLD, CYAN))
    print(colorize(bar, BOLD, BLUE))

# ─── Paths and persistent state ───────────────────────────────────────────────

SCRIPT_DIR = Path(__file__).parent.resolve()
STATE_FILE = SCRIPT_DIR / ".setup_state.json"


def load_state() -> dict:
    try:
        return json.loads(STATE_FILE.read_text())
    except Exception:
        return {}


def save_state(state: dict):
    try:
        STATE_FILE.write_text(json.dumps(state, indent=2))
    except Exception:
        pass  # non-critical

# ─── Input helpers ────────────────────────────────────────────────────────────

def ask(prompt: str, default: str = "", required: bool = True) -> str:
    hint = colorize(f" [{default}]", DIM) if default else ""
    while True:
        val = input(f"  {prompt}{hint}: ").strip()
        if not val:
            val = default
        if val or not required:
            return val
        err("This field is required.")


def ask_yes_no(prompt: str, default: bool = True) -> bool:
    hint = "Y/n" if default else "y/N"
    while True:
        val = input(f"  {prompt} [{hint}]: ").strip().lower()
        if not val:
            return default
        if val in ("y", "yes"):
            return True
        if val in ("n", "no"):
            return False
        err("Please enter y or n.")


def ask_choice(prompt: str, choices: list, default: int = 0) -> int:
    print(f"\n  {prompt}")
    for i, label in enumerate(choices):
        marker = colorize("*", GREEN) if i == default else " "
        print(f"    {marker} {i + 1}) {label}")
    while True:
        val = input(f"\n  Choice [1-{len(choices)}] (default {default + 1}): ").strip()
        if not val:
            return default
        try:
            idx = int(val) - 1
            if 0 <= idx < len(choices):
                return idx
        except ValueError:
            pass
        err(f"Enter a number from 1 to {len(choices)}.")


def validate_ip(ip: str) -> bool:
    return bool(re.match(r"^\d{1,3}(\.\d{1,3}){3}$", ip))


def safe_copy_template(src: Path, dst: Path) -> bool:
    """Returns True if the caller should proceed with writing the file."""
    if dst.exists():
        warn(f"Config already exists: {dst.relative_to(SCRIPT_DIR)}")
        return ask_yes_no("Overwrite?", default=False)
    return True


def run_cmd(cmd: list, cwd: Path = None, quiet: bool = False, env: dict = None) -> bool:
    kwargs: dict = {"cwd": cwd or SCRIPT_DIR}
    if quiet:
        kwargs["capture_output"] = True
    if env is not None:
        kwargs["env"] = env
    result = subprocess.run(cmd, **kwargs)
    if result.returncode != 0:
        err(f"Command failed (exit {result.returncode}): {' '.join(str(c) for c in cmd)}")
        return False
    return True

# ═══════════════════════════════════════════════════════════════════════════════
# STARTUP MODE
# ═══════════════════════════════════════════════════════════════════════════════

# Services that can be started, in launch order.
# cmd_tpl placeholders: {root} = SCRIPT_DIR, {camera} = gesture camera device ID
STARTUP_SERVICES = [
    {
        "key":      "fiware",
        "label":    "FIWARE Stack (Docker)",
        "checks":   [],   # readiness verified via Docker separately
        "cmd_tpl":  "cd {root}/fiware-analytics-docker && docker compose up -d",
    },
    {
        "key":      "dashboard",
        "label":    "React Dashboard",
        "checks":   ["react-dashboard/node_modules"],
        "cmd_tpl":  "cd {root}/react-dashboard && ./run.sh",
    },
    {
        "key":      "ai",
        "label":    "AI Bottle Detector",
        "checks":   [
            "torch_venv/bin/activate",
            "ai-bottle-detector-fiware/config/config.json",
            "ai-bottle-detector-fiware/models/best_model.pth",
        ],
        "cmd_tpl":  "source {root}/torch_venv/bin/activate && cd {root}/ai-bottle-detector-fiware && ./run.sh",
    },
    {
        "key":      "gesture",
        "label":    "Hand Gesture Recognition",
        "checks":   ["venv/bin/activate"],
        "cmd_tpl":  "sleep 5 && source {root}/venv/bin/activate && cd {root}/gesture-commands-fiware && python gesture-commands-fiware.py --camera {camera}",
    },
    {
        "key":      "voice",
        "label":    "Voice Commands",
        "checks":   ["venv/bin/activate"],
        "cmd_tpl":  "sleep 5 && source {root}/venv/bin/activate && cd {root}/voice-commands-fiware && python voice-commands-fiware.py --fiware",
    },
    {
        "key":      "ros2",
        "label":    "ROS 2 + xArm Control",
        "checks":   [
            "ros2-xarm-pack-bottle/ros2_ws/config/xarm_pack_bottle.json",
            "ros2-xarm-pack-bottle/ros2_ws/install",
        ],
        "cmd_tpl":  "sleep 20 && cd {root}/ros2-xarm-pack-bottle && ./run.sh",
    },
]


def _service_issues(svc: dict) -> list:
    """Return list of missing paths for a startup service."""
    return [c for c in svc["checks"] if not (SCRIPT_DIR / c).exists()]


def _docker_available() -> bool:
    result = subprocess.run(["docker", "compose", "version"],
                            capture_output=True, text=True)
    return result.returncode == 0


def detect_terminal() -> str:
    """Return the best available terminal emulator name."""
    if os.environ.get("TMUX"):
        return "tmux"
    for term in ("gnome-terminal", "konsole", "xterm"):
        if shutil.which(term):
            return term
    return "none"


def _launch_in_terminal(terminal: str, label: str, cmd: str):
    """Open cmd in a new tab/window of terminal."""
    tmp_fd, tmp_path = tempfile.mkstemp(suffix=".sh", prefix="pack_bottle_")
    os.close(tmp_fd)
    with open(tmp_path, "w") as f:
        f.write(f"#!/usr/bin/env bash\n{cmd}\nexec bash\n")
    os.chmod(tmp_path, 0o755)

    if terminal == "gnome-terminal":
        subprocess.Popen(
            ["gnome-terminal", "--tab", f"--title={label}", "--", "bash", tmp_path]
        )
    elif terminal == "konsole":
        subprocess.Popen(
            ["konsole", "--new-tab", f"--name={label}", "-e", f"bash '{tmp_path}'"]
        )
    elif terminal == "xterm":
        subprocess.Popen(
            ["xterm", "-title", label, "-e", f"bash '{tmp_path}'"]
        )
    elif terminal == "tmux":
        subprocess.Popen(
            ["tmux", "new-window", "-n", label[:20], f"bash '{tmp_path}'"]
        )


def mode_startup():
    step("Daily Startup")
    state = load_state()

    # ── Readiness table ───────────────────────────────────────────────────────
    print(f"\n  {colorize('Service readiness:', BOLD)}\n")

    docker_ok = _docker_available()
    ready_keys = set()

    for svc in STARTUP_SERVICES:
        key    = svc["key"]
        label  = svc["label"]
        issues = _service_issues(svc)

        if key == "fiware":
            if docker_ok:
                ok(f"{label}")
                ready_keys.add(key)
            else:
                err(f"{label}  — Docker / Docker Compose not available")
        elif issues:
            missing = ", ".join(issues)
            warn(f"{label}  — not configured ({missing})")
        else:
            ok(f"{label}")
            ready_keys.add(key)

    # ── Service selection ─────────────────────────────────────────────────────
    step("Select Services to Start")
    print("  Services that are not yet configured are deselected by default.\n")

    selected = set(ready_keys)

    while True:
        print()
        for i, svc in enumerate(STARTUP_SERVICES):
            key   = svc["key"]
            label = svc["label"]
            mark  = colorize("[x]", GREEN) if key in selected else colorize("[ ]", DIM)
            note  = "" if key in ready_keys else colorize("  (not configured)", YELLOW)
            print(f"   {mark} {i + 1}) {label}{note}")

        val = input(
            colorize(f"\n  Toggle 1-{len(STARTUP_SERVICES)} · A=all-ready · N=none · Enter=launch: ", DIM)
        ).strip().lower()

        if not val:
            break
        if val == "a":
            selected = set(ready_keys)
        elif val == "n":
            selected = set()
        else:
            try:
                idx = int(val) - 1
                if 0 <= idx < len(STARTUP_SERVICES):
                    key = STARTUP_SERVICES[idx]["key"]
                    selected ^= {key}
            except ValueError:
                pass

    if not selected:
        warn("Nothing selected — exiting.")
        return

    # ── Camera ID (gesture only) ───────────────────────────────────────────────
    camera = state.get("camera", 0)
    if "gesture" in selected:
        print()
        camera_str = ask("Camera device ID for gesture recognition", default=str(camera))
        try:
            camera = int(camera_str)
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

    # ── Confirm and launch ─────────────────────────────────────────────────────
    selected_labels = [s["label"] for s in STARTUP_SERVICES if s["key"] in selected]
    print(f"\n  {colorize('Will start:', BOLD)} {', '.join(selected_labels)}\n")

    if not ask_yes_no("Launch now?", default=True):
        info("Aborted.")
        return

    root = str(SCRIPT_DIR)
    for svc in STARTUP_SERVICES:
        if svc["key"] not in selected:
            continue
        cmd = svc["cmd_tpl"].format(root=root, camera=camera)
        if terminal == "none":
            print(f"\n  {colorize(svc['label'], BOLD)}:\n    {cmd}")
        else:
            info(f"Launching {svc['label']}...")
            _launch_in_terminal(terminal, svc["label"], cmd)

    # Persist state
    state["camera"] = camera
    save_state(state)

    if terminal != "none":
        print()
        ok("All selected services launched in separate terminal windows/tabs.")
        info("FIWARE and Dashboard start immediately; Gesture/Voice after 5 s; ROS 2 after 20 s.")
    else:
        print(f"\n  {colorize('Copy the commands above and run each in its own terminal.', YELLOW)}")

# ═══════════════════════════════════════════════════════════════════════════════
# INSTALL MODE
# ═══════════════════════════════════════════════════════════════════════════════

ALL_COMPONENTS = [
    ("fiware",    "FIWARE Stack (Docker)     — Orion, CrateDB, Grafana, QuantumLeap"),
    ("ai",        "AI Bottle Detector        — Computer vision + REST API (port 22001)"),
    ("gesture",   "Hand Gesture Recognition  — MediaPipe-based gesture control"),
    ("voice",     "Voice Commands            — Vosk offline speech recognition"),
    ("dashboard", "React Dashboard           — Web UI for monitoring and control"),
    ("ros2",      "ROS 2 + xArm Control     — Robot orchestration (requires ROS 2 Jazzy)"),
    ("iot",       "IoT Firmware Config       — M5StickCPlus2 header file generation"),
]


def select_components() -> set:
    step("Select Components to Configure")
    print("""
  Toggle individual components, then press Enter to confirm.
  All are selected by default for a full installation.
""")
    selected = {key for key, _ in ALL_COMPONENTS}

    while True:
        print()
        for i, (key, label) in enumerate(ALL_COMPONENTS):
            mark = colorize("[x]", GREEN) if key in selected else colorize("[ ]", DIM)
            print(f"   {mark} {i + 1}) {label}")

        val = input(
            colorize("\n  Toggle 1-7 · A=all · N=none · Enter=confirm: ", DIM)
        ).strip().lower()

        if not val:
            break
        # `val` is already .strip().lower()'d above, so == is case-insensitive
        # (A/N work). The previous `val == "a" or "A"` was always truthy and
        # forced "select all" on every keystroke, killing toggle and N.
        if val == "a":
            selected = {key for key, _ in ALL_COMPONENTS}
        elif val == "n":
            selected = set()
        else:
            try:
                idx = int(val) - 1
                if 0 <= idx < len(ALL_COMPONENTS):
                    key = ALL_COMPONENTS[idx][0]
                    selected ^= {key}
            except ValueError:
                pass

    selected_labels = [lbl.split("—")[0].strip() for key, lbl in ALL_COMPONENTS if key in selected]
    print(f"\n  {colorize('Will configure:', BOLD)} {', '.join(selected_labels) or 'nothing'}")
    return selected


def check_prerequisites(components: set) -> bool:
    step("Checking Prerequisites")
    all_ok = True
    missing = []

    # Docker
    docker_ok = (
        shutil.which("docker") is not None and
        subprocess.run(["docker", "compose", "version"],
                       capture_output=True).returncode == 0
    )
    if docker_ok:
        ok("Docker + Docker Compose v2")
    else:
        err("Docker / Docker Compose v2 not found")
        missing.append("Docker")
        all_ok = False

    # Node.js / npm
    if {"dashboard"} & components:
        if shutil.which("npm"):
            ok("Node.js and npm")
        else:
            err("npm / Node.js not found")
            missing.append("Node.js and npm")
            all_ok = False

    # Python 3
    if {"ai", "gesture", "voice"} & components:
        if shutil.which("python3"):
            ok("Python 3")
        else:
            err("Python 3 not found")
            missing.append("Python 3")
            all_ok = False

    # ROS 2 Jazzy
    if "ros2" in components:
        if Path("/opt/ros/jazzy/setup.bash").exists():
            ok("ROS 2 Jazzy")
        else:
            warn("ROS 2 Jazzy not found — workspace build will be skipped")
            missing.append("ROS 2 Jazzy")

    if missing:
        print(f"\n  {colorize('Missing:', BOLD)} {', '.join(missing)}")
        print(f"  Run {colorize('./install_prerequisites.sh', BOLD)} to install them, then re-run setup.py.")

    return all_ok


def gather_global_config() -> dict:
    step("Network Configuration")
    state = load_state()
    print("  The FIWARE host IP is this machine's LAN address, used by all components.\n")

    while True:
        default_ip = state.get("fiware_host", "192.168.1.105")
        fiware_host = ask("This machine's LAN IP address (FIWARE host)", default=default_ip)
        if validate_ip(fiware_host):
            break
        err("Invalid IP — enter four dot-separated numbers (e.g. 192.168.1.105)")

    state["fiware_host"] = fiware_host
    save_state(state)
    return {"fiware_host": fiware_host}


def setup_fiware():
    step("FIWARE Docker Stack")
    fiware_dir = SCRIPT_DIR / "fiware-analytics-docker"
    info("Pulling Docker images (may take several minutes on first run)...")
    if run_cmd(["docker", "compose", "pull"], cwd=fiware_dir):
        ok("Images pulled — use Daily Startup to launch the stack")
    else:
        warn("Image pull had errors — try 'docker compose pull' manually")


def setup_ai_detector(cfg: dict):
    step("AI Bottle Detector")

    src = SCRIPT_DIR / "ai-bottle-detector-fiware" / "config" / "config.json.tpl"
    dst = SCRIPT_DIR / "ai-bottle-detector-fiware" / "config" / "config.json"

    if safe_copy_template(src, dst):
        with open(src) as f:
            ai_cfg = json.load(f)

        ai_cfg["APP_API_HOST"] = cfg["fiware_host"]

        camera_type = ask_choice(
            "Camera source:",
            [
                "USB / built-in camera (integer device ID)",
                "RTSP stream (e.g. M5Stack Timer Camera X)",
            ],
        )
        if camera_type == 0:
            cam_id = ask("Camera device ID", default=str(ai_cfg.get("CAMERA", 2)))
            try:
                ai_cfg["CAMERA"] = int(cam_id)
            except ValueError:
                ai_cfg["CAMERA"] = cam_id
        else:
            ai_cfg["CAMERA"] = ask("RTSP stream URL")

        width  = ask("Capture width",  default=str(ai_cfg.get("WIDTH",  1600)))
        height = ask("Capture height", default=str(ai_cfg.get("HEIGHT", 1200)))
        ai_cfg["WIDTH"]  = int(width)
        ai_cfg["HEIGHT"] = int(height)

        print(f"\n  {colorize('ArUco corner marker IDs:', BOLD)}")
        print(f"  Current: {ai_cfg['CORNER_MARKERS']}  (top-left → bottom-left, clockwise)")
        if ask_yes_no("Change marker IDs?", default=False):
            for i, lbl in enumerate(["Top-left", "Top-right", "Bottom-right", "Bottom-left"]):
                mid = ask(f"  {lbl} marker ID", default=str(ai_cfg["CORNER_MARKERS"][i]))
                ai_cfg["CORNER_MARKERS"][i] = int(mid)

        dst.parent.mkdir(parents=True, exist_ok=True)
        with open(dst, "w") as f:
            json.dump(ai_cfg, f, indent=2)
        ok(f"Written: {dst.relative_to(SCRIPT_DIR)}")

    print(f"\n  {colorize('Python environment (torch_venv)', BOLD)}")
    warn("PyTorch is not in requirements.txt and must be installed separately.")
    torch_choice = ask_choice(
        "PyTorch variant:",
        [
            "CPU only    (works on any machine, slower inference)",
            "CUDA 12.x   (NVIDIA GPU — faster inference)",
            "Skip        (install PyTorch manually later)",
        ],
    )

    venv_dir = SCRIPT_DIR / "torch_venv"
    pip      = venv_dir / "bin" / "pip"

    info("Creating torch_venv...")
    run_cmd(["python3", "-m", "venv", str(venv_dir)])
    info("Installing AI detector requirements...")
    run_cmd([str(pip), "install", "-r",
             str(SCRIPT_DIR / "ai-bottle-detector-fiware" / "requirements.txt")])

    if torch_choice == 0:
        info("Installing PyTorch CPU...")
        run_cmd([str(pip), "install", "torch", "torchvision",
                 "--index-url", "https://download.pytorch.org/whl/cpu"])
        ok("torch_venv ready (CPU)")
    elif torch_choice == 1:
        info("Installing PyTorch CUDA 12.x...")
        run_cmd([str(pip), "install", "torch", "torchvision",
                 "--index-url", "https://download.pytorch.org/whl/cu121"])
        ok("torch_venv ready (CUDA 12.x)")
    else:
        warn("Skipped — activate torch_venv and pip install torch before running the detector")

    model_path = SCRIPT_DIR / "ai-bottle-detector-fiware" / "models" / "best_model.pth"
    model_url = ("https://huggingface.co/hpcbg/harmony-bottle-detector/"
                 "resolve/main/best_model.pth")
    if model_path.exists():
        ok(f"Model weights present: {model_path.relative_to(SCRIPT_DIR)}")
    else:
        print()
        warn("No trained model at ai-bottle-detector-fiware/models/best_model.pth — "
             "without it the detector exits with FileNotFoundError.")
        if ask_yes_no("Download the trained model (~159 MB) from Hugging Face "
                      "(hpcbg/harmony-bottle-detector)?", default=True):
            model_path.parent.mkdir(parents=True, exist_ok=True)
            info("Downloading best_model.pth...")
            if run_cmd(["curl", "-L", "--fail", "-o", str(model_path), model_url]):
                ok(f"Model weights downloaded: {model_path.relative_to(SCRIPT_DIR)}")
            else:
                warn(f"Download failed — fetch manually from {model_url}")
        else:
            info(f"Get the weights later from {model_url}")
            info("or train your own: python "
                 "ai-bottle-detector-fiware/utils/train_bottle_detector.py")


def setup_gesture_voice(components: set):
    step("Gesture & Voice Commands — Shared Python Environment")

    venv_dir = SCRIPT_DIR / "venv"
    pip      = venv_dir / "bin" / "pip"

    info("Creating shared venv...")
    run_cmd(["python3", "-m", "venv", str(venv_dir)])

    if "gesture" in components:
        info("Installing gesture requirements...")
        run_cmd([str(pip), "install", "-r",
                 str(SCRIPT_DIR / "gesture-commands-fiware" / "requirements.txt")])
        ok("Gesture dependencies installed")

    if "voice" in components:
        # sounddevice is a thin wrapper over PortAudio; without the system lib it
        # imports with "OSError: PortAudio library not found".
        info("Installing PortAudio system library (sounddevice depends on it)...")
        run_cmd(["sudo", "apt-get", "install", "-y", "libportaudio2"])
        info("Installing voice requirements...")
        run_cmd([str(pip), "install", "-r",
                 str(SCRIPT_DIR / "voice-commands-fiware" / "requirements.txt")])
        ok("Voice dependencies installed")
        print()
        warn("Vosk needs a language model file to function.")
        info("Download from: https://alphacephei.com/vosk/models")
        info("Extract into voice-commands-fiware/ and update the model path in the script.")


def setup_dashboard():
    step("React Dashboard")
    dash_dir = SCRIPT_DIR / "react-dashboard"
    # Vite 7 / @vitejs/plugin-react 5 require Node >= 20.19. On Node 18 npm install
    # still succeeds, but the dev server later dies with "crypto.hash is not a
    # function" — surface that up front instead of as a cryptic runtime crash.
    try:
        node_ver = subprocess.run(["node", "--version"], capture_output=True,
                                  text=True).stdout.strip().lstrip("v")
        node_major = int(node_ver.split(".")[0]) if node_ver else 0
    except Exception:
        node_ver, node_major = "", 0
    if node_major and node_major < 20:
        warn(f"Node {node_ver} detected — the dashboard (Vite 7) needs Node >= 20.19.")
        info("Install Node 20 LTS (e.g. via nvm: 'nvm install 20') before ./run.sh, "
             "or the dev server will fail with 'crypto.hash is not a function'.")
    info("Running npm install (may take a minute)...")
    if run_cmd(["npm", "install"], cwd=dash_dir):
        ok("Dashboard dependencies installed")
    else:
        warn("npm install had errors — check output above")


def setup_ros2(cfg: dict):
    step("ROS 2 + xArm Configuration")
    ros_available = Path("/opt/ros/jazzy/setup.bash").exists()

    src_xarm = SCRIPT_DIR / "ros2-xarm-pack-bottle" / "ros2_ws" / "config" / "xarm_pack_bottle.json.tpl"
    dst_xarm = SCRIPT_DIR / "ros2-xarm-pack-bottle" / "ros2_ws" / "config" / "xarm_pack_bottle.json"

    if safe_copy_template(src_xarm, dst_xarm):
        with open(src_xarm) as f:
            xarm_cfg = json.load(f)

        emulate = ask_yes_no("Run in emulation mode (no physical xArm7 required)?", default=True)
        xarm_cfg["EMULATE_ROBOT"] = emulate

        if not emulate:
            while True:
                robot_ip = ask("xArm7 robot IP address", default=xarm_cfg["ROBOT_IP"])
                if validate_ip(robot_ip):
                    break
                err("Invalid IP address format.")
            xarm_cfg["ROBOT_IP"] = robot_ip

        dst_xarm.parent.mkdir(parents=True, exist_ok=True)
        with open(dst_xarm, "w") as f:
            json.dump(xarm_cfg, f, indent=2)
        ok(f"Written: {dst_xarm.relative_to(SCRIPT_DIR)}")

    src_bridge = (SCRIPT_DIR / "ros2-xarm-pack-bottle" / "ros2_ws"
                  / "config" / "fiware_bridge_config.yaml.tpl")
    # complete.launch.py loads './config/fiware_bridge_config.yaml' relative to
    # ros2_ws, so render the deployment config there (gitignored), NOT into the
    # tracked sample under src/fiware_bridge/config/.
    dst_bridge  = (SCRIPT_DIR / "ros2-xarm-pack-bottle" / "ros2_ws"
                   / "config" / "fiware_bridge_config.yaml")

    if safe_copy_template(src_bridge, dst_bridge):
        content = src_bridge.read_text().replace("localhost", cfg["fiware_host"])
        dst_bridge.parent.mkdir(parents=True, exist_ok=True)
        dst_bridge.write_text(content)
        ok(f"Written: {dst_bridge.relative_to(SCRIPT_DIR)}")

    # FIWARE bridge backend: custom node (default) vs ARISE-native DDS enabler.
    # node is the integrated-demo default — voice/gesture/AI/dashboard/analytics
    # all use NGSI-v2 on standard Orion, which only the node backend targets. dds
    # is a standalone, bridge-only path (Orion-LD/NGSI-LD) that can't coexist with
    # that stack (same host port 1026). Both read the same fiware_bridge_config.yaml.
    backend_idx = ask_choice(
        "FIWARE bridge backend",
        ["node — custom Python bridge over NGSI-v2 (default; integrated demo, full type + value-mapping coverage)",
         "dds  — Orion-LD built-in DDS enabler, NGSI-LD (standalone bridge-only; all scalar std_msgs)"],
        default=0)
    backend = "dds" if backend_idx == 1 else "node"
    cfg["bridge_backend"] = backend

    if backend == "dds":
        dds_dir = (SCRIPT_DIR / "ros2-xarm-pack-bottle" / "ros2_ws" / "src"
                   / "fiware_bridge" / "dds")
        info("Generating DDS mapping (context_broker_config.json) from the "
             "deployment config...")
        if run_cmd(["python3", "generate_config.py", "--config", str(dst_bridge)],
                   cwd=dds_dir):
            ok("DDS mapping generated — review the eligibility/transform notes above")
        else:
            warn("generate_config.py failed — ensure pyyaml is installed (pip install pyyaml)")
        warn("DDS Orion-LD and the default Orion stack both bind host port 1026 — "
             "do not run the 'fiware' analytics stack and the DDS broker at the same time.")
        info("Start the DDS broker:  cd ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds "
             "&& docker compose -f docker-compose.dds.yml up -d")

    if not ros_available:
        warn("Skipping colcon build — install ROS 2 Jazzy first")
        return

    if ask_yes_no("\nBuild the ROS 2 workspace now? (1-3 minutes)", default=True):
        info("Installing ros-jazzy-py-trees...")
        run_cmd(["sudo", "apt-get", "install", "-y", "ros-jazzy-py-trees"])
        ws_dir = SCRIPT_DIR / "ros2-xarm-pack-bottle" / "ros2_ws"
        info("Running colcon build...")
        # rosidl (building custom_interfaces) runs under whatever python3 is first
        # on PATH. If a venv is active it lacks empy/lark and the build dies with
        # "ModuleNotFoundError: No module named 'em'", aborting the whole workspace.
        # Build with the system Python that the ROS install provides instead.
        build_env = os.environ.copy()
        active_venv = build_env.pop("VIRTUAL_ENV", None)
        if active_venv:
            build_env["PATH"] = os.pathsep.join(
                p for p in build_env.get("PATH", "").split(os.pathsep)
                if p and not p.startswith(active_venv))
            info("Active virtualenv detected — building with the system Python "
                 "(rosidl needs empy/lark from the ROS install, not the venv).")
        # A prior build done under a venv bakes that python into each package's
        # CMakeCache.txt, so it keeps failing even once PATH is fixed. Detect a
        # poisoned cache and wipe the build artifacts for a clean rebuild.
        build_dir = ws_dir / "build"
        poisoned = [c for c in build_dir.glob("*/CMakeCache.txt")
                    if "/venv/" in c.read_text(errors="ignore")]
        if poisoned:
            warn(f"Found {len(poisoned)} package(s) whose CMake cache points at a "
                 "venv Python — wiping build/install/log for a clean rebuild.")
            for d in ("build", "install", "log"):
                shutil.rmtree(ws_dir / d, ignore_errors=True)
        if run_cmd(["bash", "-c", "source /opt/ros/jazzy/setup.bash && colcon build"],
                   cwd=ws_dir, env=build_env):
            ok("ROS 2 workspace built")
        else:
            warn("colcon build reported errors — check output above")


def setup_iot(cfg: dict):
    step("IoT Firmware — M5StickCPlus2 Configuration")

    src = SCRIPT_DIR / "iot-device-firmware" / "M5StickCPlus2-to-FIWARE" / "config.h.tpl"
    dst = SCRIPT_DIR / "iot-device-firmware" / "M5StickCPlus2-to-FIWARE" / "config.h"

    if not safe_copy_template(src, dst):
        info("Keeping existing config.h")
        return

    ssid     = ask("WiFi network name (SSID)")
    password = ask("WiFi password", required=False)

    content = src.read_text()
    content = content.replace('"<SSID>"',     f'"{ssid}"')
    content = content.replace('"<password>"', f'"{password}"')
    content = content.replace("http://localhost:1026", f"http://{cfg['fiware_host']}:1026")

    dst.write_text(content)
    ok(f"Written: {dst.relative_to(SCRIPT_DIR)}")
    info("Flash config.h + the .ino sketch via Arduino IDE 2.3.6")
    info("Board: M5Stack v3.2.5  ·  Library: M5Stack v0.4.6")


def print_install_summary(components: set, cfg: dict):
    step("Installation Complete")
    host = cfg["fiware_host"]

    rows = []
    if "fiware"    in components: rows.append(("FIWARE",       "cd fiware-analytics-docker && docker compose up -d"))
    if "ai"        in components: rows.append(("AI Detector",  "source torch_venv/bin/activate && cd ai-bottle-detector-fiware && ./run.sh"))
    if "gesture"   in components: rows.append(("Gesture",      "source venv/bin/activate && cd gesture-commands-fiware && python gesture-commands-fiware.py"))
    if "voice"     in components: rows.append(("Voice",        "source venv/bin/activate && cd voice-commands-fiware && python voice-commands-fiware.py --fiware"))
    if "dashboard" in components: rows.append(("Dashboard",    "cd react-dashboard && ./run.sh"))
    if "ros2"      in components:
        backend = cfg.get("bridge_backend", "node")
        if backend == "dds":
            rows.append(("FIWARE DDS",  "cd ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds && docker compose -f docker-compose.dds.yml up -d   # Orion-LD DDS broker (replaces the standard Orion stack)"))
            rows.append(("ROS 2",       "cd ros2-xarm-pack-bottle && BRIDGE_BACKEND=dds ./run.sh   # standalone bridge-only DDS path"))
        else:
            rows.append(("ROS 2",       "cd ros2-xarm-pack-bottle && ./run.sh"))
    if "iot"       in components: rows.append(("IoT Firmware", "Flash with Arduino IDE 2.3.6"))

    if rows:
        col = max(len(name) for name, _ in rows)
        print(f"\n  {colorize('Manual start commands:', BOLD)}\n")
        for name, cmd in rows:
            print(f"  {colorize(name.ljust(col), GREEN)}  {cmd}")

    print(f"""
  {colorize('Next time, use Daily Startup mode:', BOLD)}
    python3 setup.py  →  choose option 2

  {colorize('Or launch everything at once:', BOLD)}
    ./launch_pack_bottle.sh

  {colorize('Service endpoints:', BOLD)}
    Grafana   →  http://{host}:4000    (admin / admin)
    Orion     →  http://{host}:1026
    AI API    →  http://{host}:22001
    CrateDB   →  http://{host}:4200
""")


def mode_install():
    components = select_components()
    if not components:
        warn("No components selected — nothing to do.")
        return

    prereqs_ok = check_prerequisites(components)
    if not prereqs_ok:
        print()
        if not ask_yes_no("Some prerequisites are missing. Continue anyway?", default=False):
            sys.exit(1)

    cfg = gather_global_config()

    if "fiware"    in components:                   setup_fiware()
    if "ai"        in components:                   setup_ai_detector(cfg)
    if {"gesture", "voice"} & components:           setup_gesture_voice(components)
    if "dashboard" in components:                   setup_dashboard()
    if "ros2"      in components:                   setup_ros2(cfg)
    if "iot"       in components:                   setup_iot(cfg)

    print_install_summary(components, cfg)

# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print(f"""
{colorize('═' * 60, BOLD, BLUE)}
{colorize('  Pack Bottle  —  Setup Assistant', BOLD, CYAN)}
{colorize('  Robotic Bottle Packing System', DIM)}
{colorize('═' * 60, BOLD, BLUE)}
""")

    mode = ask_choice(
        "What would you like to do?",
        [
            "Daily Startup    — check readiness and launch services",
            "First-time Install — install dependencies and generate config files",
        ],
        default=0,
    )

    print()
    if mode == 0:
        mode_startup()
    else:
        mode_install()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n\n{colorize('  Cancelled.', YELLOW)}\n")
        sys.exit(0)
