#!/usr/bin/env bash
# install_prerequisites.sh — Install system-level prerequisites for Pack Bottle
# Requires: Ubuntu 24.04 (Noble Numbat) with sudo access
#
# Usage: ./install_prerequisites.sh
#
# Installs:
#   • Docker + Docker Compose v2
#   • Node.js and npm
#   • ROS 2 Jazzy (desktop)

set -euo pipefail

# ─── Output helpers ───────────────────────────────────────────────────────────

RESET='\033[0m'; BOLD='\033[1m'; DIM='\033[2m'
RED='\033[91m'; GREEN='\033[92m'; YELLOW='\033[93m'
BLUE='\033[94m'; CYAN='\033[96m'

ok()   { echo -e "${GREEN}  ✓ $*${RESET}"; }
warn() { echo -e "${YELLOW}  ! $*${RESET}"; }
err()  { echo -e "${RED}  ✗ $*${RESET}"; }
info() { echo -e "${CYAN}  → $*${RESET}"; }
step() {
    local bar='══════════════════════════════════════════════════════════════'
    echo -e "\n${BOLD}${BLUE}${bar}${RESET}"
    echo -e "${BOLD}${CYAN}  $*${RESET}"
    echo -e "${BOLD}${BLUE}${bar}${RESET}"
}

ask_yes_no() {
    local prompt="$1" default="${2:-y}"
    local hint; [[ "$default" == "y" ]] && hint="Y/n" || hint="y/N"
    while true; do
        read -rp "  ${prompt} [${hint}]: " ans
        ans="${ans:-$default}"
        case "${ans,,}" in
            y|yes) return 0 ;;
            n|no)  return 1 ;;
            *)     echo "  Please enter y or n." ;;
        esac
    done
}

# ─── Docker ───────────────────────────────────────────────────────────────────

install_docker() {
    info "Installing docker.io and docker-compose-v2..."
    sudo apt install -y docker.io docker-compose-v2

    info "Adding ${USER} to the docker group..."
    sudo usermod -aG docker "$USER"

    info "Enabling and starting Docker service..."
    sudo systemctl enable docker
    sudo systemctl start docker

    ok "Docker installed and ${USER} added to docker group"
    echo ""
    warn "The docker group is not active in this terminal session yet."
    info "Run the command below now to activate it without logging out:"
    echo ""
    echo -e "    ${BOLD}newgrp docker${RESET}"
    echo ""
    info "After that, re-run this script or proceed to: python3 setup.py"
}

check_docker() {
    step "Docker + Docker Compose v2"
    if command -v docker &>/dev/null && docker compose version &>/dev/null 2>&1; then
        ok "Already installed"
        return
    fi
    warn "Docker or Docker Compose v2 not found"
    if ask_yes_no "Install Docker and Docker Compose v2?"; then
        install_docker
    else
        warn "Skipped — Docker is required for the FIWARE stack"
    fi
}

# ─── Node.js / npm ────────────────────────────────────────────────────────────

check_nodejs() {
    step "Node.js and npm"
    if command -v npm &>/dev/null; then
        ok "Already installed  (npm $(npm --version))"
        return
    fi
    warn "npm / Node.js not found"
    if ask_yes_no "Install Node.js and npm?"; then
        info "Running: sudo apt install npm"
        sudo apt install -y npm
        ok "Node.js and npm installed"
    else
        warn "Skipped — required for the React Dashboard"
    fi
}

# ─── ROS 2 Jazzy ──────────────────────────────────────────────────────────────

install_ros2_jazzy() {
    # Step 1 — Locale
    info "Configuring locale..."
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Step 2 — Universe repository
    info "Enabling Universe repository..."
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y universe

    # Step 3 — ROS 2 apt source package
    info "Adding ROS 2 apt repository (fetching latest ros-apt-source release)..."
    sudo apt install -y curl
    ROS_APT_SOURCE_VERSION=$(
        curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" | awk -F'"' '{print $4}'
    )
    UBUNTU_CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
    DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"
    curl -L -o /tmp/ros2-apt-source.deb "$DEB_URL"
    sudo dpkg -i /tmp/ros2-apt-source.deb

    # Step 4 — Install ros-jazzy-desktop
    info "Updating package lists..."
    sudo apt update
    sudo apt upgrade -y
    info "Installing ros-jazzy-desktop (~1 GB — this may take several minutes)..."
    sudo apt install -y ros-jazzy-desktop

    # Step 5 — Source in .bashrc
    local setup_line="source /opt/ros/jazzy/setup.bash"
    if ! grep -qF "$setup_line" ~/.bashrc; then
        {
            echo ""
            echo "# ROS 2 Jazzy"
            echo "$setup_line"
        } >> ~/.bashrc
        info "Added '$setup_line' to ~/.bashrc"
    fi

    ok "ROS 2 Jazzy installed"
    info "Open a new terminal or run 'source ~/.bashrc' to activate the ROS 2 environment."
}

check_ros2() {
    step "ROS 2 Jazzy"
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        ok "Already installed"
        return
    fi
    warn "ROS 2 Jazzy not found"
    if ask_yes_no "Install ROS 2 Jazzy? (~1 GB download)"; then
        install_ros2_jazzy
    else
        warn "Skipped — required for robot control (ROS 2 + xArm)"
    fi
}

# ─── Main ─────────────────────────────────────────────────────────────────────

echo ""
echo -e "${BOLD}${BLUE}══════════════════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}${CYAN}  Pack Bottle — Prerequisites Installer${RESET}"
echo -e "${DIM}  Installs: Docker, Node.js/npm, ROS 2 Jazzy${RESET}"
echo -e "${BOLD}${BLUE}══════════════════════════════════════════════════════════════${RESET}"
echo ""

info "Updating package lists..."
sudo apt update

check_docker
check_nodejs
check_ros2

echo ""
step "Done"
ok "Prerequisites check complete."
echo ""
info "Next step: run  ${BOLD}python3 setup.py${RESET}${CYAN}  to configure the project."
echo ""
