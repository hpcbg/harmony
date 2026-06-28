#!/usr/bin/env bash
# install_prerequisites.sh — Install system-level prerequisites for Pack Bottle
# Requires: Ubuntu 24.04 (Noble Numbat) with sudo access
#
# Usage: ./install_prerequisites.sh
#
# Installs:
#   • Docker + Docker Compose v2
#   • Node.js 20 LTS and npm (NodeSource)
#   • Vulcanexus Jazzy (ROS 2 + Fast DDS, ARISE-aligned; desktop)

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

install_node20() {
    # The React Dashboard uses Vite 7, which requires Node >= 20.19. The version
    # in Ubuntu 24.04's apt (npm -> Node 18) is too old, so install from NodeSource.
    command -v curl &>/dev/null || sudo apt install -y curl
    info "Installing Node.js 20 LTS via NodeSource..."
    curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
    sudo apt install -y nodejs
    ok "Node.js installed  (node $(node --version), npm $(npm --version))"
}

check_nodejs() {
    step "Node.js 20 LTS and npm"
    if command -v node &>/dev/null; then
        local major; major="$(node --version | sed 's/^v//' | cut -d. -f1)"
        if [[ "$major" -ge 20 ]]; then
            ok "Already installed  (node $(node --version), npm $(npm --version))"
            return
        fi
        warn "Node $(node --version) is too old — the React Dashboard (Vite 7) needs Node >= 20.19."
        if ask_yes_no "Upgrade to Node.js 20 LTS (NodeSource)?"; then
            install_node20
        else
            warn "Skipped — the dashboard dev server will fail with 'crypto.hash is not a function' on Node < 20."
        fi
        return
    fi
    warn "Node.js / npm not found"
    if ask_yes_no "Install Node.js 20 LTS?"; then
        install_node20
    else
        warn "Skipped — required for the React Dashboard"
    fi
}

# ─── Vulcanexus Jazzy (ARISE-aligned ROS 2 / Fast DDS distro) ──────────────────

install_vulcanexus_jazzy() {
    # ARISE fixes Fast DDS as the middleware, so we install Vulcanexus Jazzy
    # (eProsima's Fast-DDS-aligned ROS 2 distro). It is a superset of ROS 2 Jazzy,
    # so the custom-node backend builds and runs on it too.

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

    # Step 3 — Vulcanexus apt repository (https://docs.vulcanexus.org)
    info "Adding the Vulcanexus apt repository..."
    sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key \
        -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian $(source /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/vulcanexus.list > /dev/null

    # Step 4 — Install vulcanexus-jazzy-desktop
    info "Updating package lists..."
    sudo apt update
    info "Installing vulcanexus-jazzy-desktop (~1 GB — this may take several minutes)..."
    sudo apt install -y vulcanexus-jazzy-desktop

    # Step 5 — Source in .bashrc
    local setup_line="source /opt/vulcanexus/jazzy/setup.bash"
    if ! grep -qF "$setup_line" ~/.bashrc; then
        {
            echo ""
            echo "# Vulcanexus Jazzy (ROS 2 + Fast DDS)"
            echo "$setup_line"
        } >> ~/.bashrc
        info "Added '$setup_line' to ~/.bashrc"
    fi

    ok "Vulcanexus Jazzy installed"
    info "Open a new terminal or run 'source ~/.bashrc' to activate the ROS 2 environment."
}

check_ros2() {
    step "Vulcanexus Jazzy (ROS 2 + Fast DDS)"
    if [[ -f /opt/vulcanexus/jazzy/setup.bash ]]; then
        ok "Already installed (Vulcanexus Jazzy)"
        return
    fi
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        ok "Standard ROS 2 Jazzy already installed (Vulcanexus recommended for the DDS path)"
        return
    fi
    warn "Vulcanexus / ROS 2 Jazzy not found"
    if ask_yes_no "Install Vulcanexus Jazzy? (~1 GB download)"; then
        install_vulcanexus_jazzy
    else
        warn "Skipped — required for robot control (ROS 2 + xArm)"
    fi
}

# ─── Main ─────────────────────────────────────────────────────────────────────

echo ""
echo -e "${BOLD}${BLUE}══════════════════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}${CYAN}  Pack Bottle — Prerequisites Installer${RESET}"
echo -e "${DIM}  Installs: Docker, Node.js/npm, Vulcanexus Jazzy (ROS 2 + Fast DDS)${RESET}"
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
