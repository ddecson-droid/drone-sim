#!/bin/bash
# =============================================================================
# Drone Simulation Workspace Setup Script
# Target: Ubuntu 24.10 / ROS 2 Jazzy / PX4 SITL / Gazebo Harmonic
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh
# =============================================================================
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

WS_DIR="$(cd "$(dirname "$0")" && pwd)"

echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  Drone Simulation Workspace Setup${NC}"
echo -e "${GREEN}  PX4 + Gazebo Harmonic + ROS 2 Jazzy + RViz2${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""

# =============================================================================
# 1. System dependency check
# =============================================================================
check_cmd() {
    if command -v "$1" &>/dev/null; then
        echo -e "  ${GREEN}[OK]${NC} $1"
        return 0
    else
        echo -e "  ${RED}[MISSING]${NC} $1"
        return 1
    fi
}

echo -e "${YELLOW}[1/6] Checking system dependencies...${NC}"
MISSING=0

check_cmd "ros2" || MISSING=1
check_cmd "gz" || MISSING=1
check_cmd "python3" || MISSING=1
check_cmd "colcon" || MISSING=1
check_cmd "git" || MISSING=1

if dpkg -l 2>/dev/null | grep -q "ros-jazzy-desktop"; then
    echo -e "  ${GREEN}[OK]${NC} ros-jazzy-desktop"
else
    echo -e "  ${YELLOW}[WARN]${NC} ros-jazzy-desktop not found. Install: sudo apt install ros-jazzy-desktop"
fi

if dpkg -l 2>/dev/null | grep -q "gz-harmonic"; then
    echo -e "  ${GREEN}[OK]${NC} gz-harmonic"
else
    echo -e "  ${YELLOW}[WARN]${NC} gz-harmonic not found. Install: sudo apt install gz-harmonic"
fi

if [ "$MISSING" -eq 1 ]; then
    echo ""
    echo -e "${RED}Missing dependencies detected. Please install and re-run.${NC}"
    echo ""
    echo "Quick install commands:"
    echo "  sudo apt update"
    echo "  sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions"
    echo "  sudo apt install -y gz-harmonic"
    exit 1
fi

# =============================================================================
# 2. Install ROS 2 package dependencies
# =============================================================================
echo ""
echo -e "${YELLOW}[2/6] Installing ROS 2 dependencies...${NC}"
sudo apt install -y \
    ros-jazzy-ros-gzharmonic \
    ros-jazzy-ros-gzharmonic-bridge \
    ros-jazzy-ros-gzharmonic-sim \
    ros-jazzy-ros-gzharmonic-image \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-common \
    ros-jazzy-joint-state-publisher \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    2>/dev/null || echo -e "${YELLOW}Some packages may have failed — continuing...${NC}"

# =============================================================================
# 3. Install Micro XRCE-DDS Agent
# =============================================================================
echo ""
echo -e "${YELLOW}[3/6] Installing Micro XRCE-DDS Agent...${NC}"
if ! command -v MicroXRCEAgent &>/dev/null; then
    echo "Installing MicroXRCEAgent via snap..."
    sudo snap install micro-xrce-dds-agent --edge 2>/dev/null || {
        echo -e "${YELLOW}Snap install failed. Trying apt...${NC}"
        sudo apt install -y ros-jazzy-micro-ros-agent 2>/dev/null || {
            echo -e "${YELLOW}Building from source...${NC}"
            AGENT_DIR="/tmp/Micro-XRCE-DDS-Agent"
            if [ ! -d "$AGENT_DIR" ]; then
                git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
            fi
            cd "$AGENT_DIR"
            mkdir -p build && cd build
            cmake .. -DCMAKE_BUILD_TYPE=Release
            make -j$(nproc)
            sudo make install
            sudo ldconfig
        }
    }
    echo -e "  ${GREEN}[OK]${NC} MicroXRCEAgent installed"
else
    echo -e "  ${GREEN}[OK]${NC} MicroXRCEAgent already installed"
fi

# =============================================================================
# 4. Clone PX4-Autopilot (if needed)
# =============================================================================
echo ""
echo -e "${YELLOW}[4/6] Setting up PX4-Autopilot...${NC}"
PX4_DIR="$HOME/PX4-Autopilot"
if [ ! -d "$PX4_DIR" ]; then
    echo "Cloning PX4-Autopilot (this may take a while)..."
    git clone --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" || {
        echo "Shallow clone failed, trying full clone..."
        git clone --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
    }
    echo "Running PX4 Ubuntu setup script..."
    cd "$PX4_DIR"
    bash Tools/setup/ubuntu.sh --no-sim-tools 2>/dev/null || {
        echo -e "${YELLOW}PX4 setup had warnings — this is usually OK.${NC}"
    }
    echo -e "  ${GREEN}[OK]${NC} PX4-Autopilot cloned"
else
    echo -e "  ${GREEN}[OK]${NC} PX4-Autopilot found at $PX4_DIR"
fi

# =============================================================================
# 5. Clone px4_msgs and build workspace
# =============================================================================
echo ""
echo -e "${YELLOW}[5/6] Cloning px4_msgs...${NC}"
cd "$WS_DIR"
if [ ! -d "src/px4_msgs" ]; then
    git clone --depth 1 https://github.com/PX4/px4_msgs.git src/px4_msgs
    echo -e "  ${GREEN}[OK]${NC} px4_msgs cloned"
else
    echo -e "  ${GREEN}[OK]${NC} px4_msgs already exists"
fi

# =============================================================================
# 6. Build the workspace
# =============================================================================
echo ""
echo -e "${YELLOW}[6/6] Building ROS 2 workspace...${NC}"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
cd "$WS_DIR"

# Install any missing rosdep dependencies
sudo rosdep init 2>/dev/null || true
rosdep update 2>/dev/null || true
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || echo -e "${YELLOW}rosdep had warnings — continuing...${NC}"

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || {
    echo ""
    echo -e "${RED}Build failed! Troubleshooting:${NC}"
    echo "  1. Make sure px4_msgs compiled first: colcon build --packages-select px4_msgs"
    echo "  2. Source ROS 2: source /opt/ros/jazzy/setup.bash"
    echo "  3. Check missing deps: rosdep check --from-paths src"
    exit 1
}

echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  Setup Complete!${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo ""
echo "  1. Build PX4 SITL (one-time, ~10 min):"
echo "     cd ~/PX4-Autopilot"
echo "     make px4_sitl gz_x500"
echo ""
echo "  2. Add this to your ~/.bashrc:"
echo "     source /opt/ros/jazzy/setup.bash"
echo "     source ${WS_DIR}/install/setup.bash"
echo ""
echo "  3. Set PX4 environment variables:"
echo "     export PX4_AUTOPILOT=\$HOME/PX4-Autopilot"
echo "     export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$PX4_AUTOPILOT/Tools/simulation/gz/models"
echo "     export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:\$PX4_AUTOPILOT/build/px4_sitl_default/src/modules/simulation/gz_plugins"
echo ""
echo "  4. Run simulation (two terminals):"
echo "     Terminal 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500"
echo "     Terminal 2: ros2 launch drone_bringup simulation.launch.py"
echo ""
echo "  Or use the one-command launcher:"
echo "     ./run_simulation.sh"
echo ""
