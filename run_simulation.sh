#!/bin/bash
# =============================================================================
# One-command launcher: opens tmux windows for PX4 SITL + ROS 2 stack.
# Requires: tmux
#
# Usage:
#   chmod +x run_simulation.sh
#   ./run_simulation.sh
#   ./run_simulation.sh --with-offboard    # Also start offboard control
# =============================================================================

SESSION="drone_sim"
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${PX4_AUTOPILOT:-$HOME/PX4-Autopilot}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Starting Drone Simulation...${NC}"

# Check prerequisites
if [ ! -d "$PX4_DIR" ]; then
    echo -e "${RED}PX4-Autopilot not found at $PX4_DIR${NC}"
    echo "Run ./setup.sh first or set PX4_AUTOPILOT env var."
    exit 1
fi

if ! command -v tmux &>/dev/null; then
    echo -e "${RED}tmux is required. Install: sudo apt install tmux${NC}"
    exit 1
fi

# Kill existing session if any
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "Creating tmux session '$SESSION'..."

# Window 0: PX4 SITL (with Gazebo)
tmux new-session -d -s "$SESSION" -n "PX4-SITL"
tmux send-keys -t "$SESSION:PX4-SITL" \
    "cd $PX4_DIR && echo '=== PX4 SITL + Gazebo Harmonic ===' && make px4_sitl gz_x500" C-m

# Window 1: ROS 2 stack
tmux new-window -t "$SESSION" -n "ROS2"
tmux send-keys -t "$SESSION:ROS2" \
    "echo 'Waiting for PX4 to start (20s)...' && sleep 20 && \
     source /opt/ros/jazzy/setup.bash && \
     source $WS_DIR/install/setup.bash && \
     echo '=== Launching ROS 2 Simulation Stack ===' && \
     ros2 launch drone_bringup simulation.launch.py" C-m

# Window 2: Optional offboard control
WITH_OFFBOARD=false
if [ "$1" = "--with-offboard" ]; then
    WITH_OFFBOARD=true
    tmux new-window -t "$SESSION" -n "Offboard"
    tmux send-keys -t "$SESSION:Offboard" \
        "echo 'Waiting for PX4 topics (30s)...' && sleep 30 && \
         source /opt/ros/jazzy/setup.bash && \
         source $WS_DIR/install/setup.bash && \
         echo '=== Starting Offboard Control ===' && \
         ros2 launch drone_control offboard.launch.py" C-m
fi

# Window: Monitor (topics / TF)
tmux new-window -t "$SESSION" -n "Monitor"
tmux send-keys -t "$SESSION:Monitor" \
    "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
     source $WS_DIR/install/setup.bash 2>/dev/null; \
     echo 'Useful commands:' && \
     echo '  ros2 topic list | grep fmu' && \
     echo '  ros2 topic echo /fmu/out/vehicle_status' && \
     echo '  ros2 run tf2_tools view_frames' && \
     echo '  ros2 topic hz /fmu/out/vehicle_odometry'" C-m

# Attach to session
tmux select-window -t "$SESSION:ROS2"
tmux attach -t "$SESSION"

echo ""
echo -e "${GREEN}Simulation ended.${NC}"
echo "Clean up tmux sessions with: tmux kill-session -t $SESSION"
