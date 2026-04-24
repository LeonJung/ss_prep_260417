#!/usr/bin/env bash
# Bootstrap the DG5F right-hand workspace.
#
# Installs system packages, (optionally) clones the tesollo upstream repos
# via vcstool, resolves remaining rosdep keys, and runs colcon build.
#
# Usage:
#   bash install.sh                     # full bootstrap (default: jazzy, skip gz/moveit)
#   WS=$HOME/hand_ws bash install.sh    # override workspace root
#   ROS_DISTRO=humble bash install.sh   # override distro
#   WITH_GZ=1 bash install.sh           # also build dg5f_gz (Gazebo)
#   WITH_MOVEIT=1 bash install.sh       # also build dg5f_moveit_config
#   SKIP_APT=1 bash install.sh          # skip apt (if deps already handled)
#   SKIP_FETCH=1 bash install.sh        # skip vcs import
#   SKIP_BUILD=1 bash install.sh        # skip colcon build

set -euo pipefail

: "${WS:=$HOME/hand_ws}"
: "${ROS_DISTRO:=jazzy}"
: "${SKIP_APT:=0}"
: "${SKIP_FETCH:=0}"
: "${SKIP_BUILD:=0}"
: "${WITH_GZ:=0}"
: "${WITH_MOVEIT:=0}"

SUDO=sudo
[[ ${EUID:-$(id -u)} -eq 0 ]] && SUDO=

echo "== DG5F bootstrap =="
echo "   WS=$WS"
echo "   ROS_DISTRO=$ROS_DISTRO"

if [[ ! -d "/opt/ros/$ROS_DISTRO" ]]; then
  echo "ERROR: /opt/ros/$ROS_DISTRO not found. Install ROS 2 $ROS_DISTRO first."
  exit 1
fi

# --- 1. System packages ------------------------------------------------------
if [[ "$SKIP_APT" != "1" ]]; then
  echo "-- apt install system deps --"
  APT_PKGS=(
    "ros-$ROS_DISTRO-ros2-controllers"        # meta: joint_trajectory / effort / ft_broadcaster
    "ros-$ROS_DISTRO-controller-manager"
    "ros-$ROS_DISTRO-joint-state-broadcaster"
    "ros-$ROS_DISTRO-robot-state-publisher"
    "ros-$ROS_DISTRO-xacro"
    "ros-$ROS_DISTRO-rmw-fastrtps-cpp"        # fallback RMW (Humble default, Jazzy alt)
    python3-pyqt5
    python3-empy                              # URDF/xacro templating (ROS build-time)
    python3-lark                              # rosidl parser (ROS build-time)
    python3-numpy                             # SOTA retargeters (FK/IK matrices)
    python3-scipy                             # manus_dg5f_sota_retarget_a SLSQP
    python3-colcon-common-extensions
    python3-vcstool
    python3-rosdep
    git
  )
  if [[ "$WITH_GZ" == "1" ]]; then
    APT_PKGS+=("ros-$ROS_DISTRO-gz-ros2-control" "ros-$ROS_DISTRO-ros-gz")
  fi
  if [[ "$WITH_MOVEIT" == "1" ]]; then
    APT_PKGS+=("ros-$ROS_DISTRO-moveit")
  fi
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends "${APT_PKGS[@]}"
fi

# --- 2. rosdep init + update (idempotent) -----------------------------------
if ! command -v rosdep >/dev/null 2>&1; then
  echo "ERROR: rosdep not available after apt install."
  exit 1
fi
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "-- rosdep init --"
  $SUDO rosdep init
fi
echo "-- rosdep update --"
rosdep update

# --- 3. Fetch upstream sources ----------------------------------------------
mkdir -p "$WS/src/dg" "$WS/src/manus_glove"
REPOS_FILE="$WS/src/dg/dg5f_hand.repos"
if [[ "$SKIP_FETCH" != "1" ]]; then
  if [[ ! -f "$REPOS_FILE" ]]; then
    echo "ERROR: $REPOS_FILE not found. Make sure dg5f_hand_bringup is in $WS/src/dg/."
    exit 1
  fi
  echo "-- vcs import < $REPOS_FILE (into src/dg/) --"
  (cd "$WS/src/dg" && vcs import < "$REPOS_FILE")
fi

# --- 4. rosdep install (package.xml -> apt) ---------------------------------
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
echo "-- rosdep install from sources --"
rosdep install --from-paths "$WS/src" --ignore-src -r -y \
  --rosdistro "$ROS_DISTRO" || true

# --- 5. Manus SDK detection -------------------------------------------------
# The manus_* packages depend on manus_ros2_msgs, which ships only with the
# commercial Manus SDK (see src/manus_glove/MANUS_SETUP.md). If the msg
# package is not reachable, skip our manus packages from the build.
HAS_MANUS=0
if ros2 pkg list 2>/dev/null | grep -q '^manus_ros2_msgs$'; then
  HAS_MANUS=1
elif [[ -d "$WS/src/manus_glove/manus_ros2_msgs" ]]; then
  HAS_MANUS=1
fi

# --- 6. colcon build --------------------------------------------------------
if [[ "$SKIP_BUILD" != "1" ]]; then
  SKIP_PKGS=()
  [[ "$WITH_GZ"     != "1" ]] && SKIP_PKGS+=("dg5f_gz")
  [[ "$WITH_MOVEIT" != "1" ]] && SKIP_PKGS+=("dg5f_moveit_config")
  if [[ "$HAS_MANUS" != "1" ]]; then
    echo "-- manus_ros2_msgs not found; skipping manus_* packages --"
    echo "   (install Manus SDK per src/manus_glove/MANUS_SETUP.md to enable)"
    SKIP_PKGS+=("manus_dg5f_retarget" "manus_glove_sim")
  fi

  echo "-- colcon build --"
  (
    cd "$WS"
    if (( ${#SKIP_PKGS[@]} )); then
      colcon build --symlink-install --packages-skip "${SKIP_PKGS[@]}"
    else
      colcon build --symlink-install
    fi
  )
fi

cat <<EOF

DONE.

Next steps:
  source $WS/install/setup.bash

Pre-hardware simulation (no DG5F needed):
  ros2 launch dg5f_hand_bringup dg5f_right_sim.launch.py scenario:=grip_cycle

Headless regression test:
  python3 $WS/src/dg/dg5f_hand_bringup/test/e2e_sim_test.py

Mock ros2_control (requires ros2_controllers, now installed):
  ros2 launch dg5f_driver dg5f_right_mock.launch.py

Manus -> DG teleop (requires Manus SDK, see src/manus_glove/MANUS_SETUP.md):
  ros2 launch manus_dg5f_retarget manus_teleop_right.launch.py

Real DG5F hardware (right hand, Modbus TCP):
  ros2 launch dg5f_hand_bringup dg5f_right_bringup.launch.py \\
      use_mock_hardware:=false delto_ip:=169.254.186.72 ft_broadcaster:=true
EOF
