#!/usr/bin/env bash
# One-line bootstrap wrapper for the DG5F + Manus workspace.
#
# Usage:
#   cd ~/hand_ws/src && bash setup.sh                 # full bootstrap (jazzy, skip gz/moveit)
#   ROS_DISTRO=humble bash setup.sh                   # build/deploy on Humble
#   WITH_GZ=1 bash setup.sh                           # + Gazebo
#   WITH_MOVEIT=1 bash setup.sh                       # + MoveIt 2
#   SKIP_APT=1 bash setup.sh                          # skip apt (deps already present)
#
# Delegates to dg5f_hand_bringup/scripts/install.sh which handles:
#   1) apt (ros2_controllers, xacro, rmw_fastrtps, PyQt5, colcon, vcstool, rosdep, git)
#   2) rosdep init + update + install
#   3) vcs import of upstream tesollo repos into src/dg/
#   4) Manus SDK auto-detection (skips manus_* packages if manus_ros2_msgs missing)
#   5) colcon build --symlink-install

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_DEFAULT="$(cd "$HERE/.." && pwd)"
: "${WS:=$WS_DEFAULT}"
export WS

INSTALL="$HERE/dg/dg5f_hand_bringup/scripts/install.sh"
if [[ ! -f "$INSTALL" ]]; then
  echo "ERROR: $INSTALL not found."
  echo "Are you running this from ~/hand_ws/src after cloning dg5f_hand_bringup?"
  exit 1
fi

echo "== bootstrapping workspace at $WS =="
exec bash "$INSTALL" "$@"
