# Manus ROS2 SDK setup

The packages under `src/manus_glove/` (both `manus_dg5f_retarget` and
`manus_glove_sim`) depend on `manus_ros2_msgs`, which ships with the
**commercial Manus SDK** — it is not in any public apt repo.

## Install (per dev/robot machine)

1. Register/download the Manus SDK ROS2 package from Manus support:
   <https://docs.manus-meta.com/3.1.0/Plugins/SDK/ROS2/>
2. Extract the ROS2 subpackages into this workspace, e.g.:
   ```
   ~/hand_ws/src/manus_glove/manus_ros2_msgs/
   ~/hand_ws/src/manus_glove/manus_ros2/           # optional driver
   ```
3. Rebuild:
   ```
   cd ~/hand_ws
   colcon build --symlink-install
   source install/setup.bash
   ```
4. Verify:
   ```
   ros2 pkg list | grep manus_ros2_msgs
   ros2 interface show manus_ros2_msgs/msg/ManusGlove
   ```

## Without the SDK

`install.sh` auto-detects the absence of `manus_ros2_msgs` and passes
`--packages-skip manus_dg5f_retarget manus_glove_sim` to colcon so that
the DG-only stack still builds. Our slider UI / retargeter can only be
exercised once the SDK is present.

## Runtime topics we assume

| Direction | Topic                                 | Type                           | Rate  |
|-----------|---------------------------------------|--------------------------------|-------|
| Manus ->  | `/manus_glove_1`                      | `manus_ros2_msgs/ManusGlove`   | 120 Hz (real), 30 Hz (sim) |
| Retarget -> | `/dg5f_right/rj_dg_pospid/reference` | `control_msgs/MultiDOFCommand` | same as input |

The right-hand glove id defaults to `1` (Manus SDK assigns ids by connection order);
override with `expected_side:=any` in the retargeter yaml if your setup differs.
