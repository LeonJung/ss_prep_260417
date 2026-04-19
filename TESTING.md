# Testing guide — DG5F · Manus teleop

Three tiers of tests, ordered by required hardware.

## 0. One-line bootstrap (first time)

```bash
cd ~/hand_ws/src
bash setup.sh           # apt + rosdep + vcs import + colcon build
source ~/hand_ws/install/setup.bash
```

See `setup.sh` header for flags (`ROS_DISTRO`, `WITH_GZ`, `SKIP_APT`, …).

---

## Tier 1 — No hardware (fully offline)

Every pipeline component verified without a DG gripper or Manus glove.

```bash
source ~/hand_ws/install/setup.bash

# (1) retarget math — unit tests
python3 ~/hand_ws/src/manus_glove/manus_dg5f_retarget/test/unit_formula_test.py

# (2) DG sim — grip_cycle from fake driver -> contact_viz pipeline
python3 ~/hand_ws/src/dg/dg5f_hand_bringup/test/e2e_sim_test.py

# (3) Manus teleop fixed-mode E2E — index MCP drives rj_dg_2_2
python3 ~/hand_ws/src/manus_glove/manus_dg5f_retarget/test/e2e_teleop_test.py

# (4) Manus teleop coupled-mode E2E — thumb CMC clamps to URDF limits
python3 ~/hand_ws/src/manus_glove/manus_dg5f_retarget/test/e2e_coupled_test.py
```

All four should print `OK:` / `ALL UNIT TESTS PASSED`.

### Interactive GUI sanity (no DG / no glove)

```bash
# Terminal 1: Qt slider (publishes fake /manus_glove_1)
ros2 run manus_glove_sim sim_glove

# Terminal 2: retargeter (fixed thumb CMC)
ros2 run manus_dg5f_retarget retarget_node --ros-args \
  --params-file ~/hand_ws/src/manus_glove/manus_dg5f_retarget/config/right_hand_calib.yaml

# Terminal 3: watch the command topic
ros2 topic echo /dg5f_right/rj_dg_pospid/reference
```

Drag a slider → corresponding `values[i]` in the echo changes.

---

## Tier 2 — DG5F real hardware · Manus **simulated** (Qt sliders)

Use the Qt slider UI as the leader; the DG5F right hand follows. Good first
test with the real gripper — no human-in-the-loop mistakes.

### 2.1  Physical prep

1. Power on DG5F right hand.
2. Wire the PC NIC to the gripper (direct or via switch).
3. Set a static IPv4 on that NIC, e.g. `169.254.186.10/24`.
4. Verify: `ping 169.254.186.72` (default gripper IP).

### 2.2  Run (4 terminals, all after `source install/setup.bash`)

```bash
# T1 — DG real driver with PID-all controller + FT broadcasters
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py \
    delto_ip:=169.254.186.72 delto_port:=502 \
    fingertip_sensor:=true ft_broadcaster:=true

# T2 — Manus slider (fake /manus_glove_1 at 30 Hz)
ros2 run manus_glove_sim sim_glove

# T3 — retarget bridge
ros2 run manus_dg5f_retarget retarget_node --ros-args \
    --params-file ~/hand_ws/src/manus_glove/manus_dg5f_retarget/config/right_hand_calib.yaml \
    -p thumb_cmc_mode:=fixed

# T4 — contact / current visualizer (optional but recommended)
ros2 launch dg5f_contact_viz contact_viz.launch.py
```

### 2.3  What to check

| Slider motion                         | Expected DG behaviour                                       |
|---------------------------------------|-------------------------------------------------------------|
| `IndexMCPStretch` 0→70°               | index finger base joint (`rj_dg_2_2`) closes smoothly        |
| `MiddleMCPStretch` 0→70°              | middle finger closes, no other finger moves                  |
| `Fist` preset button                   | all four non-thumb fingers close together                    |
| `Pinch` preset                         | thumb + index approach each other                           |
| `Open` preset                          | all fingers return to neutral                               |
| Finger pressed against a soft object  | `contact_viz` cell turns red on that finger's distal joint  |

### 2.4  Safety

- Start with `thumb_cmc_mode:=fixed` — keeps thumb base at 0 rad.
- If a finger twitches before T2/T3 publish anything, **stop T1 first**
  (cancel `ros2 launch`) then unplug the gripper and check current IP /
  URDF xacro args.
- URDF joint limits clamp every command; if the gripper still hits its
  hard stop, reduce `calib[i]` in `right_hand_calib.yaml` for that joint.

---

## Tier 3 — DG5F real · Manus glove real

Replace the stub `manus_ros2_msgs` with the official package from the Manus
SDK, then swap T2 of Tier 2 for the real Manus driver.

### 3.1  Replace the stub with the real Manus SDK

```bash
# Backup / remove the stub (only interfaces we use; official SDK has more)
rm -rf ~/hand_ws/src/manus_glove/manus_ros2_msgs

# Drop in the SDK-shipped packages (path depends on your Manus download)
cp -r /path/to/manus-sdk/ros2/manus_ros2_msgs ~/hand_ws/src/manus_glove/
cp -r /path/to/manus-sdk/ros2/manus_ros2      ~/hand_ws/src/manus_glove/   # driver

cd ~/hand_ws && colcon build --symlink-install
source install/setup.bash
```

### 3.2  Run (replaces T2 only)

```bash
# T1 — DG real driver (same as Tier 2)
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py \
    delto_ip:=169.254.186.72 delto_port:=502 \
    fingertip_sensor:=true ft_broadcaster:=true

# T2 — Manus real driver (publishes /manus_glove_1 at ~120 Hz)
ros2 run manus_ros2 manus_data_publisher        # SDK-provided executable

# T3 — retarget (same as Tier 2)
ros2 run manus_dg5f_retarget retarget_node --ros-args \
    --params-file ~/hand_ws/src/manus_glove/manus_dg5f_retarget/config/right_hand_calib.yaml \
    -p thumb_cmc_mode:=fixed

# T4 — contact viz
ros2 launch dg5f_contact_viz contact_viz.launch.py
```

### 3.3  Calibration pass (glove-on-wearer)

Wear the glove, then with DG powered but idle:

1. Keep hand relaxed → check `ros2 topic echo /manus_glove_1 --once` shows
   all ergonomics near 0°. If a finger reads 30°+ at rest, re-run Manus
   calibration in their tool.
2. Make a fist slowly → DG should close in the same phase. If a finger
   over/under-closes, edit `calib[i]` for that joint in `right_hand_calib.yaml`
   and `ros2 launch`-relaunch T3.
3. Spread fingers → watch the `<Finger>Spread` joints (`rj_dg_2_1`,
   `rj_dg_3_1`, …) on DG. Sign flips are fixed via `dir_sign[i]`.
4. For the thumb CMC axis (`rj_dg_1_1`):
   - Stay on `thumb_cmc_mode:=fixed` first → thumb base stays at 0.
   - Then try `:=coupled` with default `offset_deg:=58.5`. Thumb should now
     swing to follow your thumb-in/thumb-out motion. Tune `gain_stretch`
     and `gain_spread` in the yaml.

### 3.4  Latency sanity

```bash
ros2 topic hz /manus_glove_1                       # should be ~120 Hz
ros2 topic hz /dg5f_right/rj_dg_pospid/reference   # should match
ros2 topic delay /dg5f_right/joint_states          # DG feedback freshness
```

Glove → DG command round-trip is typically < 20 ms on a local net. DG
mechanical response adds ~40–80 ms on top.

---

## Quick troubleshooting

| Symptom                                              | Likely cause / fix                                      |
|------------------------------------------------------|---------------------------------------------------------|
| `Scouting delay elapsed` warning, no topic data      | Zenoh router not running. `ros2 run rmw_zenoh_cpp rmw_zenohd &` |
| `manus_ros2_msgs not found`                          | Stub deleted without SDK replacement; re-run `setup.sh` |
| DG fingers twitch at launch                          | Stop T1 first, check `delto_ip`, verify URDF side arg   |
| `rj_dg_1_1` hits hard stop                           | Switch to `thumb_cmc_mode:=fixed`; re-tune coupled gain |
| One finger closes backwards                          | Flip its sign in `right_hand_calib.yaml → dir_sign[i]`  |
| contact_viz stays green under heavy grip             | Increase `contact_high_mA` or lower `contact_low_mA` in `contact_thresholds.yaml` |
