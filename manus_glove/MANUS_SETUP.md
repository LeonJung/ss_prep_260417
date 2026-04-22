# Manus ROS2 setup & tuning

The packages under `src/manus_glove/` (`manus_dg5f_retarget`, `manus_glove_sim`)
depend on `manus_ros2_msgs`, which ships with the **commercial Manus SDK** —
it is not in any public apt repo.

Until the real SDK is installed, this workspace includes a stub copy of
`manus_ros2_msgs` under `src/manus_glove/manus_ros2_msgs/` whose .msg files
mirror the public Manus docs. That stub is enough to build and run the sim;
replace it with the SDK-shipped version before doing real-hand teleop so
field-level compatibility is guaranteed.

---

## 1. Install the real SDK on the control PC

1. Grab the Manus Core SDK (ROS2 variant) from Manus Support:
   <https://docs.manus-meta.com/latest/Plugins/SDK/> → "Getting started" for ROS2.
   Typical contents include `manus_ros2` (driver) and `manus_ros2_msgs` (messages).

2. Drop the SDK's ROS2 packages into this workspace and replace the stub:
   ```
   rm -rf ~/hand_ws/src/manus_glove/manus_ros2_msgs   # remove the stub
   cp -r  <manus-sdk>/ros2/manus_ros2_msgs ~/hand_ws/src/manus_glove/
   cp -r  <manus-sdk>/ros2/manus_ros2      ~/hand_ws/src/manus_glove/
   ```

3. Rebuild:
   ```
   cd ~/hand_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Verify:
   ```
   ros2 pkg list | grep manus_ros2
   ros2 interface show manus_ros2_msgs/msg/ManusGlove
   ```

5. (Usually done already) Linux USB permissions for the Manus dongle are
   provided via a udev rule from the SDK. If the driver later complains it
   can't open the device, reinstall the SDK's udev rule and unplug/replug.

### Building without the SDK
`scripts/install.sh` detects the absence of `manus_ros2_msgs` and appends
`--packages-skip manus_dg5f_retarget manus_glove_sim` to colcon, so the
DG-only stack still builds on a dev machine that has no SDK.

---

## 2. Wear & calibrate the glove (Manus Core app)

Manus gloves need per-user calibration before the ergonomics stream makes
sense. The SDK does NOT auto-calibrate; this is done once per session in
the Manus Core desktop app.

Steps (paraphrased from the Manus user guide):
1. Launch **Manus Core** (the desktop app shipped with the SDK).
2. Connect the dongle and put the glove on the correct hand (right here).
3. In Manus Core, go to the calibration wizard for "Metagloves" (or "Quantum"
   depending on product).
4. Follow the prompted hand poses:
   * Flat hand, palm down
   * Fist
   * Thumb-up / thumb-to-pinky
   * Spread fingers
5. Save the calibration profile. Subsequent SDK sessions read this profile
   automatically; you only redo it after doffing/donning or if motion feels
   drifted.

After calibration, verify in Manus Core's live preview that the virtual
hand mirrors your real hand. If a finger looks stuck or inverted, redo
calibration.

---

## 3. Topics exchanged

| Direction       | Topic                                   | Type                           | Rate  |
|-----------------|-----------------------------------------|--------------------------------|-------|
| Manus driver -> | `/manus_glove_1` (right hand)           | `manus_ros2_msgs/ManusGlove`   | 120 Hz real · 30 Hz sim |
| Retarget ->     | `/dg5f_right/rj_dg_pospid/reference`    | `control_msgs/MultiDOFCommand` | same as input |

The right-hand glove defaults to id `1` (Manus assigns ids in connection
order). If your first connected glove ends up as `0` instead, either
reconnect the right glove first or switch the input topic via launch arg.

---

## 4. Running the teleop

### Right hand (glove id 1, IP 169.254.186.72)

Real Manus glove + real DG5F:
```
ros2 launch manus_dg5f_retarget manus_teleop_right.launch.py \
     manus_source:=real use_mock_hardware:=false
```

Real DG5F but with the Qt slider sim instead of the glove:
```
ros2 launch manus_dg5f_retarget manus_teleop_right.launch.py \
     manus_source:=sim use_mock_hardware:=false
```

### Left hand (glove id 0, IP 169.254.186.73)

```
ros2 launch manus_dg5f_retarget manus_teleop_left.launch.py \
     manus_source:=real use_mock_hardware:=false
```
The left launch uses `left_hand_calib.yaml`, subscribes to
`/manus_glove_0`, publishes to `/dg5f_left/lj_dg_pospid/reference`,
and includes the dg5f_left_pid_all_controller (IP 169.254.186.73).

### Bimanual (both hands)

If you want both launches up at once, **only one of them should launch
the real Manus driver** — the driver binds the USB dongle, so running
two is a conflict. Start the driver separately and use
`manus_source:=external` on both launches:

```
# terminal A — shared Manus driver (publishes both /manus_glove_0 and /manus_glove_1)
ros2 run manus_ros2 manus_data_publisher
# terminal B — right teleop
ros2 launch manus_dg5f_retarget manus_teleop_right.launch.py \
     manus_source:=external use_mock_hardware:=false
# terminal C — left teleop
ros2 launch manus_dg5f_retarget manus_teleop_left.launch.py \
     manus_source:=external use_mock_hardware:=false
```

### Terminal-per-piece layout

If you prefer to run the Manus driver and DG driver in their own
terminals:
```
# terminal A
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py \
     delto_ip:=169.254.186.72
# terminal B
ros2 run manus_ros2 manus_data_publisher    # or our sim_glove
# terminal C
ros2 launch manus_dg5f_retarget retarget_only.launch.py hand_side:=right
```
Replace `right` with `left` on terminals A and C for the left hand.

Optional contact visualizer (right hand only for now):
```
ros2 launch dg5f_contact_viz contact_viz.launch.py
```

---

## 5. Tuning the retargeter

Each hand has its own yaml, auto-loaded by the matching launch:

```
~/hand_ws/src/manus_glove/manus_dg5f_retarget/config/right_hand_calib.yaml
~/hand_ws/src/manus_glove/manus_dg5f_retarget/config/left_hand_calib.yaml
```

The launches above auto-load this file; you never pass `--params-file`
by hand. Edit the yaml, relaunch, done. With `--symlink-install` there
is no need to rebuild after an edit — the installed copy is a symlink
back to the source file.

Key fields:
| Field | Purpose |
|-------|---------|
| `thumb_cmc_mode`           | `fixed` · `coupled` · `raw_nodes_ik`. See thumb_cmc.py in this package. |
| `thumb_cmc_fixed_value_rad`| Held thumb base angle when mode is `fixed`. |
| `thumb_cmc_offset_deg`, `thumb_cmc_gain_stretch`, `thumb_cmc_gain_spread` | Linear blend for `coupled` mode. |
| `calib[20]`                | Per-joint magnitude scale. Increase if the DG under-moves for that joint. |
| `dir_sign[20]`             | Per-joint sign. Flip to `-1.0` if a joint moves the wrong way. |
| `clamp_to_urdf_limits`     | Clamp outputs to the hardware joint limits (keep `true` for safety). |
| `apply_postprocess`        | Zero out the "wrong-direction" bends originally present in the Tesollo reference retarget. |

Override `thumb_cmc_mode` at launch time without editing the yaml:
```
ros2 launch manus_dg5f_retarget manus_teleop_right.launch.py \
     thumb_cmc_mode:=coupled
```

---

## 6. First-touch calibration checklist (real glove + real DG)

1. Manus Core calibration done, virtual hand mirrors yours in the app.
2. `ros2 pkg list | grep manus_ros2` shows `manus_ros2_msgs` **and** `manus_ros2`.
3. `ros2 topic hz /manus_glove_1` reports ~120 Hz after the real driver is up.
4. Start the teleop with `use_mock_hardware:=true` first. Watch the joint
   names + values on `/dg5f_right/rj_dg_pospid/reference` before trusting
   the real hand.
5. Switch to `use_mock_hardware:=false` only once signs and scales look
   reasonable in mock. Tune `calib`, `dir_sign`, and `thumb_cmc_*` from
   the yaml iteratively.
