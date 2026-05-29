# dg5f_finger_diag

Per-joint self-diagnostic for the DG5F hand. Drives each of the 20 joints
with a small step from the current pose, measures position response and
peak effort, and prints a PASS / WARN / FAIL summary per joint so flaky
fingers are easy to identify.

## Setup

Run the **driver + controller stack** in one terminal, NOT a teleop
launch (teleop publishes to the same `*/lj_dg_pospid/reference` topic and
would race with the diag):

```
# Left
ros2 launch dg5f_driver dg5f_left_pid_all_controller.launch.py
# Right
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py
```

## Run

```
# Step test (default): per-joint baseline -> +0.30 rad -> baseline
ros2 run dg5f_finger_diag finger_diag --ros-args -p hand_side:=left

# Passive: just records joint_states for 3 s, no motion
ros2 run dg5f_finger_diag finger_diag --ros-args \
    -p hand_side:=left -p mode:=passive
```

Override any parameter the same way (e.g. `-p amplitude_rad:=0.2`,
`-p settle_time_s:=2.0`, `-p step_sign:='[1,1,-1,-1, ...20 values]'`).

## Output

The step report is a 20-row table:

```
#  fng joint            base     cmd   reach     err   motion    peakE  verdict
0  a   lj_dg_1_1     +0.000  +0.300  +0.298   0.002  +0.298     3.10  PASS
...
17 e   lj_dg_5_2     +0.410  +0.710  +0.450   0.260  +0.040    18.50  FAIL_POOR_TRACK
```

Verdicts:

- `PASS` — tracked within `pos_error_warn` (default 0.05 rad)
- `WARN_TRACKING` — between warn and `pos_error_fail` (0.15 rad)
- `WARN_HIGH_EFFORT` — tracked OK but peak |effort| > `peak_eff_high`
- `FAIL_STUCK` — moved less than `motion_stuck_thresh` (0.03 rad)
- `FAIL_REVERSED` — moved the wrong direction
- `FAIL_POOR_TRACK` — settled, but more than `pos_error_fail` from commanded

## Safety

- Amplitude defaults to 0.30 rad (~17°) from current pose, clamped to
  ±`max_abs_command` (1.0 rad). Each joint is tested alone with the other
  19 held at baseline.
- The node returns each joint to baseline before testing the next, and
  briefly holds at baseline at the end. The hardware `on_deactivate`
  (zero-duty release) takes the hand limp when the controller is then
  stopped.
- Hit Ctrl-C any time; the current command frame is the last one
  published, so motors hold position until the controller is stopped.

## Parameters

| name | default | purpose |
|---|---|---|
| `hand_side` | `left` | `left` or `right` |
| `mode` | `step` | `step` (default) or `passive` |
| `amplitude_rad` | `0.30` | per-joint step from baseline |
| `max_abs_command` | `1.0` | absolute clamp on commanded value |
| `settle_time_s` | `1.5` | hold time at step target |
| `recover_time_s` | `0.5` | hold time at baseline between joints |
| `collect_time_s` | `3.0` | passive-mode collection window |
| `publish_rate_hz` | `50.0` | reference publish rate |
| `step_sign` | `[1]*20` | per-joint step direction (+1 / -1) |
| `pos_error_warn` | `0.05` | WARN_TRACKING threshold |
| `pos_error_fail` | `0.15` | FAIL_POOR_TRACK threshold |
| `motion_stuck_thresh` | `0.03` | FAIL_STUCK threshold |
| `peak_eff_high` | `30.0` | WARN_HIGH_EFFORT threshold on \|effort\| |
