"""DG5F per-joint self-diagnostic.

Drives each of the 20 joints from the current pose with a small step, holds,
measures position response + peak effort, then returns to baseline before
moving on. Prints a PASS/WARN/FAIL summary so misbehaving fingers can be
identified before teleop.

Modes:
  passive : record joint_states for `collect_time_s` seconds, report per-joint
            mean/std of position, mean velocity, mean+peak effort. No commands
            are published; safe to run any time the controller is up.
  step    : per-joint step test (default). For each of 20 joints, commands
            baseline + sign*amplitude_rad (clamped to ±max_abs_command),
            holds for settle_time_s, records reached position + peak effort,
            then returns to baseline before testing the next joint.

Usage (left hand):
  # Terminal 1: driver + lj_dg_pospid controller (NO teleop).
  ros2 launch dg5f_driver dg5f_left_pid_all_controller.launch.py
  # Terminal 2: run the diag.
  ros2 run dg5f_finger_diag finger_diag --ros-args -p hand_side:=left

The node publishes control_msgs/MultiDOFCommand on the controller's
reference topic, so DO NOT run a teleop pipeline at the same time
(commands would race). On exit it holds at baseline briefly; the
hardware on_deactivate then releases torque (fingers go limp) when the
controller is stopped.

Key params (all optional):
  hand_side         : "left" (default) | "right"
  mode              : "step" (default) | "passive"
  amplitude_rad     : per-joint step magnitude from baseline (0.30)
  max_abs_command   : absolute clamp on commanded value (1.0)
  settle_time_s     : hold time at step target (1.5)
  recover_time_s    : hold time at baseline between joints (0.5)
  collect_time_s    : passive-mode collection window (3.0)
  publish_rate_hz   : reference publish rate (50.0)
  step_sign         : list[20] of +/-1 to choose step direction per joint
  pos_error_warn    : WARN_TRACKING threshold on |reached-cmd| (0.05 rad)
  pos_error_fail    : FAIL_POOR_TRACK threshold (0.15 rad)
  motion_stuck_thresh: FAIL_STUCK if |reached-baseline| below this (0.03 rad)
  peak_eff_high     : WARN_HIGH_EFFORT threshold on peak |effort| (30.0)
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState


SIDE_PREFIX = {"left": "lj", "right": "rj"}


def joint_names_for(side: str) -> List[str]:
    p = SIDE_PREFIX[side]
    # Order: a..e (thumb..pinky) × joints 1..4, matching the rest of the stack.
    return [f"{p}_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]


def finger_letter(slot: int) -> str:
    return "abcde"[slot // 4]  # a=thumb b=index c=middle d=ring e=pinky


@dataclass
class Trace:
    pos: List[float] = field(default_factory=list)
    vel: List[float] = field(default_factory=list)
    eff: List[float] = field(default_factory=list)


@dataclass
class StepResult:
    name: str
    baseline: float
    commanded: float
    reached: float
    pos_error: float       # |reached - commanded|
    motion: float          # reached - baseline (signed)
    expected_sign: int
    measured_sign: int
    peak_eff: float
    verdict: str


class FingerDiagNode(Node):
    def __init__(self):
        super().__init__("dg5f_finger_diag")

        self.declare_parameter("hand_side", "left")
        self.declare_parameter("mode", "step")
        self.declare_parameter("amplitude_rad", 0.30)
        self.declare_parameter("max_abs_command", 1.0)
        self.declare_parameter("settle_time_s", 1.5)
        self.declare_parameter("recover_time_s", 0.5)
        self.declare_parameter("collect_time_s", 3.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("step_sign", [1.0] * 20)
        self.declare_parameter("pos_error_warn", 0.05)
        self.declare_parameter("pos_error_fail", 0.15)
        self.declare_parameter("motion_stuck_thresh", 0.03)
        self.declare_parameter("peak_eff_high", 30.0)

        self._side = str(self.get_parameter("hand_side").value).lower()
        if self._side not in ("left", "right"):
            raise ValueError(f"hand_side must be left|right, got {self._side!r}")
        self._mode = str(self.get_parameter("mode").value).lower()
        if self._mode not in ("passive", "step"):
            raise ValueError(f"mode must be passive|step, got {self._mode!r}")

        self._amp = float(self.get_parameter("amplitude_rad").value)
        self._abs_max = float(self.get_parameter("max_abs_command").value)
        self._settle = float(self.get_parameter("settle_time_s").value)
        self._recover = float(self.get_parameter("recover_time_s").value)
        self._collect = float(self.get_parameter("collect_time_s").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._pub_dt = 1.0 / max(rate, 1.0)
        signs = list(self.get_parameter("step_sign").value)
        if len(signs) != 20:
            raise ValueError("step_sign must have 20 entries")
        self._sign = [1.0 if float(s) >= 0 else -1.0 for s in signs]
        self._err_warn = float(self.get_parameter("pos_error_warn").value)
        self._err_fail = float(self.get_parameter("pos_error_fail").value)
        self._stuck = float(self.get_parameter("motion_stuck_thresh").value)
        self._peak_eff_high = float(self.get_parameter("peak_eff_high").value)

        self._joint_names = joint_names_for(self._side)
        ctrl = f"{SIDE_PREFIX[self._side]}_dg_pospid"
        self._ref_topic = f"/dg5f_{self._side}/{ctrl}/reference"
        self._state_topic = f"/dg5f_{self._side}/joint_states"

        self._latest_pos: Dict[str, float] = {}
        self._traces: Dict[str, Trace] = {n: Trace() for n in self._joint_names}
        self._collect_trace = False

        self._pub = self.create_publisher(MultiDOFCommand, self._ref_topic, 10)
        self.create_subscription(
            JointState, self._state_topic, self._on_state, 10)

        self.get_logger().info(
            f"finger_diag[{self._side}]: mode={self._mode} "
            f"ref->{self._ref_topic} state<-{self._state_topic} "
            f"amp={self._amp:.3f}rad settle={self._settle}s")

    # ------------------------------------------------------------------ I/O
    def _on_state(self, msg: JointState):
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                self._latest_pos[n] = float(msg.position[i])
            if self._collect_trace and n in self._traces:
                t = self._traces[n]
                if i < len(msg.position):
                    t.pos.append(float(msg.position[i]))
                if i < len(msg.velocity):
                    t.vel.append(float(msg.velocity[i]))
                if i < len(msg.effort):
                    t.eff.append(float(msg.effort[i]))

    def _wait_state(self, timeout_s: float = 10.0) -> bool:
        end = time.time() + timeout_s
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(n in self._latest_pos for n in self._joint_names):
                return True
        return False

    def _publish_ref(self, values: List[float]):
        msg = MultiDOFCommand()
        msg.dof_names = list(self._joint_names)
        msg.values = list(values)
        msg.values_dot = [0.0] * len(values)
        self._pub.publish(msg)

    def _hold(self, duration_s: float, values: List[float]):
        """Publish `values` at publish_rate_hz for duration_s while spinning."""
        end = time.time() + duration_s
        next_pub = 0.0
        while rclpy.ok() and time.time() < end:
            now = time.time()
            if now >= next_pub:
                self._publish_ref(values)
                next_pub = now + self._pub_dt
            rclpy.spin_once(self, timeout_sec=0.01)

    def _clear_traces(self):
        for t in self._traces.values():
            t.pos.clear()
            t.vel.clear()
            t.eff.clear()

    # ------------------------------------------------------------------ run
    def run(self):
        self.get_logger().info("waiting for joint_states...")
        if not self._wait_state(10.0):
            self.get_logger().error(
                f"no joint_states on {self._state_topic} after 10s. "
                f"Is the driver up and the controller "
                f"{SIDE_PREFIX[self._side]}_dg_pospid active?")
            return

        baseline = [float(self._latest_pos[n]) for n in self._joint_names]
        flagged = ", ".join(
            f"{i}({n})={v:+.2f}"
            for i, (n, v) in enumerate(zip(self._joint_names, baseline))
            if abs(v) > 0.1)
        self.get_logger().info(
            f"baseline non-zero (>0.1rad): {flagged or '(none)'}")

        if self._mode == "passive":
            self._run_passive()
        else:
            self._run_step(baseline)

    def _run_passive(self):
        self.get_logger().info(
            f"PASSIVE: collecting {self._collect:.1f}s of joint_states "
            f"(no commands sent)")
        self._clear_traces()
        self._collect_trace = True
        end = time.time() + self._collect
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._collect_trace = False
        self._print_passive()

    def _print_passive(self):
        print()
        print("=== passive diagnostic (no motion commanded) ===")
        print(f"{'#':<3}{'fng':<4}{'joint':<13}"
              f"{'pos_mean':>10}{'pos_std':>10}"
              f"{'vel_mean':>10}{'eff_mean':>10}{'eff_peak':>10}{'N':>5}")
        for i, n in enumerate(self._joint_names):
            t = self._traces[n]
            if not t.pos:
                print(f"{i:<3}{finger_letter(i):<4}{n:<13}  (no samples)")
                continue
            pm = sum(t.pos) / len(t.pos)
            ps = (sum((p - pm) ** 2 for p in t.pos) / len(t.pos)) ** 0.5
            vm = sum(t.vel) / len(t.vel) if t.vel else 0.0
            em = sum(t.eff) / len(t.eff) if t.eff else 0.0
            ep = max((abs(e) for e in t.eff), default=0.0)
            print(f"{i:<3}{finger_letter(i):<4}{n:<13}"
                  f"{pm:>10.3f}{ps:>10.3f}{vm:>10.3f}"
                  f"{em:>10.2f}{ep:>10.2f}{len(t.pos):>5}")
        print()

    def _run_step(self, baseline: List[float]):
        results: List[StepResult] = []
        for i, n in enumerate(self._joint_names):
            if not rclpy.ok():
                break
            cmd = baseline[i] + self._sign[i] * self._amp
            cmd = max(-self._abs_max, min(self._abs_max, cmd))
            ref = list(baseline)
            ref[i] = cmd

            self.get_logger().info(
                f"[{i + 1:2d}/20] {finger_letter(i)} {n}: "
                f"base={baseline[i]:+.3f} -> cmd={cmd:+.3f} "
                f"(Δ={cmd - baseline[i]:+.3f})")

            self._clear_traces()
            self._collect_trace = True
            self._hold(self._settle, ref)
            self._collect_trace = False

            t = self._traces[n]
            tail = t.pos[-max(5, len(t.pos) // 3):] if t.pos else []
            reached = sum(tail) / len(tail) if tail else baseline[i]
            peak_eff = max((abs(e) for e in t.eff), default=0.0)
            pos_err = abs(reached - cmd)
            motion = reached - baseline[i]
            expected_sign = 1 if self._sign[i] >= 0 else -1
            measured_sign = (
                1 if motion > 0 else (-1 if motion < 0 else 0))

            results.append(StepResult(
                name=n, baseline=baseline[i], commanded=cmd,
                reached=reached, pos_error=pos_err, motion=motion,
                expected_sign=expected_sign,
                measured_sign=measured_sign,
                peak_eff=peak_eff,
                verdict=self._verdict(
                    pos_err, motion, expected_sign,
                    measured_sign, peak_eff)))

            # Recover to baseline before next joint.
            self._hold(self._recover, baseline)

        # Brief hold at baseline; HW on_deactivate will release torque when the
        # controller stops (so the hand goes limp at shutdown, not stiff).
        self._hold(0.3, baseline)
        self._print_step(results)

    def _verdict(self, pos_err, motion, expected_sign, measured_sign,
                 peak_eff) -> str:
        if abs(motion) < self._stuck:
            return "FAIL_STUCK"
        if measured_sign != expected_sign:
            return "FAIL_REVERSED"
        if pos_err > self._err_fail:
            return "FAIL_POOR_TRACK"
        if peak_eff > self._peak_eff_high:
            return "WARN_HIGH_EFFORT"
        if pos_err > self._err_warn:
            return "WARN_TRACKING"
        return "PASS"

    def _print_step(self, results: List[StepResult]):
        print()
        print("=== step diagnostic ===")
        print(f"{'#':<3}{'fng':<4}{'joint':<13}"
              f"{'base':>8}{'cmd':>8}{'reach':>8}"
              f"{'err':>8}{'motion':>9}{'peakE':>8}  verdict")
        for i, r in enumerate(results):
            print(f"{i:<3}{finger_letter(i):<4}{r.name:<13}"
                  f"{r.baseline:>+8.3f}{r.commanded:>+8.3f}"
                  f"{r.reached:>+8.3f}{r.pos_error:>8.3f}"
                  f"{r.motion:>+9.3f}{r.peak_eff:>8.2f}  {r.verdict}")
        bad = [r for r in results if not r.verdict.startswith("PASS")]
        if not bad:
            print("\n=> all 20 joints PASS")
        else:
            print(f"\n=> {len(bad)} joint(s) flagged:")
            for r in bad:
                print(f"   {r.name}: {r.verdict} "
                      f"(motion={r.motion:+.3f}, err={r.pos_error:.3f}, "
                      f"peakE={r.peak_eff:.2f})")
        print()


def main(args=None):
    rclpy.init(args=args)
    node = FingerDiagNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
