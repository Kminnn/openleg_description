#!/usr/bin/env python3
"""Deterministic right-then-left joint-space walking demonstration."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    *(f"left_joint{index}" for index in range(1, 7)),
    *(f"right_joint{index}" for index in range(1, 7)),
]

JOINT_LOWER = np.asarray([
    -0.915, -math.pi / 6, -math.pi / 2, -37 * math.pi / 72,
    -49 * math.pi / 360, -math.pi / 6,
    -0.915, -math.pi / 2, -math.pi / 2, -37 * math.pi / 72,
    -49 * math.pi / 360, -math.pi / 6,
])
JOINT_UPPER = np.asarray([
    -0.915 + 5 * math.pi / 6, math.pi / 9, math.pi / 2, math.pi / 72,
    49 * math.pi / 360, math.pi / 6,
    -0.915 + 5 * math.pi / 6, math.pi / 6, math.pi / 2, math.pi / 72,
    49 * math.pi / 360, math.pi / 6,
])


@dataclass(frozen=True)
class Keyframe:
    label: str
    duration: float
    left: tuple[float, ...]
    right: tuple[float, ...]

    @property
    def positions(self) -> np.ndarray:
        return np.asarray(self.left + self.right, dtype=float)


STAND = (0.1271, 0.0002, 0.0, -0.3522, 0.2566, 0.0)
DEFAULT_FORWARD_LEAN_DEG = 2.5
DEFAULT_WEIGHT_SHIFT_SCALE = 2.0
DEFAULT_SUPPORT_ANKLE_ROLL_DEG = 3.0
LIFT_HIP_PITCH = 0.7060
SWING_HIP_PITCH = 0.5747
SWING_KNEE_PITCH = -1.5800
SWING_ANKLE_PITCH = 49.0 * math.pi / 360.0


def apply_forward_support_pitch(
    positions: np.ndarray, label: str,
    chain_pitch: float = math.radians(DEFAULT_FORWARD_LEAN_DEG),
    weight_shift_scale: float = DEFAULT_WEIGHT_SHIFT_SCALE,
    support_ankle_roll: float = math.radians(DEFAULT_SUPPORT_ANKLE_ROLL_DEG),
) -> np.ndarray:
    """Apply a fixed forward body pitch through only the planted leg(s)."""
    result = positions.copy()
    result[[1, 7]] *= weight_shift_scale
    if label in ("lift right leg", "swing right leg forward"):
        support_offsets = (0,)
        swing_offset = 6
    elif label in ("lift left leg", "swing left leg forward"):
        support_offsets = (6,)
        swing_offset = 0
    else:
        support_offsets = (0, 6)
        swing_offset = None

    for offset in support_offsets:
        # Chain pitch is joint1 + joint4 + joint5. In the Gazebo model’s observed
        # orientation, forward body direction requires a positive support-chain pitch.
        result[offset + 4] = (
            chain_pitch
            - result[offset]
            - result[offset + 3]
        )
    if swing_offset is not None:
        # These poses come from the current URDF kinematics. The lift pose
        # raises the foot about 120 mm without a fore-aft kick; the swing pose
        # then advances it about 60 mm while holding that clearance.
        result[swing_offset] = (
            LIFT_HIP_PITCH if label.startswith("lift ") else SWING_HIP_PITCH
        )
        result[swing_offset + 3] = SWING_KNEE_PITCH
        result[swing_offset + 4] = SWING_ANKLE_PITCH
    # In double support, both ankles create the same body roll using
    # opposite joint6 signs because the feet are mirrored.
    if label == "weight onto left leg":
        result[5] = support_ankle_roll
        result[11] = -support_ankle_roll
    elif label in ("lift right leg", "swing right leg forward"):
        result[5] = support_ankle_roll
    elif label == "plant right foot":
        result[5] = 0.5 * support_ankle_roll
        result[11] = -0.5 * support_ankle_roll
    elif label in ("move pelvis over right foot", "weight onto right leg"):
        result[5] = -support_ankle_roll
        result[11] = support_ankle_roll
    elif label in ("lift left leg", "swing left leg forward"):
        result[11] = support_ankle_roll
    elif label == "plant left foot":
        result[5] = -0.5 * support_ankle_roll
        result[11] = 0.5 * support_ankle_roll
    return result

# Fixed motor path generated from the current URDF. Joint order for each leg is
# hip pitch, hip roll, hip yaw, knee pitch, ankle pitch, ankle roll.
FIXED_CYCLE = (
    Keyframe("pre-lean before first step", 1.20, STAND, STAND),
    Keyframe(
        "weight onto left leg", 0.80,
        (0.0931, 0.0357, 0.0, -0.3235, 0.3500, 0.0),
        (0.0934, -0.0354, 0.0, -0.3248, 0.3515, 0.0),
    ),
    Keyframe(
        "lift right leg", 0.60,
        (0.1278, 0.0391, 0.0, -0.3249, 0.3146, 0.0),
        (0.4121, -0.0445, 0.0, -1.0542, 0.4276, 0.0),
    ),
    Keyframe(
        "swing right leg forward", 0.70,
        (0.1731, 0.0354, 0.0, -0.3236, 0.2642, 0.0),
        (0.3811, -0.0412, 0.0, -1.1027, 0.4225, 0.0),
    ),
    Keyframe(
        "plant right foot", 0.55,
        (0.1846, 0.0177, 0.0, -0.3241, 0.2525, 0.0),
        (-0.0255, -0.0176, 0.0, -0.3024, 0.4276, 0.0),
    ),
    Keyframe(
        "move pelvis over right foot", 0.70,
        (0.1513, 0.0001, 0.0, -0.3276, 0.2930, 0.0),
        (0.0344, 0.0002, 0.0, -0.3173, 0.4039, 0.0),
    ),
    Keyframe(
        "weight onto right leg", 0.80,
        (0.0934, -0.0354, 0.0, -0.3248, 0.3515, 0.0),
        (0.0931, 0.0357, 0.0, -0.3235, 0.3500, 0.0),
    ),
    Keyframe(
        "lift left leg", 0.60,
        (0.4121, -0.0445, 0.0, -1.0542, 0.4276, 0.0),
        (0.1278, 0.0391, 0.0, -0.3249, 0.3146, 0.0),
    ),
    Keyframe(
        "swing left leg forward", 0.70,
        (0.3811, -0.0412, 0.0, -1.1027, 0.4225, 0.0),
        (0.1731, 0.0354, 0.0, -0.3236, 0.2642, 0.0),
    ),
    Keyframe(
        "plant left foot", 0.55,
        (-0.0255, -0.0176, 0.0, -0.3024, 0.4276, 0.0),
        (0.1846, 0.0177, 0.0, -0.3241, 0.2525, 0.0),
    ),
    Keyframe(
        "center after left step", 0.70,
        (0.0344, 0.0002, 0.0, -0.3173, 0.4039, 0.0),
        (0.1513, 0.0001, 0.0, -0.3276, 0.2930, 0.0),
    ),
    Keyframe("hold forward stance", 0.80, STAND, STAND),
)


def validate_path() -> None:
    if len(JOINT_NAMES) != 12 or len(set(JOINT_NAMES)) != 12:
        raise ValueError("Expected 12 unique motor joint names")
    for frame in FIXED_CYCLE:
        positions = frame.positions
        if positions.shape != (12,) or not np.all(np.isfinite(positions)):
            raise ValueError(f"Invalid positions in keyframe: {frame.label}")
        if frame.duration <= 0.0:
            raise ValueError(f"Invalid duration in keyframe: {frame.label}")
        adjusted = apply_forward_support_pitch(positions, frame.label)
        if np.any(adjusted < JOINT_LOWER) or np.any(adjusted > JOINT_UPPER):
            raise ValueError(f"Joint limit exceeded in keyframe: {frame.label}")


def seconds_to_point_time(point: JointTrajectoryPoint, seconds: float) -> None:
    whole = int(seconds)
    point.time_from_start.sec = whole
    point.time_from_start.nanosec = int(round((seconds - whole) * 1e9))
    if point.time_from_start.nanosec == 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec = 0


class FixedGaitDemo(Node):
    def __init__(self) -> None:
        super().__init__("fixed_gait_demo")
        self.declare_parameter("cycles", 1)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("motion_scale", 1.0)
        self.declare_parameter("forward_lean_deg", DEFAULT_FORWARD_LEAN_DEG)
        self.declare_parameter(
            "weight_shift_scale", DEFAULT_WEIGHT_SHIFT_SCALE
        )
        self.declare_parameter(
            "support_ankle_roll_deg", DEFAULT_SUPPORT_ANKLE_ROLL_DEG
        )
        self.declare_parameter("auto_start", False)
        self.declare_parameter(
            "controller_topic", "/leg_controller/joint_trajectory"
        )
        self.publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("controller_topic").value),
            10,
        )

    def build_trajectory(self) -> tuple[JointTrajectory, list[tuple[float, str]]]:
        cycles = max(1, int(self.get_parameter("cycles").value))
        speed_scale = float(np.clip(
            self.get_parameter("speed_scale").value, 0.25, 1.50
        ))
        motion_scale = float(np.clip(
            self.get_parameter("motion_scale").value, 0.25, 1.00
        ))
        chain_pitch = math.radians(float(np.clip(
            self.get_parameter("forward_lean_deg").value, -10.0, 10.0
        )))
        requested_weight_shift = float(
            self.get_parameter("weight_shift_scale").value
        )
        weight_shift_scale = float(np.clip(
            requested_weight_shift, 0.5, 2.5
        ))
        if not math.isclose(requested_weight_shift, weight_shift_scale):
            self.get_logger().warning(
                f"weight_shift_scale {requested_weight_shift:g} was clipped "
                f"to {weight_shift_scale:g} to respect joint2 limits"
            )
        support_ankle_roll = math.radians(float(np.clip(
            self.get_parameter("support_ankle_roll_deg").value, -8.0, 8.0
        )))
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        schedule: list[tuple[float, str]] = []
        elapsed = 0.0
        stand = np.asarray(STAND + STAND, dtype=float)

        for cycle in range(cycles):
            frames = FIXED_CYCLE if cycle == 0 else FIXED_CYCLE[1:]
            for frame in frames:
                elapsed += frame.duration / speed_scale
                positions = stand + motion_scale * (frame.positions - stand)
                positions = apply_forward_support_pitch(
                    positions, frame.label, chain_pitch, weight_shift_scale,
                    support_ankle_roll,
                )
                point = JointTrajectoryPoint()
                point.positions = positions.tolist()
                seconds_to_point_time(point, elapsed)
                message.points.append(point)
                schedule.append((elapsed, f"cycle {cycle + 1}: {frame.label}"))
        return message, schedule

    def wait_for_controller(self, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


    def run(self) -> int:
        validate_path()
        message, schedule = self.build_trajectory()
        self.get_logger().info(
            "Fixed open-loop path ready: right step, then left step. "
            "Do not run wasd_ik_teleop at the same time."
        )
        for timestamp, label in schedule:
            self.get_logger().info(f"  {timestamp:5.2f} s  {label}")

        self.get_logger().info("Waiting for leg_controller...")
        if not self.wait_for_controller():
            self.get_logger().error(
                "leg_controller did not subscribe within 30 seconds"
            )
            return 3


        self.get_logger().info("Controller ready; no trajectory sent before Enter.")

        if not bool(self.get_parameter("auto_start").value):
            if not sys.stdin.isatty():
                self.get_logger().error(
                    "No terminal is available. Set auto_start:=true to run."
                )
                return 2
            try:
                input("Press Enter to send the fixed motor path (Ctrl+C cancels): ")
            except EOFError:
                return 2

        self.publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.5)
        total = schedule[-1][0]
        self.get_logger().info(
            f"Published {len(message.points)} fixed keyframes over "
            f"{total:.2f} simulation seconds."
        )
        return 0


def main() -> None:
    if "--self-test" in sys.argv:
        validate_path()
        print(
            f"fixed gait valid: {len(FIXED_CYCLE)} keyframes, "
            f"{len(JOINT_NAMES)} joints"
        )
        return
    rclpy.init()
    node = FixedGaitDemo()
    try:
        raise SystemExit(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
