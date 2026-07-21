#!/usr/bin/env python3
"""Quasi-static planted shift followed by a corrected 10 mm right-foot lift."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers



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

# Stable planted preparation pose from the current joint-space demo.
STAND = np.asarray(
    (0.1271, 0.0002, 0.0, -0.3522, 0.2566, 0.0) * 2,
    dtype=float,
)

# Kinematically solved against the current URDF. The shift keeps both foot
# collision frames coplanar while moving the COM well inside the left foot.
SHIFT_LEFT = np.asarray([
    0.211095, 0.174533, 0.0, -0.673240, 0.427605, -0.226374,
    0.089014, -0.166631, 0.009424, -0.423997, 0.298856, 0.221807,
], dtype=float)

# The lift keyframes keep every left support joint fixed, keep fore-aft foot
# displacement below 2 mm. Because the standing foot frame has +Z downward,
# upward swing clearance is solved along negative local Z.
LIFT_RIGHT_10_MM = np.asarray([
    0.211095, 0.174533, 0.0, -0.673240, 0.427605, -0.226374,
    0.155570, -0.170702, -0.002364, -0.599496, 0.408729, 0.225693,
], dtype=float)
LIFT_RIGHT_15_MM = np.asarray([
    0.211095, 0.174533, 0.0, -0.673240, 0.427605, -0.226374,
    0.157100, -0.205200, -0.009300, -0.604600, 0.412600, 0.260800,
], dtype=float)
LIFT_RIGHT_20_MM = np.asarray([
    0.211095, 0.174533, 0.0, -0.673240, 0.427605, -0.226374,
    0.153491, -0.240944, -0.015408, -0.595752, 0.407646, 0.297193,
], dtype=float)


@dataclass(frozen=True)
class Phase:
    label: str
    duration: float
    positions: np.ndarray


def seconds_to_point_time(point: JointTrajectoryPoint, seconds: float) -> None:
    whole = int(seconds)
    point.time_from_start.sec = whole
    point.time_from_start.nanosec = int(round((seconds - whole) * 1e9))
    if point.time_from_start.nanosec == 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec = 0


def make_phases(lift_height_mm: float = 10.0) -> tuple[Phase, ...]:
    if lift_height_mm <= 15.0:
        lower, upper = LIFT_RIGHT_10_MM, LIFT_RIGHT_15_MM
        lift_fraction = (lift_height_mm - 10.0) / 5.0
    else:
        lower, upper = LIFT_RIGHT_15_MM, LIFT_RIGHT_20_MM
        lift_fraction = (lift_height_mm - 15.0) / 5.0
    lift = lower + lift_fraction * (upper - lower)

    return (
        Phase("prepare planted stance", 2.0, STAND),
        Phase("settle with both feet planted", 1.0, STAND),
        Phase("shift body toward left foot", 2.0, SHIFT_LEFT),
        Phase("hold left shift", 2.0, SHIFT_LEFT),
        Phase(f"lift right foot {lift_height_mm:.1f} mm", 2.0, lift),
    )


def validate_phases(phases: tuple[Phase, ...]) -> None:
    if len(JOINT_NAMES) != 12 or len(set(JOINT_NAMES)) != 12:
        raise ValueError("Expected 12 unique motor joint names")
    for phase in phases:
        if phase.positions.shape != (12,) or not np.all(
            np.isfinite(phase.positions)
        ):
            raise ValueError(f"Invalid positions in phase: {phase.label}")
        if phase.duration <= 0.0:
            raise ValueError(f"Invalid duration in phase: {phase.label}")
        if np.any(phase.positions < JOINT_LOWER) or np.any(
            phase.positions > JOINT_UPPER
        ):
            raise ValueError(f"Joint limit exceeded in phase: {phase.label}")


class RightLegLiftTest(Node):
    def __init__(self) -> None:
        super().__init__("right_leg_lift_test")
        self.declare_parameter("lift_height_mm", 10.0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter(
            "controller_topic", "/leg_controller/joint_trajectory"
        )
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("controller_topic").value),
            10,
        )

    def build_trajectory(
        self,
    ) -> tuple[JointTrajectory, list[tuple[float, str]]]:
        lift_height_mm = float(np.clip(
            self.get_parameter("lift_height_mm").value, 10.0, 20.0
        ))
        phases = make_phases(lift_height_mm)
        validate_phases(phases)

        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        elapsed = 0.0
        schedule: list[tuple[float, str]] = []
        for phase in phases:
            elapsed += phase.duration
            point = JointTrajectoryPoint()
            point.positions = phase.positions.tolist()
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.accelerations = [0.0] * len(JOINT_NAMES)
            seconds_to_point_time(point, elapsed)
            message.points.append(point)
            schedule.append((elapsed, phase.label))
        return message, schedule

    def wait_for_controller(self, timeout: float = 30.0) -> bool:
        if not self.controller_client.wait_for_service(timeout_sec=timeout):
            return False
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            future = self.controller_client.call_async(
                ListControllers.Request()
            )
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            if future.done() and future.result() is not None:
                active = any(
                    controller.name == "leg_controller"
                    and controller.state == "active"
                    for controller in future.result().controller
                )
                if active and self.publisher.get_subscription_count() > 0:
                    return True
            time.sleep(0.1)
        return False

    def run(self) -> int:
        message, schedule = self.build_trajectory()
        self.get_logger().info(
            "Right-leg-lift test ready. Both feet stay planted through the "
            "left shift and hold; there is no forward swing or IMU feedback."
        )
        for timestamp, label in schedule:
            self.get_logger().info(f"  {timestamp:4.1f} s  {label}")

        self.get_logger().info("Waiting for leg_controller...")
        if not self.wait_for_controller():
            self.get_logger().error(
                "leg_controller did not become active within 30 seconds"
            )
            return 3

        if not bool(self.get_parameter("auto_start").value):
            if not sys.stdin.isatty():
                self.get_logger().error(
                    "No terminal is available. Set auto_start:=true to run."
                )
                return 2
            try:
                input(
                    "Press Enter to shift left and lift the right foot "
                    "(Ctrl+C cancels): "
                )
            except EOFError:
                return 2

        self.publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(
            "Trajectory published. The controller will hold the final "
            "single-support target after 9.0 seconds."
        )
        return 0


def main() -> None:
    if "--self-test" in sys.argv:
        phases = make_phases()
        validate_phases(phases)
        print(
            f"right-leg-lift test valid: {len(phases)} phases, "
            f"{len(JOINT_NAMES)} joints"
        )
        return

    rclpy.init()
    node = RightLegLiftTest()
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
