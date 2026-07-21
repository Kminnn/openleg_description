#!/usr/bin/env python3
"""Open-loop alternating short-step walking sequence for OpenLeg."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
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
class Phase:
    label: str
    duration: float
    positions: np.ndarray


def pose(*values: float) -> np.ndarray:
    return np.asarray(values, dtype=float)


# Solved against the current URDF with fixed world contact poses. Successive
# foot placements are 30 mm apart along +base X and swing clearance is 20 mm.
# During double support, the foot-to-foot translation stays within 0.6 mm even
# between trajectory samples. Small joint3 offsets cancel kinematic yaw; they
# do not command a turn. Joint2 and joint6 provide the lateral transfer.
STAND = pose(*((0.1271, 0.0002, 0.0, -0.3522, 0.2566, 0.0) * 2))
SHIFT_LEFT_25 = pose(
    0.142967, 0.047520, 0.005298, -0.393144, 0.281802, -0.047615,
    0.145858, -0.047118, -0.005429, -0.401037, 0.286808, 0.047628,
)
SHIFT_LEFT_50 = pose(
    0.146692, 0.094983, 0.010954, -0.401363, 0.286690, -0.095411,
    0.152320, -0.094576, -0.011484, -0.416710, 0.296434, 0.095467,
)
SHIFT_LEFT_75 = pose(
    0.138983, 0.142556, 0.015309, -0.378903, 0.272512, -0.143171,
    0.147878, -0.142143, -0.016579, -0.402941, 0.287745, 0.143299,
)
SHIFT_LEFT_INITIAL = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.131019, -0.189783, -0.018853, -0.356497, 0.258775, 0.190905,
)
RIGHT_LIFT_INITIAL = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.284276, -0.191289, -0.049387, -0.721692, 0.427605, 0.197692,
)
RIGHT_SWING_FORWARD = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.328283, -0.188770, -0.057343, -0.688650, 0.397299, 0.197387,
)
RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.190095, -0.188103, -0.029918, -0.337480, 0.181709, 0.190638,
)
TRANSFER_RIGHT_25 = pose(
    0.167577, 0.095610, 0.013051, -0.499740, 0.364286, -0.096295,
    0.237207, -0.093997, -0.019601, -0.508059, 0.303275, 0.096212,
)
TRANSFER_RIGHT_50 = pose(
    0.167965, 0.000225, 0.000012, -0.547729, 0.411264, -0.000025,
    0.236434, 0.000181, 0.000018, -0.552146, 0.347212, 0.000018,
)
TRANSFER_RIGHT_75 = pose(
    0.134402, -0.095679, -0.009884, -0.500906, 0.398478, 0.096386,
    0.201820, 0.095010, 0.016295, -0.502126, 0.332579, -0.096194,
)
SHIFT_RIGHT = pose(
    0.051650, -0.190936, -0.003843, -0.323415, 0.303635, 0.191174,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
LEFT_LIFT_VERTICAL = pose(
    0.239343, -0.193460, -0.041240, -0.740479, 0.427605, 0.197988,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
LEFT_SWING_FORWARD = pose(
    0.328283, -0.188770, -0.057343, -0.688650, 0.397299, 0.197387,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
LEFT_PLANT = pose(
    0.190095, -0.188103, -0.029918, -0.337480, 0.181709, 0.190638,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
TRANSFER_LEFT_25 = pose(
    0.237207, -0.093997, -0.019601, -0.508059, 0.303275, 0.096212,
    0.167577, 0.095610, 0.013051, -0.499740, 0.364286, -0.096295,
)
TRANSFER_LEFT_50 = pose(
    0.236434, 0.000181, 0.000018, -0.552146, 0.347212, 0.000018,
    0.167965, 0.000225, 0.000012, -0.547729, 0.411264, -0.000025,
)
TRANSFER_LEFT_75 = pose(
    0.201820, 0.095010, 0.016295, -0.502126, 0.332579, -0.096194,
    0.134402, -0.095679, -0.009884, -0.500906, 0.398478, 0.096386,
)
SHIFT_LEFT_NEXT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.051650, -0.190936, -0.003843, -0.323415, 0.303635, 0.191174,
)
RIGHT_LIFT_PERIODIC = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.239343, -0.193460, -0.041240, -0.740479, 0.427605, 0.197988,
)


def make_phases(cycles: int = 10, speed_scale: float = 1.0) -> tuple[Phase, ...]:
    cycles = max(1, min(10, int(cycles)))
    speed_scale = float(np.clip(speed_scale, 0.5, 1.5))
    phases: list[Phase] = []

    def add(label: str, duration: float, positions: np.ndarray) -> None:
        phases.append(Phase(label, duration / speed_scale, positions))

    add("prepare planted stance", 2.00, STAND)
    add("shift left 25 percent with both feet fixed", 0.50, SHIFT_LEFT_25)
    add("shift left 50 percent with both feet fixed", 0.50, SHIFT_LEFT_50)
    add("shift left 75 percent with both feet fixed", 0.50, SHIFT_LEFT_75)
    add("shift onto left foot", 0.50, SHIFT_LEFT_INITIAL)
    add("hold left support", 0.60, SHIFT_LEFT_INITIAL)
    add("lift right foot vertically", 1.50, RIGHT_LIFT_INITIAL)
    for cycle in range(1, cycles + 1):
        add(f"cycle {cycle}: swing right foot forward", 1.00, RIGHT_SWING_FORWARD)
        add(f"cycle {cycle}: hold right foot clear", 0.60, RIGHT_SWING_FORWARD)
        add(f"cycle {cycle}: plant right foot", 1.00, RIGHT_PLANT)
        add(f"cycle {cycle}: transfer right 25 percent", 1.00, TRANSFER_RIGHT_25)
        add(f"cycle {cycle}: transfer right 50 percent", 1.00, TRANSFER_RIGHT_50)
        add(f"cycle {cycle}: transfer right 75 percent", 1.00, TRANSFER_RIGHT_75)
        add(f"cycle {cycle}: support on right foot", 1.00, SHIFT_RIGHT)
        add(f"cycle {cycle}: hold right support", 1.50, SHIFT_RIGHT)
        add(f"cycle {cycle}: lift left foot vertically", 1.50, LEFT_LIFT_VERTICAL)
        add(f"cycle {cycle}: swing left foot forward", 1.00, LEFT_SWING_FORWARD)
        add(f"cycle {cycle}: hold left foot clear", 0.60, LEFT_SWING_FORWARD)
        add(f"cycle {cycle}: plant left foot", 1.00, LEFT_PLANT)
        add(f"cycle {cycle}: transfer left 25 percent", 1.00, TRANSFER_LEFT_25)
        add(f"cycle {cycle}: transfer left 50 percent", 1.00, TRANSFER_LEFT_50)
        add(f"cycle {cycle}: transfer left 75 percent", 1.00, TRANSFER_LEFT_75)
        add(f"cycle {cycle}: support on left foot", 1.00, SHIFT_LEFT_NEXT)
        add(f"cycle {cycle}: hold left support", 1.50, SHIFT_LEFT_NEXT)
        if cycle < cycles:
            add(f"cycle {cycle + 1}: lift right foot vertically", 1.50, RIGHT_LIFT_PERIODIC)
    add("hold final planted stance", 1.50, SHIFT_LEFT_NEXT)
    return tuple(phases)


def validate_phases(phases: tuple[Phase, ...]) -> None:
    if len(JOINT_NAMES) != 12 or len(set(JOINT_NAMES)) != 12:
        raise ValueError("Expected 12 unique motor joint names")
    for phase in phases:
        if phase.positions.shape != (12,) or not np.all(np.isfinite(phase.positions)):
            raise ValueError(f"Invalid positions in phase: {phase.label}")
        if phase.duration <= 0.0:
            raise ValueError(f"Invalid duration in phase: {phase.label}")
        if np.any(phase.positions < JOINT_LOWER) or np.any(phase.positions > JOINT_UPPER):
            raise ValueError(f"Joint limit exceeded in phase: {phase.label}")


def seconds_to_point_time(point: JointTrajectoryPoint, seconds: float) -> None:
    whole = int(seconds)
    point.time_from_start.sec = whole
    point.time_from_start.nanosec = int(round((seconds - whole) * 1e9))
    if point.time_from_start.nanosec == 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec = 0


class ShortStepWalkDemo(Node):
    def __init__(self) -> None:
        super().__init__("short_step_walk_demo")
        self.declare_parameter("cycles", 10)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("controller_topic", "/leg_controller/joint_trajectory")
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("controller_topic").value),
            10,
        )

    def build_trajectory(self) -> tuple[JointTrajectory, list[tuple[float, str]]]:
        phases = make_phases(
            int(self.get_parameter("cycles").value),
            float(self.get_parameter("speed_scale").value),
        )
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
            future = self.controller_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            if future.done() and future.result() is not None:
                active = any(
                    controller.name == "leg_controller" and controller.state == "active"
                    for controller in future.result().controller
                )
                if active and self.publisher.get_subscription_count() > 0:
                    return True
            time.sleep(0.1)
        return False

    def run(self) -> int:
        message, schedule = self.build_trajectory()
        cycles = max(1, min(10, int(self.get_parameter("cycles").value)))
        self.get_logger().info(
            f"{2 * cycles}-step contact-locked walk ready: 30 mm foot "
            "placements, 20 mm swing clearance, no IMU or hidden pose "
            "stabilization."
        )
        for timestamp, label in schedule:
            self.get_logger().info(f"  {timestamp:5.2f} s  {label}")
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
                    "Press Enter to execute the short walking sequence "
                    "(Ctrl+C cancels): "
                )
            except EOFError:
                return 2
        self.publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(
            f"Trajectory published. Final hold begins at "
            f"{schedule[-1][0]:.2f} seconds."
        )
        return 0


def main() -> None:
    if "--self-test" in sys.argv:
        phases = make_phases(cycles=10)
        validate_phases(phases)
        print(f"short-step walk valid: {len(phases)} phases, {len(JOINT_NAMES)} joints")
        return
    rclpy.init()
    node = ShortStepWalkDemo()
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
