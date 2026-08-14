#!/usr/bin/env python3
"""Experimental symmetric bound that previously produced backward travel."""

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


JOINT_VELOCITY = np.asarray([
    10.47198, 10.47198, 4.18879, 10.47198, 10.47198, 4.18879,
    10.47198, 10.47198, 4.18879, 10.47198, 10.47198, 4.18879,
])
MIN_SPEED_SCALE = 0.8
MAX_SPEED_SCALE = 1.1
SPLINE_PEAK_FACTOR = 1.0


@dataclass(frozen=True)
class Phase:
    label: str
    duration: float
    positions: np.ndarray


def pose(*values: float) -> np.ndarray:
    return np.asarray(values, dtype=float)


# Original symmetric bound poses. Gazebo produced bilateral flight but net -X travel.
NARROW_STAND = pose(
    0.125082, 0.112589, 0.0, -0.342976, 0.248767, 0.0,
    0.125082, 0.112589, 0.0, -0.342976, 0.248767, 0.0,
)
CENTER_CROUCH = pose(
    0.672395, 0.140000, 0.0, -1.100000, 0.427605, 0.0,
    0.672395, 0.140000, 0.0, -1.100000, 0.427605, 0.0,
)
TAKEOFF = pose(
    0.287164, 0.116519, 0.0, -0.341255, 0.054093, 0.0,
    0.275915, 0.116379, 0.0, -0.312142, 0.036238, 0.0,
)
FLIGHT_FLEX = pose(
    0.495547, 0.128770, 0.0, -0.855903, 0.360485, 0.0,
    0.491963, 0.128600, 0.0, -0.846185, 0.354329, 0.0,
)


def make_phases(cycles: int = 5, speed_scale: float = 1.0) -> tuple[Phase, ...]:
    cycles = max(1, min(10, int(cycles)))
    speed_scale = float(np.clip(speed_scale, MIN_SPEED_SCALE, MAX_SPEED_SCALE))
    phases: list[Phase] = []

    def add(label: str, duration: float, positions: np.ndarray) -> None:
        phases.append(Phase(label, duration / speed_scale, positions))

    add("settle in narrow stance", 3.00, NARROW_STAND)
    add("load centered crouch", 2.00, CENTER_CROUCH)
    add("initial push", 0.15, TAKEOFF)
    add("initial flight", 0.08, FLIGHT_FLEX)
    for cycle in range(1, cycles + 1):
        add(f"cycle {cycle}: landing", 0.08, TAKEOFF)
        add(f"cycle {cycle}: absorb", 0.08, FLIGHT_FLEX)
        add(f"cycle {cycle}: push", 0.08, TAKEOFF)
        add(f"cycle {cycle}: flight", 0.08, FLIGHT_FLEX)
    add("final landing", 0.08, TAKEOFF)
    add("settle into landing crouch", 0.20, FLIGHT_FLEX)
    add("hold landing crouch", 2.00, FLIGHT_FLEX)
    return tuple(phases)


def validate_phases(phases: tuple[Phase, ...]) -> None:
    if len(JOINT_NAMES) != 12 or len(set(JOINT_NAMES)) != 12:
        raise ValueError("Expected 12 unique motor joint names")
    previous = np.zeros(12)
    for phase in phases:
        if phase.positions.shape != (12,) or not np.all(np.isfinite(phase.positions)):
            raise ValueError(f"Invalid positions in phase: {phase.label}")
        if phase.duration <= 0.0:
            raise ValueError(f"Invalid duration in phase: {phase.label}")
        if np.any(phase.positions < JOINT_LOWER) or np.any(phase.positions > JOINT_UPPER):
            raise ValueError(f"Joint limit exceeded in phase: {phase.label}")
        peak_velocity = (
            np.abs(phase.positions - previous)
            * SPLINE_PEAK_FACTOR
            / phase.duration
        )
        if np.any(peak_velocity > JOINT_VELOCITY + 1e-6):
            joint = int(np.argmax(peak_velocity / JOINT_VELOCITY))
            raise ValueError(
                f"Velocity limit exceeded by {JOINT_NAMES[joint]} in phase: "
                f"{phase.label}"
            )
        previous = phase.positions


def seconds_to_point_time(point: JointTrajectoryPoint, seconds: float) -> None:
    whole = int(seconds)
    point.time_from_start.sec = whole
    point.time_from_start.nanosec = int(round((seconds - whole) * 1e9))
    if point.time_from_start.nanosec == 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec = 0


class BackwardBoundDemo(Node):
    def __init__(self) -> None:
        super().__init__("backward_bound_demo")
        self.declare_parameter("cycles", 5)
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
        self.get_logger().warning(
            f"Experimental {cycles}-cycle symmetric bound ready. This is the "
            "preserved sequence that hops toward -X (backward), and it can "
            "fall after the command ends. No IMU or hidden stabilization."
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
                    "Press Enter to execute the backward bound "
                    "(Ctrl+C cancels): "
                )
            except EOFError:
                return 2
        self.publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(
            f"Trajectory published. Sequence completes at "
            f"{schedule[-1][0]:.2f} seconds."
        )
        return 0


def main() -> None:
    if "--self-test" in sys.argv:
        phases = make_phases(cycles=5)
        fastest_phases = make_phases(cycles=10, speed_scale=MAX_SPEED_SCALE)
        validate_phases(phases)
        validate_phases(fastest_phases)
        flights = sum(phase.label.endswith(": flight") for phase in fastest_phases)
        if flights != 10:
            raise ValueError(f"Expected 10 repeated flight phases, got {flights}")
        duration = sum(phase.duration for phase in phases)
        print(
            f"backward bound valid: {len(phases)} phases, "
            f"{flights} maximum-test flight phases, {len(JOINT_NAMES)} joints, "
            f"default_duration={duration:.2f} seconds"
        )
        return
    rclpy.init()
    node = BackwardBoundDemo()
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
