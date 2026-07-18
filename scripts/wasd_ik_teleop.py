#!/usr/bin/env python3
"""WASD teleoperation with a URDF-derived numerical IK gait for OpenLeg."""

from __future__ import annotations

import math
import os
import select
import sys
import termios
import threading
import time
import tty
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def vector(text: str | None) -> np.ndarray:
    return np.asarray([float(value) for value in (text or "0 0 0").split()])


def rpy_rotation(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.asarray([
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ])


def origin_transform(joint: ET.Element) -> np.ndarray:
    result = np.eye(4)
    origin = joint.find("origin")
    if origin is not None:
        result[:3, 3] = vector(origin.get("xyz"))
        result[:3, :3] = rpy_rotation(vector(origin.get("rpy")))
    return result


@dataclass
class JointDefinition:
    name: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class LegKinematics:
    """Forward kinematics and damped-least-squares IK for one leg."""

    def __init__(self, side: str, robot: ET.Element):
        elements = {item.get("name"): item for item in robot.findall("joint")}
        self.base = origin_transform(elements[f"{side}_base_fixed_joint"])
        self.joints = []
        for index in range(1, 7):
            name = f"{side}_joint{index}"
            element = elements[name]
            limit = element.find("limit")
            axis = element.find("axis")
            if limit is None or axis is None:
                raise ValueError(f"{name} needs an axis and limits")
            self.joints.append(JointDefinition(
                name,
                origin_transform(element),
                vector(axis.get("xyz")),
                float(limit.get("lower")),
                float(limit.get("upper")),
            ))
        self.names = [joint.name for joint in self.joints]
        self.lower = np.asarray([joint.lower for joint in self.joints])
        self.upper = np.asarray([joint.upper for joint in self.joints])
        self.zero = np.clip(np.zeros(6), self.lower, self.upper)
        self.nominal = self.forward(self.zero)[0]

    def forward(self, positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        transform = self.base.copy()
        points, axes = [], []
        for joint, angle in zip(self.joints, positions):
            transform = transform @ joint.origin
            points.append(transform[:3, 3].copy())
            axes.append(transform[:3, :3] @ joint.axis)
            rotation = np.eye(4)
            rotation[:3, :3] = axis_rotation(joint.axis, float(angle))
            transform = transform @ rotation
        foot = transform[:3, 3].copy()
        jacobian = np.column_stack([
            np.cross(axis, foot - point) for point, axis in zip(points, axes)
        ])
        return foot, jacobian

    def solve(
        self,
        target: np.ndarray,
        seed: np.ndarray,
        joint3_target: float = 0.0,
    ) -> tuple[np.ndarray, float]:
        positions = np.clip(seed.copy(), self.lower, self.upper)
        for _ in range(30):
            foot, jacobian = self.forward(positions)
            error = target - foot
            if (
                np.linalg.norm(error) < 0.0007
                and abs(joint3_target - positions[2]) < 0.01
            ):
                break
            damped = jacobian @ jacobian.T + 0.025 ** 2 * np.eye(3)
            pseudo_inverse = jacobian.T @ np.linalg.solve(damped, np.eye(3))
            primary = pseudo_inverse @ error
            preferred = positions.copy()
            preferred[2] = joint3_target
            nullspace = np.eye(6) - pseudo_inverse @ jacobian
            secondary = 0.20 * nullspace @ (preferred - positions)
            positions = np.clip(
                positions + np.clip(primary + secondary, -0.06, 0.06),
                self.lower,
                self.upper,
            )
        error = float(np.linalg.norm(target - self.forward(positions)[0]))
        return positions, error


def load_robot() -> ET.Element:
    share = get_package_share_directory("openleg_description")
    document = xacro.process_file(
        os.path.join(share, "urdf", "main.urdf.xacro"),
        mappings={
            "use_gazebo": "false",
            "controllers_file": os.path.join(share, "config", "controllers.yaml"),
        },
    )
    return ET.fromstring(document.toxml())


class WasdIkTeleop(Node):
    def __init__(self) -> None:
        super().__init__("wasd_ik_teleop")
        defaults = {
            "linear_speed": 0.12,
            "angular_speed": 0.55,
            "stride_length": 0.050,
            "step_height": 0.030,
            "command_timeout": 0.90,
            "sprint_multiplier": 2.0,
            "joint3_turn": 0.16,
            "turn_stride_length": 0.020,
            "turn_step_height": 0.025,
            "update_rate": 30.0,
            "stabilization_gain": 4.0,
            "height_gain": 4.0,
            "max_vertical_speed": 0.15,
            "keyboard": True,
            "input_topic": "/openleg/walk_cmd",
            "model_velocity_topic": "/model/openleg/cmd_vel",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.speed = float(self.get_parameter("linear_speed").value)
        self.turn_speed = float(self.get_parameter("angular_speed").value)
        self.stride = float(self.get_parameter("stride_length").value)
        self.lift = float(self.get_parameter("step_height").value)
        self.timeout = float(self.get_parameter("command_timeout").value)
        self.sprint_multiplier = float(self.get_parameter("sprint_multiplier").value)
        self.joint3_turn = float(self.get_parameter("joint3_turn").value)
        self.turn_stride = float(self.get_parameter("turn_stride_length").value)
        self.turn_lift = float(self.get_parameter("turn_step_height").value)
        self.stabilization_gain = float(self.get_parameter("stabilization_gain").value)
        self.height_gain = float(self.get_parameter("height_gain").value)
        self.max_vertical = float(self.get_parameter("max_vertical_speed").value)

        robot = load_robot()
        self.left = LegKinematics("left", robot)
        self.right = LegKinematics("right", robot)
        self.left_q = self.left.zero.copy()
        self.right_q = self.right.zero.copy()
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, "/leg_controller/joint_trajectory", 10
        )
        self.velocity_pub = self.create_publisher(
            Twist, str(self.get_parameter("model_velocity_topic").value), 10
        )
        self.pose_z = None
        self.target_z = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.target_yaw = None
        self.odometry_sub = self.create_subscription(
            Odometry, "/model/openleg/odometry", self.odometry_callback, 10
        )
        self.command_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            lambda message: self.set_command(message.linear.x, message.angular.z),
            10,
        )

        self.forward = 0.0
        self.turn = 0.0
        self.forward_deadline = 0.0
        self.turn_deadline = 0.0
        self.phase = 0.0
        self.last_update = time.monotonic()
        self.exit_requested = False
        self.lock = threading.Lock()
        rate = float(self.get_parameter("update_rate").value)
        self.timer = self.create_timer(1.0 / rate, self.update)

        if bool(self.get_parameter("keyboard").value) and sys.stdin.isatty():
            threading.Thread(target=self.keyboard_loop, daemon=True).start()
            self.get_logger().info(
                "W/S forward/back, A/D turn, Shift+key sprint, SPACE stop, Q quit."
            )
        elif bool(self.get_parameter("keyboard").value):
            self.get_logger().warning(
                "No terminal; publish Twist to /openleg/walk_cmd instead."
            )
        self.get_logger().info(
            f"IK loaded from current URDF; feet L={np.round(self.left.nominal, 4)}, "
            f"R={np.round(self.right.nominal, 4)}"
        )

    def odometry_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        q = pose.orientation
        self.roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )
        pitch_sine = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(float(np.clip(pitch_sine, -1.0, 1.0)))
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose_z = pose.position.z
        if self.target_z is None:
            self.target_z = self.pose_z
            self.target_yaw = self.yaw
            self.get_logger().info(
                f"Stabilizing pelvis at z={self.target_z:.3f} m, roll=0, pitch=0"
            )


    def set_command(self, forward: float | None, turn: float | None) -> None:
        now = time.monotonic()
        with self.lock:
            if forward is not None:
                maximum = self.speed * self.sprint_multiplier
                self.forward = float(np.clip(forward, -maximum, maximum))
                self.forward_deadline = now + self.timeout
            if turn is not None:
                maximum = self.turn_speed * self.sprint_multiplier
                self.turn = float(np.clip(turn, -maximum, maximum))
                self.turn_deadline = now + self.timeout

    def keyboard_loop(self) -> None:
        descriptor = sys.stdin.fileno()
        previous = termios.tcgetattr(descriptor)
        try:
            tty.setcbreak(descriptor)
            while rclpy.ok() and not self.exit_requested:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                raw_key = sys.stdin.read(1)
                boost = self.sprint_multiplier if raw_key.isupper() else 1.0
                key = raw_key.lower()
                if key == "w":
                    self.set_command(self.speed * boost, None)
                elif key == "s":
                    self.set_command(-self.speed * boost, None)
                elif key == "a":
                    self.set_command(None, self.turn_speed * boost)
                elif key == "d":
                    self.set_command(None, -self.turn_speed * boost)
                elif key in (" ", "x"):
                    self.set_command(0.0, 0.0)
                elif key == "q":
                    self.set_command(0.0, 0.0)
                    self.exit_requested = True
        finally:
            termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)

    def target(
        self, leg: LegKinematics, phase: float, forward: float, turn: float
    ) -> np.ndarray:
        result = leg.nominal.copy()
        wave = math.sin(phase)
        stride_ratio = float(np.clip(forward / self.speed, -1.4, 1.4))
        turn_ratio = float(np.clip(turn / self.turn_speed, -1.4, 1.4))
        forward_weight = min(abs(stride_ratio), 1.0)

        result[0] += self.stride * stride_ratio * wave
        result[0] += (
            self.turn_stride
            * abs(turn_ratio)
            * (1.0 - forward_weight)
            * wave
        )
        lift = self.turn_lift + (self.lift - self.turn_lift) * forward_weight
        result[2] += lift * max(0.0, wave)
        return result

    def update(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self.last_update, 0.0), 0.1)
        self.last_update = now
        with self.lock:
            forward = self.forward if now <= self.forward_deadline else 0.0
            turn = self.turn if now <= self.turn_deadline else 0.0

        velocity = Twist()
        velocity.linear.x = forward
        velocity.angular.z = turn
        if self.target_yaw is not None:
            self.target_yaw += turn * dt
        if self.pose_z is not None and self.target_z is not None:
            velocity.linear.z = float(np.clip(
                self.height_gain * (self.target_z - self.pose_z),
                -self.max_vertical,
                self.max_vertical,
            ))
            velocity.angular.x = float(np.clip(
                -self.stabilization_gain * self.roll, -1.5, 1.5
            ))
            yaw_error = math.atan2(
                math.sin(self.target_yaw - self.yaw),
                math.cos(self.target_yaw - self.yaw),
            )
            velocity.angular.z = float(np.clip(
                turn + self.stabilization_gain * yaw_error, -1.5, 1.5
            ))
            velocity.angular.y = float(np.clip(
                -self.stabilization_gain * self.pitch, -1.5, 1.5
            ))
        self.velocity_pub.publish(velocity)
        if abs(forward) > 1e-4 or abs(turn) > 1e-4:
            pace = (
                2.2
                + 3.0 * abs(forward / self.speed)
                + 1.5 * abs(turn / self.turn_speed)
            )
            self.phase = (self.phase + pace * dt) % (2.0 * math.pi)
            turn_ratio = float(np.clip(turn / self.turn_speed, -1.0, 1.0))
            left_joint3 = -self.joint3_turn * turn_ratio
            right_joint3 = self.joint3_turn * turn_ratio
            self.left_q, left_error = self.left.solve(
                self.target(self.left, self.phase, forward, turn),
                self.left_q,
                left_joint3,
            )
            self.right_q, right_error = self.right.solve(
                self.target(self.right, self.phase + math.pi, forward, turn),
                self.right_q,
                right_joint3,
            )
            if max(left_error, right_error) > 0.008:
                self.get_logger().warning(
                    f"IK residual high: L={left_error:.3f} m R={right_error:.3f} m",
                    throttle_duration_sec=2.0,
                )
        else:
            self.left_q *= 0.82
            self.right_q *= 0.82
            self.phase = 0.0

        message = JointTrajectory()
        point = JointTrajectoryPoint()
        message.joint_names = self.left.names + self.right.names
        point.positions = np.concatenate([self.left_q, self.right_q]).tolist()
        point.time_from_start.nanosec = 100_000_000
        message.points = [point]
        self.trajectory_pub.publish(message)
        if self.exit_requested:
            rclpy.shutdown()

    def stop(self) -> None:
        for _ in range(3):
            self.velocity_pub.publish(Twist())


def self_test() -> int:
    robot = load_robot()
    for side in ("left", "right"):
        leg = LegKinematics(side, robot)
        target = leg.nominal + np.asarray([0.015, 0.0, 0.010])
        positions, error = leg.solve(target, leg.zero)
        print(
            f"{side}: nominal={np.round(leg.nominal, 5)}, "
            f"residual={error:.6f}, q={np.round(positions, 4)}"
        )
        if error > 0.003 or np.any(positions < leg.lower) or np.any(positions > leg.upper):
            return 1
    return 0


def main() -> None:
    if "--self-test" in sys.argv:
        raise SystemExit(self_test())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WasdIkTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
