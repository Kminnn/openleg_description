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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
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
        locked_positions: dict[int, float] | None = None,
        pitch_target: float | None = None,
        pitch_weight: float = 0.20,
        knee_target: float | None = None,
        knee_weight: float = 0.08,
        joint1_target: float | None = None,
        joint1_weight: float = 0.06,
    ) -> tuple[np.ndarray, float]:
        positions = np.clip(seed.copy(), self.lower, self.upper)
        locked = locked_positions or {}
        for index, value in locked.items():
            if index < 0 or index >= len(self.joints):
                raise IndexError(f"Invalid locked joint index {index}")
            positions[index] = float(np.clip(value, self.lower[index], self.upper[index]))
        free = np.asarray(
            [index for index in range(len(self.joints)) if index not in locked],
            dtype=int,
        )

        for _ in range(50):
            foot, jacobian = self.forward(positions)
            error = target - foot
            pitch_error = 0.0
            knee_error = 0.0
            joint1_error = 0.0
            task_error = error
            task_jacobian = jacobian
            pitch_active = pitch_target is not None and pitch_weight > 1e-6
            knee_active = knee_target is not None and knee_weight > 1e-6
            joint1_active = joint1_target is not None and joint1_weight > 1e-6
            if pitch_active:
                pitch_error = pitch_target - (
                    positions[0] + positions[3] + positions[4]
                )
                pitch_jacobian = np.zeros(6)
                pitch_jacobian[[0, 3, 4]] = 1.0
                task_error = np.append(error, pitch_weight * pitch_error)
                task_jacobian = np.vstack((
                    jacobian,
                    pitch_weight * pitch_jacobian,
                ))
            if knee_active:
                knee_error = knee_target - positions[3]
                knee_jacobian = np.zeros(6)
                knee_jacobian[3] = 1.0
                task_error = np.append(task_error, knee_weight * knee_error)
                task_jacobian = np.vstack((
                    task_jacobian,
                    knee_weight * knee_jacobian,
                ))
            if joint1_active:
                joint1_error = joint1_target - positions[0]
                joint1_jacobian = np.zeros(6)
                joint1_jacobian[0] = 1.0
                task_error = np.append(
                    task_error, joint1_weight * joint1_error
                )
                task_jacobian = np.vstack((
                    task_jacobian,
                    joint1_weight * joint1_jacobian,
                ))
            if (
                np.linalg.norm(error) < 0.0007
                and (not pitch_active or abs(pitch_error) < 0.003)
                and (not knee_active or abs(knee_error) < 0.02)
                and (not joint1_active or abs(joint1_error) < 0.03)
            ) or free.size == 0:
                break
            reduced_jacobian = task_jacobian[:, free]
            damped = (
                reduced_jacobian @ reduced_jacobian.T
                + 0.025 ** 2 * np.eye(task_error.size)
            )
            step = reduced_jacobian.T @ np.linalg.solve(damped, task_error)
            positions[free] = np.clip(
                positions[free] + np.clip(step, -0.08, 0.08),
                self.lower[free],
                self.upper[free],
            )
            for index, value in locked.items():
                positions[index] = float(
                    np.clip(value, self.lower[index], self.upper[index])
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
            "linear_speed": 0.18,
            "angular_speed": 0.75,
            "stride_length": 0.075,
            "step_height": 0.030,
            "command_timeout": 3.00,
            "sprint_multiplier": 2.0,
            "joint3_turn": 0.24,
            "turn_step_height": 0.050,
            "step_frequency": 1.15,
            "sprint_step_frequency": 1.75,
            "update_rate": 30.0,
            "balance_pitch_gain": 0.100,
            "balance_pitch_damping": 0.020,
            "balance_roll_gain": 0.070,
            "balance_roll_damping": 0.015,
            "balance_offset_limit": 0.045,
            "balance_deadband_deg": 10.0,
            "walking_pitch_deg": 7.0,
            "walking_crouch": 0.020,
            "walking_lean_shift": 0.060,
            "walking_knee_bend": -0.350,
            "joint1_pillar_center": 0.300,
            "joint1_pillar_amplitude": 0.250,
            "prelean_time": 0.45,
            "command_ramp_time": 0.35,
            "stop_ramp_time": 2.50,
            "keyboard": True,
            "input_topic": "/openleg/walk_cmd",
            "imu_topic": "/imu",
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
        self.turn_lift = float(self.get_parameter("turn_step_height").value)
        self.step_frequency = float(self.get_parameter("step_frequency").value)
        self.sprint_step_frequency = float(self.get_parameter("sprint_step_frequency").value)
        self.pitch_gain = float(self.get_parameter("balance_pitch_gain").value)
        self.pitch_damping = float(self.get_parameter("balance_pitch_damping").value)
        self.roll_gain = float(self.get_parameter("balance_roll_gain").value)
        self.roll_damping = float(self.get_parameter("balance_roll_damping").value)
        self.balance_limit = float(self.get_parameter("balance_offset_limit").value)
        self.balance_deadband = math.radians(
            float(self.get_parameter("balance_deadband_deg").value)
        )
        self.walking_pitch = math.radians(
            float(self.get_parameter("walking_pitch_deg").value)
        )
        self.walking_crouch = float(self.get_parameter("walking_crouch").value)
        self.walking_lean_shift = float(
            self.get_parameter("walking_lean_shift").value
        )
        self.walking_knee_bend = float(
            self.get_parameter("walking_knee_bend").value
        )
        self.joint1_pillar_center = float(
            self.get_parameter("joint1_pillar_center").value
        )
        self.joint1_pillar_amplitude = float(
            self.get_parameter("joint1_pillar_amplitude").value
        )
        self.command_ramp_time = max(
            float(self.get_parameter("command_ramp_time").value), 0.05
        )
        self.stop_ramp_time = max(
            float(self.get_parameter("stop_ramp_time").value), 0.05
        )
        self.prelean_time = max(
            float(self.get_parameter("prelean_time").value), 0.05
        )

        robot = load_robot()
        self.left = LegKinematics("left", robot)
        self.right = LegKinematics("right", robot)
        self.left_q = self.left.zero.copy()
        self.right_q = self.right.zero.copy()
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, "/leg_controller/joint_trajectory", 10
        )
        self.roll = 0.0
        self.pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.imu_received = False
        self.feedback_time: float | None = None
        self.imu_sub = self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.command_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            lambda message: self.set_command(message.linear.x, message.angular.z),
            10,
        )

        self.forward = 0.0
        self.turn = 0.0
        self.gait_forward = 0.0
        self.gait_turn = 0.0
        self.forward_deadline = 0.0
        self.turn_deadline = 0.0
        self.prelean_direction = 0.0
        self.prelean_started = time.monotonic()
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

    def control_time(self) -> float:
        return self.feedback_time if self.feedback_time is not None else time.monotonic()

    def imu_callback(self, message: Imu) -> None:
        feedback_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.feedback_time is None:
            self.last_update = feedback_time
            self.prelean_started = feedback_time
        self.feedback_time = feedback_time
        q = message.orientation
        self.roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )
        pitch_sine = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(float(np.clip(pitch_sine, -1.0, 1.0)))
        self.roll_rate = message.angular_velocity.x
        self.pitch_rate = message.angular_velocity.y
        if not self.imu_received:
            self.imu_received = True
            self.get_logger().info(
                "Base IMU feedback active; balance corrections now use leg joints only."
            )


    def set_command(self, forward: float | None, turn: float | None) -> None:
        now = self.control_time()
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
        self, leg: LegKinematics, phase: float, forward: float,
        turn: float, balance: np.ndarray, joint3: float, crouch: float,
        lean_shift: float,
    ) -> np.ndarray:
        result = leg.nominal.copy()
        wave = math.sin(phase)
        stride_ratio = float(np.clip(forward / self.speed, -1.35, 1.35))
        forward_weight = min(abs(stride_ratio), 1.0)
        lift_wave = max(0.0, wave)

        if abs(turn) > 1e-4:
            turned_pose = leg.zero.copy()
            turned_pose[2] = joint3
            turn_lift_scale = (
                self.turn_lift
                / 0.050
                * (1.0 - forward_weight)
                * lift_wave
            )
            turned_pose[0] = 0.30 * turn_lift_scale
            turned_pose[3] = -1.05 * turn_lift_scale
            turned_pose[4] = 0.42 * turn_lift_scale
            result = leg.forward(turned_pose)[0]

        # The CAD model faces -base X, so positive W uses a negative-X stride.
        result[0] -= self.stride * stride_ratio * wave
        result[2] += self.lift * forward_weight * lift_wave
        result[2] += crouch
        result[0] += lean_shift
        result[0] += balance[0]
        result[1] += balance[1]
        return result

    def balance_offsets(self) -> np.ndarray:
        if not self.imu_received:
            return np.zeros(2)
        pitch_error = math.copysign(
            max(abs(self.pitch) - self.balance_deadband, 0.0), self.pitch
        )
        roll_error = math.copysign(
            max(abs(self.roll) - self.balance_deadband, 0.0), self.roll
        )
        transition = math.radians(5.0)
        pitch_activation = min(abs(pitch_error) / transition, 1.0)
        roll_activation = min(abs(roll_error) / transition, 1.0)
        forward = float(np.clip(
            self.pitch_gain * pitch_error
            + self.pitch_damping * self.pitch_rate * pitch_activation,
            -self.balance_limit, self.balance_limit,
        ))
        lateral = float(np.clip(
            -self.roll_gain * roll_error
            - self.roll_damping * self.roll_rate * roll_activation,
            -self.balance_limit, self.balance_limit,
        ))
        return np.asarray([forward, lateral])

    def update(self) -> None:
        now = self.control_time()
        dt = min(max(now - self.last_update, 0.0), 0.1)
        self.last_update = now
        with self.lock:
            requested_forward = self.forward if now <= self.forward_deadline else 0.0
            requested_turn = self.turn if now <= self.turn_deadline else 0.0

        requested_direction = 0.0 if abs(requested_forward) <= 1e-4 else math.copysign(1.0, requested_forward)
        if requested_direction != self.prelean_direction:
            self.prelean_direction = requested_direction
            self.prelean_started = now

        gait_requested_forward = requested_forward
        gait_requested_turn = requested_turn
        lean_ratio = 0.0
        if requested_direction != 0.0:
            prelean_progress = float(np.clip(
                (now - self.prelean_started) / self.prelean_time, 0.0, 1.0
            ))
            lean_ratio = (
                float(np.clip(requested_forward / self.speed, -1.0, 1.0))
                * prelean_progress
            )


        forward_ramp_time = (
            self.stop_ramp_time
            if abs(gait_requested_forward) <= 1e-4
            else self.command_ramp_time
        )
        turn_ramp_time = (
            self.stop_ramp_time
            if abs(gait_requested_turn) <= 1e-4
            else self.command_ramp_time
        )
        forward_step = self.speed * dt / forward_ramp_time
        turn_step = self.turn_speed * dt / turn_ramp_time
        self.gait_forward += float(np.clip(
            gait_requested_forward - self.gait_forward, -forward_step, forward_step
        ))
        self.gait_turn += float(np.clip(
            gait_requested_turn - self.gait_turn, -turn_step, turn_step
        ))
        forward = self.gait_forward
        turn = self.gait_turn
        if requested_direction == 0.0:
            lean_ratio = float(np.clip(forward / self.speed, -1.0, 1.0))
        moving = abs(forward) > 1e-4 or abs(turn) > 1e-4
        if moving:
            command_ratio = max(
                abs(forward / self.speed),
                abs(turn / self.turn_speed),
            )
            sprint_range = max(self.sprint_multiplier - 1.0, 1e-6)
            sprint_ratio = float(np.clip(
                (command_ratio - 1.0) / sprint_range,
                0.0, 1.0,
            ))
            frequency = (
                self.step_frequency
                + sprint_ratio * (self.sprint_step_frequency - self.step_frequency)
            )
            startup_frequency_scale = 0.70 + 0.30 * min(command_ratio, 1.0)
            frequency *= startup_frequency_scale
            self.phase = (self.phase + 2.0 * math.pi * frequency * dt) % (2.0 * math.pi)
        else:
            self.phase = 0.0

        left_phase = self.phase
        right_phase = self.phase + math.pi
        turn_ratio = float(np.clip(turn / self.turn_speed, -1.35, 1.35))
        left_joint3 = (
            -self.joint3_turn
            * turn_ratio
            * 0.5
            * (1.0 + math.cos(left_phase))
        )
        right_joint3 = (
            self.joint3_turn
            * turn_ratio
            * 0.5
            * (1.0 + math.cos(right_phase))
        )

        # Positive joint-chain pitch produces the observed forward body lean.
        target_pitch = self.walking_pitch * lean_ratio
        walking_lean_shift = self.walking_lean_shift * lean_ratio
        walking_knee_bend = self.walking_knee_bend * abs(lean_ratio)
        walking_crouch = self.walking_crouch * abs(lean_ratio)
        left_wave = math.sin(left_phase)
        right_wave = math.sin(right_phase)
        forward_active = abs(forward) > 1e-4
        left_swing = forward_active and left_wave > 1e-4
        right_swing = forward_active and right_wave > 1e-4
        left_pitch_target = None if left_swing else target_pitch
        right_pitch_target = None if right_swing else target_pitch
        left_knee_target = (
            None if left_swing
            else walking_knee_bend * (1.0 - max(0.0, -left_wave))
        )
        right_knee_target = (
            None if right_swing
            else walking_knee_bend * (1.0 - max(0.0, -right_wave))
        )
        forward_weight = min(abs(forward / self.speed), 1.0)
        travel_direction = math.copysign(1.0, forward) if forward_active else 0.0
        left_joint1_target = None
        right_joint1_target = None
        if forward_active:
            left_joint1_target = (
                self.joint1_pillar_center
                + self.joint1_pillar_amplitude
                * travel_direction * left_wave * forward_weight
            )
            right_joint1_target = (
                self.joint1_pillar_center
                + self.joint1_pillar_amplitude
                * travel_direction * right_wave * forward_weight
            )
        balance = self.balance_offsets()
        left_balance = balance.copy()
        right_balance = balance.copy()
        left_locked = {2: left_joint3}
        right_locked = {2: right_joint3}
        if abs(turn) > 1e-4 and abs(forward) <= 1e-4:
            left_locked[1] = 0.0
            right_locked[1] = 0.0
            left_balance[1] = 0.0
            right_balance[1] = 0.0

        self.left_q, left_error = self.left.solve(
            self.target(
                self.left, left_phase, forward, turn, left_balance, left_joint3,
                walking_crouch, walking_lean_shift,
            ),
            self.left_q,
            left_locked,
            left_pitch_target,
            0.20,
            left_knee_target,
            0.08,
            left_joint1_target,
        )
        self.right_q, right_error = self.right.solve(
            self.target(
                self.right, right_phase, forward, turn, right_balance, right_joint3,
                walking_crouch, walking_lean_shift,
            ),
            self.right_q,
            right_locked,
            right_pitch_target,
            0.20,
            right_knee_target,
            0.08,
            right_joint1_target,
        )
        if max(left_error, right_error) > 0.008:
            self.get_logger().warning(
                f"IK residual high: L={left_error:.3f} m R={right_error:.3f} m",
                throttle_duration_sec=2.0,
            )

        message = JointTrajectory()
        point = JointTrajectoryPoint()
        message.joint_names = self.left.names + self.right.names
        point.positions = np.concatenate([self.left_q, self.right_q]).tolist()
        point.time_from_start.nanosec = 100_000_000
        message.points = [point]
        self.trajectory_pub.publish(message)
        if self.exit_requested:
            rclpy.shutdown()

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

        pitch_target = math.radians(7.0)
        pitch_positions, pitch_error = leg.solve(
            leg.nominal + np.asarray([0.060, 0.0, 0.020]),
            leg.zero, {2: 0.0}, pitch_target, 0.20, -0.350,
        )
        pitch_result = float(
            pitch_positions[0] + pitch_positions[3] + pitch_positions[4]
        )
        print(
            f"{side} lean: residual={pitch_error:.6f}, "
            f"pitch={math.degrees(pitch_result):.3f} deg"
        )
        if pitch_error > 0.008 or abs(pitch_result - pitch_target) > 0.030:
            return 1

        joint3_target = -0.20 if side == "left" else 0.20
        turn_pose = leg.zero.copy()
        turn_pose[2] = joint3_target
        turn_target = leg.forward(turn_pose)[0]
        turn_positions, turn_error = leg.solve(
            turn_target, leg.zero, {1: 0.0, 2: joint3_target}
        )
        print(
            f"{side} turn: residual={turn_error:.6f}, "
            f"joint2={turn_positions[1]:.4f}, joint3={turn_positions[2]:.4f}"
        )
        if turn_error > 0.001 or abs(turn_positions[1]) > 1e-9:
            return 1
        if abs(turn_positions[2] - joint3_target) > 1e-9:
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
