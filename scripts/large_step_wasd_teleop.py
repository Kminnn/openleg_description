#!/usr/bin/env python3
"""Separate WASD controller with longer steps and larger per-step turns."""

from __future__ import annotations

import sys

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

from directional_step_gait import (
    make_backward_phases as make_base_backward_phases,
    make_turn_phases as make_base_turn_phases,
)
from short_step_walk_demo import (
    JOINT_NAMES,
    Phase,
    make_phases as make_base_forward_phases,
    validate_phases,
)
from wasd_ik_teleop import (
    WasdIkTeleop,
    build_phase_trajectory,
    make_continuation_template,
    retime_fixed_phases,
)


STEP_DISTANCE_MM = 35
TURN_ANGLE_DEG = 10


def pose(*values: float) -> np.ndarray:
    return np.asarray(values, dtype=float)


# Offline IK solutions against the current URDF. Both feet remain contact
# compatible throughout double support. The worst solved backward contact
# residual is 0.023 mm; all other listed targets solve to numerical precision.
F_RIGHT_LIFT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.284278, -0.191289, -0.049387, -0.721692, 0.427603, 0.197692,
)
F_RIGHT_SWING = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.338142, -0.188159, -0.059176, -0.684265, 0.383210, 0.197339,
)
F_RIGHT_PRECONTACT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.233129, -0.187914, -0.038353, -0.418514, 0.220503, 0.192282,
)
F_RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.197826, -0.187826, -0.031355, -0.328984, 0.165613, 0.190594,
)
F_TRANSFER_RIGHT_25 = pose(
    0.167577, 0.095610, 0.013051, -0.499740, 0.364286, -0.096295,
    0.246701, -0.093788, -0.020487, -0.503837, 0.289597, 0.096192,
)
F_TRANSFER_RIGHT_50 = pose(
    0.167965, 0.000225, 0.000012, -0.547729, 0.411264, -0.000025,
    0.246612, 0.000174, 0.000018, -0.549633, 0.334521, 0.000025,
)
F_TRANSFER_RIGHT_75 = pose(
    0.134402, -0.095679, -0.009884, -0.500906, 0.398478, 0.096386,
    0.212463, 0.094799, 0.017298, -0.500899, 0.320755, -0.096161,
)
F_SHIFT_RIGHT = pose(
    0.051650, -0.190936, -0.003843, -0.323415, 0.303635, 0.191174,
    0.128376, 0.189946, 0.018327, -0.320067, 0.224934, -0.190619,
)
F_LEFT_LIFT = pose(
    0.239346, -0.193460, -0.041241, -0.740480, 0.427603, 0.197988,
    0.128374, 0.189946, 0.018327, -0.320061, 0.224931, -0.190619,
)
F_LEFT_SWING = pose(
    0.347678, -0.187550, -0.060943, -0.679082, 0.368640, 0.197291,
    0.128374, 0.189946, 0.018327, -0.320061, 0.224931, -0.190619,
)
F_LEFT_PRECONTACT = pose(
    0.241100, -0.187564, -0.039832, -0.410422, 0.204573, 0.192238,
    0.128374, 0.189946, 0.018327, -0.320061, 0.224931, -0.190619,
)
F_LEFT_PLANT = pose(
    0.204849, -0.187562, -0.032659, -0.318723, 0.148448, 0.190549,
    0.128374, 0.189946, 0.018327, -0.320061, 0.224931, -0.190619,
)
F_TRANSFER_LEFT_25 = pose(
    0.255784, -0.093580, -0.021332, -0.498567, 0.275282, 0.096173,
    0.179375, 0.095417, 0.014171, -0.501548, 0.354348, -0.096261,
)
F_TRANSFER_LEFT_50 = pose(
    0.256424, 0.000166, 0.000018, -0.546173, 0.321249, 0.000032,
    0.180246, 0.000218, 0.000014, -0.550794, 0.402047, -0.000018,
)
F_TRANSFER_LEFT_75 = pose(
    0.222712, 0.094585, 0.018261, -0.498653, 0.308304, -0.096128,
    0.147342, -0.095524, -0.011117, -0.505733, 0.390424, 0.096367,
)
F_SHIFT_LEFT = pose(
    0.138999, 0.189687, 0.020325, -0.318942, 0.213375, -0.190561,
    0.066400, -0.190815, -0.006640, -0.332889, 0.298627, 0.191129,
)
F_RIGHT_LIFT_PERIODIC = pose(
    0.138996, 0.189687, 0.020325, -0.318936, 0.213371, -0.190560,
    0.251977, -0.192893, -0.043652, -0.743085, 0.417799, 0.197946,
)

B_RIGHT_SWING = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.240712, -0.189393, -0.054923, -0.752986, 0.409036, 0.197303,
)
B_RIGHT_PRECONTACT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.071992, -0.192194, -0.007921, -0.405236, 0.365512, 0.192894,
)
B_RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.036191, -0.191018, -0.000910, -0.312087, 0.307484, 0.191218,
)
B_TRANSFER_RIGHT_25 = pose(
    0.201820, 0.095010, 0.016295, -0.502126, 0.332579, -0.096194,
    0.121069, -0.095821, -0.008611, -0.495014, 0.405858, 0.096405,
)
B_TRANSFER_RIGHT_50 = pose(
    0.236434, 0.000181, 0.000018, -0.552146, 0.347212, 0.000018,
    0.155333, 0.000232, 0.000010, -0.543709, 0.419876, -0.000032,
)
B_TRANSFER_RIGHT_75 = pose(
    0.237207, -0.093997, -0.019601, -0.508059, 0.303275, 0.096212,
    0.155394, 0.095795, 0.011891, -0.496906, 0.373581, -0.096329,
)
B_SHIFT_RIGHT = pose(
    0.190095, -0.188103, -0.029918, -0.337480, 0.181709, 0.190638,
    0.105310, 0.190429, 0.013977, -0.317659, 0.245181, -0.190735,
)
B_LEFT_LIFT = pose(
    0.372966, -0.185911, -0.066282, -0.724154, 0.279842, 0.197477,
    0.105312, 0.190429, 0.013977, -0.317664, 0.245184, -0.190735,
)
B_LEFT_SWING = pose(
    0.227729, -0.190000, -0.052448, -0.749502, 0.418316, 0.197220,
    0.105312, 0.190429, 0.013977, -0.317664, 0.245184, -0.190735,
)
B_LEFT_PRECONTACT = pose(
    0.056874, -0.192334, -0.005034, -0.394917, 0.370032, 0.192938,
    0.105312, 0.190429, 0.013977, -0.317664, 0.245184, -0.190735,
)
B_LEFT_PLANT = pose(
    0.019944, -0.191053, 0.002175, -0.298698, 0.310046, 0.191263,
    0.105312, 0.190429, 0.013977, -0.317664, 0.245184, -0.190735,
)
B_TRANSFER_LEFT_25 = pose(
    0.107332, -0.095949, -0.007298, -0.488021, 0.412539, 0.096424,
    0.190790, 0.095216, 0.015253, -0.502342, 0.343777, -0.096228,
)
B_TRANSFER_LEFT_50 = pose(
    0.142225, 0.000240, 0.000007, -0.538356, 0.427605, -0.000040,
    0.225897, 0.000188, 0.000017, -0.553727, 0.359330, 0.000011,
)
B_TRANSFER_LEFT_75 = pose(
    0.142822, 0.095970, 0.010692, -0.493033, 0.382223, -0.096363,
    0.227311, -0.094204, -0.018675, -0.511257, 0.316327, 0.096231,
)
B_SHIFT_LEFT = pose(
    0.092862, 0.190644, 0.011623, -0.314102, 0.253850, -0.190794,
    0.181710, -0.188387, -0.028356, -0.344338, 0.196809, 0.190682,
)
B_RIGHT_LIFT_PERIODIC = pose(
    0.092864, 0.190643, 0.011623, -0.314107, 0.253853, -0.190794,
    0.363438, -0.186563, -0.064536, -0.729537, 0.294610, 0.197520,
)

TL_RIGHT_CLEAR = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.298676, -0.146842, -0.217394, -0.657376, 0.406904, 0.202396,
)
TL_RIGHT_PRECONTACT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.205798, -0.159492, -0.201783, -0.434954, 0.293901, 0.194785,
)
TL_RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.174735, -0.162590, -0.196420, -0.354317, 0.243933, 0.191734,
)
TL_TRANSFER_25 = pose(
    0.184682, 0.099443, -0.006651, -0.502042, 0.347475, -0.096366,
    0.223789, -0.062348, -0.166967, -0.513244, 0.333597, 0.097189,
)
TL_TRANSFER_50 = pose(
    0.206940, 0.009305, -0.042662, -0.554280, 0.378634, -0.000137,
    0.233219, 0.029534, -0.127537, -0.552470, 0.348923, 0.001098,
)
TL_TRANSFER_75 = pose(
    0.200386, -0.081947, -0.080032, -0.514854, 0.352398, 0.096267,
    0.213738, 0.116994, -0.088838, -0.499575, 0.306943, -0.094793,
)
TL_SHIFT_RIGHT = pose(
    0.153576, -0.177168, -0.107919, -0.356338, 0.252212, 0.191081,
    0.147856, 0.200918, -0.062902, -0.314849, 0.183877, -0.188757,
)
TL_LEFT_LIFT = pose(
    0.279797, -0.169972, -0.131164, -0.662144, 0.416687, 0.198897,
    0.147856, 0.200918, -0.062901, -0.314852, 0.183879, -0.188757,
)
TL_LEFT_CLEAR = pose(
    0.231793, -0.216624, 0.043292, -0.652447, 0.418905, 0.199837,
    0.147856, 0.200918, -0.062901, -0.314852, 0.183879, -0.188757,
)
TL_LEFT_PRECONTACT = pose(
    0.138656, -0.205327, 0.064088, -0.435684, 0.313434, 0.194174,
    0.147856, 0.200918, -0.062901, -0.314852, 0.183879, -0.188757,
)
TL_LEFT_PLANT = pose(
    0.107409, -0.200222, 0.070705, -0.355173, 0.263802, 0.191123,
    0.147856, 0.200918, -0.062901, -0.314852, 0.183879, -0.188757,
)

TR_RIGHT_CLEAR = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.203292, -0.235553, 0.132507, -0.644162, 0.416899, 0.200602,
)
TR_RIGHT_PRECONTACT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.112458, -0.216145, 0.154148, -0.433041, 0.318247, 0.194877,
)
TR_RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.081571, -0.208272, 0.160955, -0.351956, 0.268217, 0.191824,
)
TR_TRANSFER_25 = pose(
    0.185391, 0.091465, 0.036110, -0.501815, 0.350630, -0.096368,
    0.150052, -0.120279, 0.138637, -0.510359, 0.376019, 0.097243,
)
TR_TRANSFER_50 = pose(
    0.200180, -0.008360, 0.042797, -0.553846, 0.384994, -0.000124,
    0.169528, -0.023030, 0.128879, -0.548633, 0.409065, 0.001025,
)
TR_TRANSFER_75 = pose(
    0.177984, -0.106794, 0.050062, -0.514134, 0.361828, 0.096311,
    0.151919, 0.078676, 0.118831, -0.494155, 0.383621, -0.095077,
)
TR_SHIFT_RIGHT = pose(
    0.106695, -0.200217, 0.070843, -0.354949, 0.264277, 0.191168,
    0.079310, 0.183225, 0.094474, -0.304558, 0.273726, -0.189320,
)
TR_LEFT_LIFT = pose(
    0.230062, -0.215262, 0.043963, -0.647877, 0.413710, 0.199360,
    0.079312, 0.183225, 0.094475, -0.304563, 0.273727, -0.189320,
)
TR_LEFT_CLEAR = pose(
    0.277447, -0.172591, -0.131344, -0.651330, 0.405805, 0.202109,
    0.079312, 0.183225, 0.094475, -0.304563, 0.273727, -0.189320,
)
TR_LEFT_PRECONTACT = pose(
    0.185279, -0.176779, -0.113869, -0.436527, 0.301344, 0.194171,
    0.079312, 0.183225, 0.094475, -0.304563, 0.273727, -0.189320,
)
TR_LEFT_PLANT = pose(
    0.154161, -0.177144, -0.108027, -0.356237, 0.251535, 0.191120,
    0.079312, 0.183225, 0.094475, -0.304563, 0.273727, -0.189320,
)


def action(label: str) -> str:
    return label.split(": ", 1)[-1].lower()


def remap_walk(phases, mapping, initial_lift, periodic_lift) -> tuple[Phase, ...]:
    result = []
    for phase in phases:
        key = action(phase.label)
        positions = mapping.get(key, phase.positions)
        if key == "lift right foot vertically":
            positions = periodic_lift if phase.label.startswith("cycle ") else initial_lift
        result.append(Phase(phase.label, phase.duration, positions))
    return tuple(result)


FORWARD = {
    "swing right foot forward": F_RIGHT_SWING,
    "hold right foot clear": F_RIGHT_SWING,
    "lower right foot to pre-contact": F_RIGHT_PRECONTACT,
    "softly plant right foot": F_RIGHT_PLANT,
    "settle right-foot contact": F_RIGHT_PLANT,
    "transfer right 25 percent": F_TRANSFER_RIGHT_25,
    "transfer right 50 percent": F_TRANSFER_RIGHT_50,
    "transfer right 75 percent": F_TRANSFER_RIGHT_75,
    "finish shift onto right foot": F_SHIFT_RIGHT,
    "hold right support": F_SHIFT_RIGHT,
    "lift left foot vertically": F_LEFT_LIFT,
    "swing left foot forward": F_LEFT_SWING,
    "hold left foot clear": F_LEFT_SWING,
    "lower left foot to pre-contact": F_LEFT_PRECONTACT,
    "softly plant left foot": F_LEFT_PLANT,
    "settle left-foot contact": F_LEFT_PLANT,
    "transfer left 25 percent": F_TRANSFER_LEFT_25,
    "transfer left 50 percent": F_TRANSFER_LEFT_50,
    "transfer left 75 percent": F_TRANSFER_LEFT_75,
    "finish shift onto left foot": F_SHIFT_LEFT,
    "hold left support": F_SHIFT_LEFT,
    "hold final planted stance": F_SHIFT_LEFT,
}

BACKWARD = {
    "swing right foot backward": B_RIGHT_SWING,
    "hold right foot clear": B_RIGHT_SWING,
    "lower right foot to pre-contact": B_RIGHT_PRECONTACT,
    "softly plant right foot": B_RIGHT_PLANT,
    "settle right-foot contact": B_RIGHT_PLANT,
    "transfer right 25 percent": B_TRANSFER_RIGHT_25,
    "transfer right 50 percent": B_TRANSFER_RIGHT_50,
    "transfer right 75 percent": B_TRANSFER_RIGHT_75,
    "finish shift onto right foot": B_SHIFT_RIGHT,
    "hold right support": B_SHIFT_RIGHT,
    "lift left foot vertically": B_LEFT_LIFT,
    "swing left foot backward": B_LEFT_SWING,
    "hold left foot clear": B_LEFT_SWING,
    "lower left foot to pre-contact": B_LEFT_PRECONTACT,
    "softly plant left foot": B_LEFT_PLANT,
    "settle left-foot contact": B_LEFT_PLANT,
    "transfer left 25 percent": B_TRANSFER_LEFT_25,
    "transfer left 50 percent": B_TRANSFER_LEFT_50,
    "transfer left 75 percent": B_TRANSFER_LEFT_75,
    "finish shift onto left foot": B_SHIFT_LEFT,
    "hold left support": B_SHIFT_LEFT,
    "hold final planted stance": B_SHIFT_LEFT,
}


def make_long_forward_base(cycles: int, speed_scale: float = 1.0):
    phases = make_base_forward_phases(cycles, speed_scale)
    return remap_walk(phases, FORWARD, F_RIGHT_LIFT, F_RIGHT_LIFT_PERIODIC)


def make_long_backward_base(cycles: int, speed_scale: float = 1.0):
    phases = make_base_backward_phases(cycles, speed_scale)
    return remap_walk(phases, BACKWARD, F_RIGHT_LIFT, B_RIGHT_LIFT_PERIODIC)


def make_long_forward(cycles: int = 1, speed_scale: float = 1.0):
    return retime_fixed_phases(
        make_long_forward_base(cycles, 1.0), speed_scale, support_scale=2.5
    )


def make_long_backward(cycles: int = 1, speed_scale: float = 1.0):
    return retime_fixed_phases(
        make_long_backward_base(cycles, 1.0), speed_scale, support_scale=2.0
    )


def make_long_continuation(
    base_builder, cycles: int, speed_scale: float, support: float
):
    return retime_fixed_phases(
        make_continuation_template(base_builder, cycles),
        speed_scale,
        support_scale=support,
    )


def make_large_turn(left: bool, speed_scale: float = 1.0):
    phases = make_base_turn_phases(left, speed_scale=1.0)
    prefix = "TL" if left else "TR"
    values = globals()
    mapped = []
    for phase in phases:
        key = action(phase.label)
        positions = phase.positions
        if (
            "right foot" in key
            or "right-foot" in key
            or key.startswith("transfer right")
            or key in (
            "finish shift onto right foot", "hold right support"
            )
        ):
            if "clear" in key or "swing" in key or "hold right foot clear" == key:
                positions = values[f"{prefix}_RIGHT_CLEAR"]
            elif "pre-contact" in key:
                positions = values[f"{prefix}_RIGHT_PRECONTACT"]
            elif "plant" in key or "settle" in key:
                positions = values[f"{prefix}_RIGHT_PLANT"]
            elif "25 percent" in key:
                positions = values[f"{prefix}_TRANSFER_25"]
            elif "50 percent" in key:
                positions = values[f"{prefix}_TRANSFER_50"]
            elif "75 percent" in key:
                positions = values[f"{prefix}_TRANSFER_75"]
            elif key in ("finish shift onto right foot", "hold right support"):
                positions = values[f"{prefix}_SHIFT_RIGHT"]
        elif key == "lift left foot vertically":
            positions = values[f"{prefix}_LEFT_LIFT"]
        elif "left foot" in key or "left-foot" in key:
            if "clear" in key or "swing" in key or "hold left foot clear" == key:
                positions = values[f"{prefix}_LEFT_CLEAR"]
            elif "pre-contact" in key:
                positions = values[f"{prefix}_LEFT_PRECONTACT"]
            elif "plant" in key or "settle" in key:
                positions = values[f"{prefix}_LEFT_PLANT"]
        mapped.append(Phase(phase.label, phase.duration, positions))
    return retime_fixed_phases(
        tuple(mapped), min(float(speed_scale), 1.5),
        support_scale=1.5, final_hold_cap=None,
    )


class LargeStepWasdTeleop(WasdIkTeleop):
    """Keep the validated controller state machine but use larger keyframes."""

    def __init__(self) -> None:
        super().__init__()
        self.get_logger().info(
            f"Large-step mode active: {STEP_DISTANCE_MM} mm per foot placement, "
            f"{TURN_ANGLE_DEG} degrees per A/D step. Shift+W/S uses the original "
            "fast profile; Shift+A/D remains capped at 1.5x."
        )

    def start_short_step(self, sprint: bool = False) -> None:
        if self.queue_continuation_if_active("forward", sprint):
            return
        speed = self.fixed_speed_scale(sprint)
        continuing = self.continuation_ready("forward")
        phases = (
            make_long_continuation(
                make_long_forward_base, self.short_step_cycles, speed, 2.5
            )
            if continuing else make_long_forward(self.short_step_cycles, speed)
        )
        self.start_fixed_phases(
            phases,
            f"{STEP_DISTANCE_MM} mm forward gait at {speed:.2f}x",
            coast_through_waypoints=sprint,
            gait_kind="forward",
        )

    def start_backward_step(self, sprint: bool = False) -> None:
        if self.queue_continuation_if_active("backward", sprint):
            return
        speed = self.fixed_speed_scale(sprint)
        continuing = self.continuation_ready("backward")
        phases = (
            make_long_continuation(
                make_long_backward_base, self.short_step_cycles, speed, 2.0
            )
            if continuing else make_long_backward(self.short_step_cycles, speed)
        )
        self.start_fixed_phases(
            phases,
            f"{STEP_DISTANCE_MM} mm backward gait at {speed:.2f}x",
            coast_through_waypoints=sprint,
            gait_kind="backward",
        )

    def start_turn(self, left: bool, sprint: bool = False) -> None:
        turn_scale = min(self.fixed_speed_scale(sprint), 1.5)
        direction = "left" if left else "right"
        self.start_fixed_phases(
            make_large_turn(left, turn_scale),
            f"{TURN_ANGLE_DEG}-degree {direction} turn at {turn_scale:.2f}x",
            coast_through_waypoints=False,
            gait_kind=f"turn-{direction}",
        )


def self_test() -> int:
    cases = (
        ("forward", make_long_forward(1, 1.0), False),
        ("forward-sprint", make_long_forward(1, 5.0), True),
        ("forward-continuation", make_long_continuation(
            make_long_forward_base, 1, 5.0, 2.5
        ), True),
        ("backward", make_long_backward(1, 1.0), False),
        ("backward-sprint", make_long_backward(1, 5.0), True),
        ("backward-continuation", make_long_continuation(
            make_long_backward_base, 1, 5.0, 2.0
        ), True),
        ("turn-left", make_large_turn(True, 1.5), False),
        ("turn-right", make_large_turn(False, 1.5), False),
    )
    for name, phases, coast in cases:
        validate_phases(phases)
        message, duration, final = build_phase_trajectory(
            phases, coast_through_waypoints=coast
        )
        if message.joint_names != list(JOINT_NAMES):
            return 1
        if len(message.points) != len(phases) or duration <= 0.0:
            return 1
        if not np.allclose(message.points[-1].positions, final):
            return 1
        print(f"{name}: {len(phases)} points, {duration:.2f} seconds")
    return 0


def main() -> None:
    if "--self-test" in sys.argv:
        raise SystemExit(self_test())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LargeStepWasdTeleop()
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
