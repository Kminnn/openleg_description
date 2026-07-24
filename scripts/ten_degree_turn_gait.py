#!/usr/bin/env python3
"""Validated contact-locked 10-degree turn keyframes."""

from __future__ import annotations

import numpy as np

from directional_step_gait import make_turn_phases as make_turn_template
from short_step_walk_demo import Phase


def pose(*values: float) -> np.ndarray:
    return np.asarray(values, dtype=float)


LEFT_RIGHT_CLEAR = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.298676, -0.146842, -0.217394, -0.657376, 0.406904, 0.202396,
)
LEFT_RIGHT_PRECONTACT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.205798, -0.159492, -0.201783, -0.434954, 0.293901, 0.194785,
)
LEFT_RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.174735, -0.162590, -0.196420, -0.354317, 0.243933, 0.191734,
)
LEFT_TRANSFER_25 = pose(
    0.184682, 0.099443, -0.006651, -0.502042, 0.347475, -0.096366,
    0.223789, -0.062348, -0.166967, -0.513244, 0.333597, 0.097189,
)
LEFT_TRANSFER_50 = pose(
    0.206940, 0.009305, -0.042662, -0.554280, 0.378634, -0.000137,
    0.233219, 0.029534, -0.127537, -0.552470, 0.348923, 0.001098,
)
LEFT_TRANSFER_75 = pose(
    0.200386, -0.081947, -0.080032, -0.514854, 0.352398, 0.096267,
    0.213738, 0.116994, -0.088838, -0.499575, 0.306943, -0.094793,
)
LEFT_SHIFT_RIGHT = pose(
    0.153576, -0.177168, -0.107919, -0.356338, 0.252212, 0.191081,
    0.147856, 0.200918, -0.062902, -0.314849, 0.183877, -0.188757,
)
LEFT_LEFT_LIFT = pose(
    0.279797, -0.169972, -0.131164, -0.662144, 0.416687, 0.198897,
    0.147856, 0.200918, -0.062901, -0.314852, 0.183879, -0.188757,
)
LEFT_LEFT_CLEAR = pose(
    0.231793, -0.216624, 0.043292, -0.652447, 0.418905, 0.199837,
    0.147856, 0.200918, -0.062901, -0.314852, 0.183879, -0.188757,
)
LEFT_LEFT_PRECONTACT = pose(
    0.138656, -0.205327, 0.064088, -0.435684, 0.313434, 0.194174,
    0.147856, 0.200918, -0.062901, -0.314852, 0.183879, -0.188757,
)
LEFT_LEFT_PLANT = pose(
    0.107409, -0.200222, 0.070705, -0.355173, 0.263802, 0.191123,
    0.147856, 0.200918, -0.062901, -0.314852, 0.183879, -0.188757,
)

RIGHT_RIGHT_CLEAR = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.203292, -0.235553, 0.132507, -0.644162, 0.416899, 0.200602,
)
RIGHT_RIGHT_PRECONTACT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.112458, -0.216145, 0.154148, -0.433041, 0.318247, 0.194877,
)
RIGHT_RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.081571, -0.208272, 0.160955, -0.351956, 0.268217, 0.191824,
)
RIGHT_TRANSFER_25 = pose(
    0.185391, 0.091465, 0.036110, -0.501815, 0.350630, -0.096368,
    0.150052, -0.120279, 0.138637, -0.510359, 0.376019, 0.097243,
)
RIGHT_TRANSFER_50 = pose(
    0.200180, -0.008360, 0.042797, -0.553846, 0.384994, -0.000124,
    0.169528, -0.023030, 0.128879, -0.548633, 0.409065, 0.001025,
)
RIGHT_TRANSFER_75 = pose(
    0.177984, -0.106794, 0.050062, -0.514134, 0.361828, 0.096311,
    0.151919, 0.078676, 0.118831, -0.494155, 0.383621, -0.095077,
)
RIGHT_SHIFT_RIGHT = pose(
    0.106695, -0.200217, 0.070843, -0.354949, 0.264277, 0.191168,
    0.079310, 0.183225, 0.094474, -0.304558, 0.273726, -0.189320,
)
RIGHT_LEFT_LIFT = pose(
    0.230062, -0.215262, 0.043963, -0.647877, 0.413710, 0.199360,
    0.079312, 0.183225, 0.094475, -0.304563, 0.273727, -0.189320,
)
RIGHT_LEFT_CLEAR = pose(
    0.277447, -0.172591, -0.131344, -0.651330, 0.405805, 0.202109,
    0.079312, 0.183225, 0.094475, -0.304563, 0.273727, -0.189320,
)
RIGHT_LEFT_PRECONTACT = pose(
    0.185279, -0.176779, -0.113869, -0.436527, 0.301344, 0.194171,
    0.079312, 0.183225, 0.094475, -0.304563, 0.273727, -0.189320,
)
RIGHT_LEFT_PLANT = pose(
    0.154161, -0.177144, -0.108027, -0.356237, 0.251535, 0.191120,
    0.079312, 0.183225, 0.094475, -0.304563, 0.273727, -0.189320,
)


def _action(label: str) -> str:
    return label.split(": ", 1)[-1].lower()


def make_turn_phases(
    left: bool, speed_scale: float = 1.0
) -> tuple[Phase, ...]:
    """Return the large controller's validated 10-degree contact path."""
    phases = make_turn_template(left, speed_scale=speed_scale)
    prefix = "LEFT" if left else "RIGHT"
    values = globals()
    mapped: list[Phase] = []
    for phase in phases:
        key = _action(phase.label)
        positions = phase.positions
        if (
            "right foot" in key
            or "right-foot" in key
            or key.startswith("transfer right")
            or key in ("finish shift onto right foot", "hold right support")
        ):
            if "clear" in key or "swing" in key:
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
            if "clear" in key or "swing" in key:
                positions = values[f"{prefix}_LEFT_CLEAR"]
            elif "pre-contact" in key:
                positions = values[f"{prefix}_LEFT_PRECONTACT"]
            elif "plant" in key or "settle" in key:
                positions = values[f"{prefix}_LEFT_PLANT"]
        mapped.append(Phase(phase.label, phase.duration, positions))
    return tuple(mapped)
