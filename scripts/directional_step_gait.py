#!/usr/bin/env python3
"""Contact-locked backward and turn steps derived from the short-step gait."""

from __future__ import annotations

import numpy as np

from short_step_walk_demo import (
    LEFT_PLANT,
    Phase,
    RIGHT_LIFT_INITIAL,
    RIGHT_PLANT,
    SHIFT_LEFT_25,
    SHIFT_LEFT_50,
    SHIFT_LEFT_75,
    SHIFT_LEFT_INITIAL,
    SHIFT_RIGHT,
    STAND,
    TRANSFER_LEFT_25,
    TRANSFER_LEFT_50,
    TRANSFER_LEFT_75,
    TRANSFER_RIGHT_25,
    TRANSFER_RIGHT_50,
    TRANSFER_RIGHT_75,
)


def pose(*values: float) -> np.ndarray:
    return np.asarray(values, dtype=float)


def swap_legs(positions: np.ndarray) -> np.ndarray:
    return np.concatenate([positions[6:], positions[:6]])


# Backward placements mirror the canonical 30 mm forward contacts in base X.
# Lateral transfer poses remain contact-compatible; only the swing and planted
# sagittal solutions change.
BACK_SWING_LEG = pose(
    0.253433, -0.188770, -0.057343, -0.755749, 0.399290, 0.197387,
)
BACK_PRECONTACT_LEG = pose(
    0.086536, -0.192017, -0.010696, -0.414091, 0.360089, 0.192850,
)
BACK_PLANT_LEG = pose(
    0.051650, -0.190937, -0.003844, -0.323416, 0.303635, 0.191173,
)
BACK_LIFT_LEG = pose(
    0.372968, -0.185911, -0.066282, -0.724153, 0.279838, 0.197477,
)
BACK_RIGHT_SWING = np.concatenate([SHIFT_LEFT_INITIAL[:6], BACK_SWING_LEG])
BACK_RIGHT_PRECONTACT = np.concatenate([
    SHIFT_LEFT_INITIAL[:6], BACK_PRECONTACT_LEG,
])
BACK_RIGHT_PLANT = np.concatenate([SHIFT_LEFT_INITIAL[:6], BACK_PLANT_LEG])
BACK_TRANSFER_RIGHT_25 = swap_legs(TRANSFER_RIGHT_75)
BACK_TRANSFER_RIGHT_50 = swap_legs(TRANSFER_RIGHT_50)
BACK_TRANSFER_RIGHT_75 = swap_legs(TRANSFER_RIGHT_25)
BACK_SHIFT_RIGHT = swap_legs(RIGHT_PLANT)
BACK_LEFT_LIFT = np.concatenate([BACK_LIFT_LEG, SHIFT_RIGHT[6:]])
BACK_LEFT_SWING = np.concatenate([BACK_SWING_LEG, SHIFT_RIGHT[6:]])
BACK_LEFT_PRECONTACT = np.concatenate([BACK_PRECONTACT_LEG, SHIFT_RIGHT[6:]])
BACK_LEFT_PLANT = np.concatenate([BACK_PLANT_LEG, SHIFT_RIGHT[6:]])
BACK_TRANSFER_LEFT_25 = swap_legs(TRANSFER_LEFT_75)
BACK_TRANSFER_LEFT_50 = swap_legs(TRANSFER_LEFT_50)
BACK_TRANSFER_LEFT_75 = swap_legs(TRANSFER_LEFT_25)
BACK_SHIFT_LEFT_NEXT = swap_legs(LEFT_PLANT)
BACK_RIGHT_LIFT_PERIODIC = np.concatenate([
    SHIFT_LEFT_INITIAL[:6], BACK_LIFT_LEG,
])


# One turn command rotates both foot contacts by five degrees. The intermediate
# poses keep the old support contact fixed until the swing foot has landed.
TURN_LEFT_RIGHT_CLEAR = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.274303, -0.169643, -0.129926, -0.659218, 0.427605, 0.197822,
)
TURN_LEFT_RIGHT_PRECONTACT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.186256, -0.174820, -0.113790, -0.440738, 0.304374, 0.192316,
)
TURN_LEFT_RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.153577, -0.177168, -0.107920, -0.356340, 0.252213, 0.191081,
)
TURN_LEFT_TRANSFER_25 = pose(
    0.184682, 0.099443, -0.006651, -0.502042, 0.347475, -0.096366,
    0.200386, -0.081947, -0.080032, -0.514855, 0.352398, 0.096267,
)
TURN_LEFT_TRANSFER_50 = pose(
    0.206940, 0.009305, -0.042662, -0.554280, 0.378634, -0.000137,
    0.206940, 0.009305, -0.042662, -0.554281, 0.378634, -0.000137,
)
TURN_LEFT_TRANSFER_75 = pose(
    0.200386, -0.081947, -0.080032, -0.514854, 0.352398, 0.096267,
    0.184682, 0.099443, -0.006651, -0.502043, 0.347475, -0.096366,
)
TURN_LEFT_SHIFT_RIGHT = pose(
    0.153576, -0.177168, -0.107919, -0.356338, 0.252212, 0.191081,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
TURN_LEFT_LEFT_LIFT = pose(
    0.274303, -0.169643, -0.129926, -0.659217, 0.427605, 0.197822,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)

TURN_RIGHT_RIGHT_CLEAR = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.223031, -0.213547, 0.045859, -0.643774, 0.427605, 0.197901,
)
TURN_RIGHT_RIGHT_PRECONTACT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.139437, -0.203617, 0.064145, -0.439603, 0.316698, 0.192404,
)
TURN_RIGHT_RIGHT_PLANT = pose(
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
    0.106696, -0.200217, 0.070843, -0.354951, 0.264278, 0.191168,
)
TURN_RIGHT_TRANSFER_25 = pose(
    0.185391, 0.091465, 0.036110, -0.501815, 0.350630, -0.096368,
    0.177984, -0.106794, 0.050062, -0.514135, 0.361828, 0.096310,
)
TURN_RIGHT_TRANSFER_50 = pose(
    0.200180, -0.008360, 0.042797, -0.553846, 0.384994, -0.000124,
    0.200181, -0.008360, 0.042797, -0.553847, 0.384994, -0.000124,
)
TURN_RIGHT_TRANSFER_75 = pose(
    0.177984, -0.106794, 0.050062, -0.514134, 0.361828, 0.096311,
    0.185391, 0.091465, 0.036110, -0.501816, 0.350630, -0.096368,
)
TURN_RIGHT_SHIFT_RIGHT = pose(
    0.106695, -0.200217, 0.070843, -0.354949, 0.264277, 0.191168,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
TURN_RIGHT_LEFT_LIFT = pose(
    0.223031, -0.213547, 0.045859, -0.643773, 0.427605, 0.197901,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)

TURN_LEFT_CLEAR = pose(
    0.249849, -0.192847, -0.042582, -0.651730, 0.427605, 0.197635,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
TURN_LEFT_PRECONTACT = pose(
    0.163802, -0.190301, -0.025186, -0.440866, 0.310970, 0.192140,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
TURN_LEFT_PLANT = pose(
    0.131019, -0.189783, -0.018853, -0.356497, 0.258775, 0.190905,
    0.117146, 0.190195, 0.016211, -0.319639, 0.235537, -0.190677,
)
TURN_CENTER_25 = swap_legs(SHIFT_LEFT_75)
TURN_CENTER_50 = swap_legs(SHIFT_LEFT_50)
TURN_CENTER_75 = swap_legs(SHIFT_LEFT_25)


def make_backward_phases(
    cycles: int = 1,
    speed_scale: float = 1.0,
) -> tuple[Phase, ...]:
    cycles = max(1, min(10, int(cycles)))
    speed_scale = float(np.clip(speed_scale, 0.5, 1.5))
    phases: list[Phase] = []

    def add(label: str, duration: float, positions: np.ndarray) -> None:
        phases.append(Phase(label, duration / speed_scale, positions))

    add("prepare planted stance", 2.00, STAND)
    add("shift left 25 percent with both feet fixed", 0.50, SHIFT_LEFT_25)
    add("shift left 50 percent with both feet fixed", 0.50, SHIFT_LEFT_50)
    add("shift left 75 percent with both feet fixed", 0.50, SHIFT_LEFT_75)
    add("finish initial shift onto left foot", 0.50, SHIFT_LEFT_INITIAL)
    add("hold initial left support", 0.60, SHIFT_LEFT_INITIAL)
    add("lift right foot vertically", 1.50, RIGHT_LIFT_INITIAL)
    for cycle in range(1, cycles + 1):
        add(f"cycle {cycle}: swing right foot backward", 1.00, BACK_RIGHT_SWING)
        add(f"cycle {cycle}: hold right foot clear", 0.60, BACK_RIGHT_SWING)
        add(f"cycle {cycle}: lower right foot to pre-contact", 0.65, BACK_RIGHT_PRECONTACT)
        add(f"cycle {cycle}: softly plant right foot", 0.75, BACK_RIGHT_PLANT)
        add(f"cycle {cycle}: settle right-foot contact", 0.30, BACK_RIGHT_PLANT)
        add(f"cycle {cycle}: transfer right 25 percent", 1.00, BACK_TRANSFER_RIGHT_25)
        add(f"cycle {cycle}: transfer right 50 percent", 1.00, BACK_TRANSFER_RIGHT_50)
        add(f"cycle {cycle}: transfer right 75 percent", 1.00, BACK_TRANSFER_RIGHT_75)
        add(f"cycle {cycle}: finish shift onto right foot", 1.00, BACK_SHIFT_RIGHT)
        add(f"cycle {cycle}: hold right support", 1.50, BACK_SHIFT_RIGHT)
        add(f"cycle {cycle}: lift left foot vertically", 1.50, BACK_LEFT_LIFT)
        add(f"cycle {cycle}: swing left foot backward", 1.00, BACK_LEFT_SWING)
        add(f"cycle {cycle}: hold left foot clear", 0.60, BACK_LEFT_SWING)
        add(f"cycle {cycle}: lower left foot to pre-contact", 0.65, BACK_LEFT_PRECONTACT)
        add(f"cycle {cycle}: softly plant left foot", 0.75, BACK_LEFT_PLANT)
        add(f"cycle {cycle}: settle left-foot contact", 0.30, BACK_LEFT_PLANT)
        add(f"cycle {cycle}: transfer left 25 percent", 1.00, BACK_TRANSFER_LEFT_25)
        add(f"cycle {cycle}: transfer left 50 percent", 1.00, BACK_TRANSFER_LEFT_50)
        add(f"cycle {cycle}: transfer left 75 percent", 1.00, BACK_TRANSFER_LEFT_75)
        add(f"cycle {cycle}: finish shift onto left foot", 1.00, BACK_SHIFT_LEFT_NEXT)
        add(f"cycle {cycle}: hold left support", 1.50, BACK_SHIFT_LEFT_NEXT)
        if cycle < cycles:
            add(
                f"cycle {cycle + 1}: lift right foot vertically",
                1.50,
                BACK_RIGHT_LIFT_PERIODIC,
            )
    add("hold final planted stance", 1.50, BACK_SHIFT_LEFT_NEXT)
    return tuple(phases)


def make_turn_phases(
    left: bool,
    speed_scale: float = 1.0,
) -> tuple[Phase, ...]:
    speed_scale = float(np.clip(speed_scale, 0.5, 1.5))
    direction = "left" if left else "right"
    if left:
        right_clear = TURN_LEFT_RIGHT_CLEAR
        right_precontact = TURN_LEFT_RIGHT_PRECONTACT
        right_plant = TURN_LEFT_RIGHT_PLANT
        transfers = (
            TURN_LEFT_TRANSFER_25, TURN_LEFT_TRANSFER_50,
            TURN_LEFT_TRANSFER_75, TURN_LEFT_SHIFT_RIGHT,
        )
        left_lift = TURN_LEFT_LEFT_LIFT
    else:
        right_clear = TURN_RIGHT_RIGHT_CLEAR
        right_precontact = TURN_RIGHT_RIGHT_PRECONTACT
        right_plant = TURN_RIGHT_RIGHT_PLANT
        transfers = (
            TURN_RIGHT_TRANSFER_25, TURN_RIGHT_TRANSFER_50,
            TURN_RIGHT_TRANSFER_75, TURN_RIGHT_SHIFT_RIGHT,
        )
        left_lift = TURN_RIGHT_LEFT_LIFT
    phases: list[Phase] = []

    def add(label: str, duration: float, positions: np.ndarray) -> None:
        phases.append(Phase(label, duration / speed_scale, positions))

    add("prepare planted stance", 2.00, STAND)
    add("shift left 25 percent with both feet fixed", 0.50, SHIFT_LEFT_25)
    add("shift left 50 percent with both feet fixed", 0.50, SHIFT_LEFT_50)
    add("shift left 75 percent with both feet fixed", 0.50, SHIFT_LEFT_75)
    add("finish initial shift onto left foot", 0.50, SHIFT_LEFT_INITIAL)
    add("hold initial left support", 0.60, SHIFT_LEFT_INITIAL)
    add("lift right foot vertically", 1.50, RIGHT_LIFT_INITIAL)
    add(f"swing right foot into {direction} turn", 1.00, right_clear)
    add("hold right foot clear", 0.60, right_clear)
    add("lower right foot to pre-contact", 0.65, right_precontact)
    add("softly plant turned right foot", 0.75, right_plant)
    add("settle right-foot contact", 0.30, right_plant)
    add("transfer right 25 percent", 1.00, transfers[0])
    add("transfer right 50 percent", 1.00, transfers[1])
    add("transfer right 75 percent", 1.00, transfers[2])
    add("finish shift onto right foot", 1.00, transfers[3])
    add("hold right support", 1.50, transfers[3])
    add("lift left foot vertically", 1.50, left_lift)
    add(f"swing left foot into {direction} turn", 1.00, TURN_LEFT_CLEAR)
    add("hold left foot clear", 0.60, TURN_LEFT_CLEAR)
    add("lower left foot to pre-contact", 0.65, TURN_LEFT_PRECONTACT)
    add("softly plant turned left foot", 0.75, TURN_LEFT_PLANT)
    add("settle left-foot contact", 0.30, TURN_LEFT_PLANT)
    add("return to center 25 percent", 0.50, TURN_CENTER_25)
    add("return to center 50 percent", 0.50, TURN_CENTER_50)
    add("return to center 75 percent", 0.50, TURN_CENTER_75)
    add("finish centered stance", 0.50, STAND)
    add("hold final planted stance", 1.50, STAND)
    return tuple(phases)

