# OpenLeg description

`openleg_description` is a ROS 2 Jazzy description package for a biped with six actuated joints per leg.

For the complete problem-and-solution history from CAD pivots through the validated 20-step gait, see [TROUBLESHOOTING_HISTORY.md](TROUBLESHOOTING_HISTORY.md).

The model uses the CAD STL meshes for both visual and collision geometry. Dimensions, masses, joint limits, mesh transforms, and inertias should be kept aligned with the physical robot before controller tuning.

## Description layout

- `urdf/main.urdf.xacro`: pelvis, robot assembly, Gazebo plugin, and `ros2_control`
- `urdf/left_leg.urdf.xacro`: complete six-joint left leg
- `urdf/right_leg.urdf.xacro`: complete six-joint right leg

The intended joint order is hip pitch, hip roll, hip yaw, knee pitch, toes pitch, and toes roll.

## Build and preview

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select openleg_description
source install/setup.bash
ros2 launch openleg_description display.launch.py
```

## Gazebo Sim

```bash
ros2 launch openleg_description gazebo.launch.py
```

The Gazebo launch starts `joint_state_broadcaster` and a position-based `leg_controller`. Send a complete 12-joint trajectory to `/leg_controller/joint_trajectory` to command the legs.

## WASD walking preview

Start Gazebo in the first terminal:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch openleg_description gazebo.launch.py

Run the keyboard controller in a second terminal:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run openleg_description wasd_ik_teleop

`W` and `S` run fixed sequential forward and backward gaits. One press commands one complete right-left cycle, or two individual steps. `A` and `D` run fixed 10-degree left and right turns. Shift walking uses contact-aware overlap: Shift+W uses 2.5x support/touchdown timing with 5x unloaded swing timing, Shift+S uses 2x support/touchdown with 5x swing, and Shift+A/D retains the proven 1.5x turn. Lowercase commands retain 1.0x timing. `short_step_sprint_scale` controls unloaded Shift swing timing from 0.5-5.0 and defaults to 5.0. Set `short_step_cycles:=10` if one forward or backward keypress should run a complete 20-step sequence. While a W/S trajectory is active, pressing the same direction again queues exactly one continuation. It starts directly from the existing left-support endpoint and skips the center-reset and repeated initial weight shift; keyboard repeat can keep one pair queued for continuous walking. Opposite-direction and turn keys remain ignored until the active pair ends. Walking uses only a 0.15-second terminal hold between queued trajectories; `Space` clears both the active trajectory and any queued continuation.

`Space` (or `X`) cancels the active trajectory and holds the measured joint positions; `Q` holds and quits. Keyboard WASD does not use IMU correction. ROS `Twist` input on `/openleg/walk_cmd` remains the experimental continuous IK path.

Forward `W` uses the exact contact-locked, 30 mm placement, 20 mm clearance, weight-transfer, and soft-touchdown phases from `short_step_walk_demo`. Backward `S` mirrors the sagittal contacts while preserving the same lateral support transfer. Each turn places the right foot at the new yaw, transfers support, places the left foot at the same yaw, and then returns to a centered stance. Joint 3 supplies the turn while Joint 2 and Joint 6 perform the required weight transfer.

Server-only Gazebo validation of the selected Shift timings completed upright. A single Shift+W pair takes 7.97 seconds and a no-reset continuation takes 6.25 seconds; the queued four-step trial advanced 146.6 mm and ended at about 0.01 degrees roll, 1.80 degrees pitch, and -2.96 degrees yaw. A single Shift+S pair takes 9.45 seconds and its continuation takes 7.33 seconds; the queued four-step trial moved 106.0 mm backward and ended at about -0.19 degrees roll, 1.80 degrees pitch, and 2.83 degrees yaw. Both trials retain the approximately 107 mm lateral left-support offset instead of recentering between pairs. Shift+A/D retain their previously validated 1.5x timing. A 2.75x forward support trial remains rejected because it accumulated about -7.3 degrees yaw.

For larger displacement and rotation per keypress, use the separate controller below instead of `wasd_ik_teleop`:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run openleg_description large_step_wasd_teleop

This version moves each foot 35 mm per W/S step and turns 10 degrees per A/D step. Shift+W uses the original controller's 2.5x support/touchdown and 5x unloaded-swing timing; Shift+S uses 2x support/touchdown and 5x unloaded swing. Shift+A/D remains capped at the proven 1.5x turn timing. Do not run both keyboard controllers at the same time. The earlier 1.5x clean Gazebo trials finished upright: W advanced 75.7 mm, S moved 42.1 mm backward, A turned 10.03 degrees, and D turned 9.99 degrees. A clean Shift+W trial at the full original timing finished upright in 7.97 seconds and advanced 93.9 mm, but accumulated 4.8 degrees of right yaw from the faster contact slip.

The experimental joint-1 pillar preference targets about 0.55 rad on the front leg and 0.05 rad on the rear leg at full stride. It is a soft IK objective: foot placement, body pitch, knee posture, and URDF limits retain priority if the exact joint-1 target is not reachable.

The remaining `Twist` IK preview uses a 75 mm stride amplitude, a 30 mm backward lift, a 50 mm turn lift, and a 1.15 Hz steady cycle. Its timing follows Gazebo simulation time; IK turn, lean, crouch, and IMU feedback gains remain exposed as ROS parameters.

The simulated IMU is mounted 70 mm above `base_link` and bridged on `/imu`. Roll, pitch, and angular-rate feedback adjust only the leg IK targets after the body exceeds the 10-degree stability deadband. The current base mass is 4 kg.

Gazebo starts paused, waits for `wasd_ik_teleop`, `fixed_gait_demo`, `right_leg_lift_test`, `short_step_walk_demo`, `overlap_walk_demo`, or `backward_bound_demo`, primes the joint controllers, and then releases physics. After that release there is no external fall-prevention constraint.

There is no Gazebo model-velocity, pose, height, or upright override. Motion and recovery come from the commanded joints, contacts, gravity, and the robot's own IMU feedback, so the robot is allowed to fall. Use `spawn_z:=<height>` to change only its initial pelvis height and `world:=<file.sdf>` to load another Gazebo world.
Visual geometry uses the CAD STL meshes. Non-foot collisions use their STL meshes; each foot uses a moderate 200 x 140 x 30 mm collision box, extended slightly beyond the original 140 x 80 x 30 mm CAD footprint. Its ground-contact face remains aligned while the 30 mm height clears joint6. Existing contact damping, zero floor restitution, and floor friction remain configured.
The two Link1 mounting origins temporarily share zero center spacing for the single-support calibration. The leg chains remain separated by their internal CAD offsets.

## Fixed joint-path presentation

`fixed_gait_demo` is a separate open-loop controller for a repeatable presentation. It sends 12 pre-recorded motor angles through `leg_controller`: pre-lean forward, move weight left, lift/swing/plant the right leg, move weight right, then lift/swing/plant the left leg. It does not use runtime IK or IMU corrections.

Start Gazebo normally, but do not start `wasd_ik_teleop`. In a second terminal run:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run openleg_description fixed_gait_demo --ros-args -p cycles:=2

The node prints every scheduled phase and waits for Enter before publishing. For an automatic presentation use `-p auto_start:=true`. Use `-p speed_scale:=1.2` for 20% faster motion or `-p motion_scale:=0.7` for a smaller, safer joint range.

This path is intentionally open-loop. Validate it in Gazebo first, never run the WASD and fixed-path nodes together, and do not use it on unsupported hardware without an emergency stop, mechanical support, current limits, and a person ready to cut motor power.
## Right-leg-lift calibration

`right_leg_lift_test` is a dedicated open-loop unloading test. It prepares a planted stance, shifts toward the left foot over 2 seconds using 10 degrees of left joint2, holds for 2 seconds, then commands a corrected 10 mm right-foot lift over 2 seconds. It has no forward swing and uses no IMU correction. The controller holds the final pose.

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run openleg_description right_leg_lift_test

For automated Gazebo trials, append `--ros-args -p auto_start:=true`. The only motion-tuning parameter is `lift_height_mm`; it is constrained to 10-20 mm and defaults to 10 mm.

Current Gazebo validation of the default corrected 10 mm target kept the robot upright for more than 47 simulated seconds after the endpoint. The left foot remained on the floor while the lowest right-foot collision corner held 9.69 mm above it. Joint velocities settled effectively to zero, the base roll remained about 3.3 degrees, and the largest final effort was about 40.4 Nm. This verifies open-loop single support for the calibration pose; the optional 15-20 mm targets have not been dynamically validated.

## Walking presentation sequences

`short_step_walk_demo` and `overlap_walk_demo` now use the same validated contact-locked forward phases. `overlap_walk_demo` defaults to the fastest validated 1.5x timing. Never run both nodes at the same time.

### Sequential walking sequence

`short_step_walk_demo` completes every lateral transfer before lifting the next foot. Its default remains 20 individual walking steps.

### Fast forward sequence

`overlap_walk_demo` prioritizes reliable forward travel over the unsuccessful bound attempt. It uses the contact-locked 30 mm placements, 20 mm clearance, complete support transfers, and soft touchdowns from `short_step_walk_demo`, with a default speed of 1.5x. `cycles` accepts 1-10; one cycle is one right-left pair.

A clean one-cycle Gazebo check completed upright at +45.5 mm X. The final base orientation was approximately 0.01 degrees roll, 1.8 degrees pitch, and -0.7 degrees yaw. The earlier bilateral bound is preserved as `backward_bound_demo` because it achieves flight but moves backward; true forward running remains unresolved and is not claimed.

Launch Gazebo on the NVIDIA GPU in the first terminal:

    cd ~/ros2_ws
    source install/setup.bash
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ros2 launch openleg_description gazebo.launch.py spawn_z:=0.58

In a second terminal, choose exactly one sequence:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run openleg_description short_step_walk_demo

or:

    ros2 run openleg_description overlap_walk_demo

Press Enter to start. Add `--ros-args -p auto_start:=true` for automated trials. For ten forward right-left cycles use:

    ros2 run openleg_description overlap_walk_demo --ros-args -p cycles:=10

The earlier symmetric hopping experiment is preserved separately as
`backward_bound_demo`. It produces bilateral flight but travels toward -X
(backward), and it may fall after the trajectory ends. It is intentionally
kept experimental and has no IMU or hidden fall prevention:

    ros2 run openleg_description backward_bound_demo
