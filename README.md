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

Run the keyboard and IK controller in a second terminal:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run openleg_description wasd_ik_teleop

Use `W` / `S` for forward and backward, `A` / `D` to turn, `Shift` with a movement key for 2x speed, `Space` (or `X`) to stop, and `Q` to quit. A movement key remains active for 3 simulation seconds. Walking begins immediately; hold or tap repeatedly for continuous walking.

The node reads both kinematic chains and their joint limits from the current expanded URDF, solves alternating foot positions numerically, and publishes all 12 positions to `leg_controller`. Joint 3 is constrained to the turn trajectory instead of being a weak IK preference. During a turn-in-place, joint 2 is held at zero; it may still move during combined walking and turning or as part of IMU balance correction.
The experimental joint-1 pillar preference targets about 0.55 rad on the front leg and 0.05 rad on the rear leg at full stride. It is a soft IK objective: foot placement, body pitch, knee posture, and URDF limits retain priority if the exact joint-1 target is not reachable.

The default gait uses a 75 mm stride amplitude, a 30 mm walking lift, a 50 mm turn lift, and a 1.15 Hz steady cycle. Shift increases the cycle to 1.75 Hz and increases stride up to 101 mm. Forward stepping starts immediately while the gait reaches full amplitude over 0.35 seconds and the body lean builds to 7 degrees over 0.45 seconds. The controller shifts the support targets 60 mm toward `+base X` and crouches 20 mm, reducing the initial forward center-of-mass shove. During walking, only the planted leg enforces body pitch; the airborne leg remains free to follow its reachable swing path. Stopping ramps out over 2.5 seconds. This timing follows Gazebo simulation time. The controller also accepts `geometry_msgs/msg/Twist` on `/openleg/walk_cmd`. Gait, turn, lean, crouch, and IMU feedback gains are exposed as ROS parameters.

The simulated IMU is mounted 70 mm above `base_link` and bridged on `/imu`. Roll, pitch, and angular-rate feedback adjust only the leg IK targets after the body exceeds the 10-degree stability deadband. The current base mass is 4 kg.

Gazebo starts paused, waits for `wasd_ik_teleop`, `fixed_gait_demo`, `right_leg_lift_test`, or `short_step_walk_demo`, primes the joint controllers, and then releases physics. After that release there is no external fall-prevention constraint.

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
## Short-step walking sequence

`short_step_walk_demo` is a separate open-loop walking sequence built from the verified one-leg pose. It commands a right step followed by a left step, advances each foot 30 mm toward `+base X`, and uses 20 mm commanded swing clearance. During double support, the solved keyframes preserve both planted-foot poses; joint2 and joint6 transfer the weight, while small joint3 feed-forward offsets cancel kinematic yaw. It does not use the IMU or any hidden pose constraint.

Launch Gazebo on the NVIDIA GPU in the first terminal:

    cd ~/ros2_ws
    source install/setup.bash
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ros2 launch openleg_description gazebo.launch.py spawn_z:=0.58

Then run the sequence in a second terminal:

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run openleg_description short_step_walk_demo

Press Enter to start. Add `--ros-args -p auto_start:=true` for automated Gazebo validation. The default is `cycles:=10`, which commands 10 right-left cycles (20 individual steps). The `cycles` parameter accepts 1-10 and `speed_scale` accepts 0.5-1.5. The Gazebo position proportional gain is 0.2 so position errors are corrected without the velocity-driven skating seen at the previous high gain.

With the current URDF and `spawn_z:=0.58`, the full 20-step run stayed upright, advanced the pelvis about 616 mm along `+base X`, accumulated effectively zero yaw, and held its final pose without post-command sliding. The feet retained their lateral positions within about 1 mm and landed at the planned 30 mm spacing. This remains an open-loop presentation gait and should still be validated cautiously after geometry or mass changes.
