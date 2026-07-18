# OpenLeg description

<img width="1266" height="797" alt="Screenshot from 2026-07-19 01-08-56-Photoroom" src="https://github.com/user-attachments/assets/707b7a36-a7d0-44e1-9b90-4b68107f7071" />


`openleg_description` is a ROS 2 Jazzy description package for a biped with six actuated joints per leg.

The model is intentionally made from URDF primitives so it can be viewed and simulated before final CAD meshes and measured inertial properties are available. Dimensions, masses, joint limits, collision geometry, and inertias should be updated to match the physical robot before controller tuning.

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

Use `W` / `S` for forward and backward, `A` / `D` to turn, `Shift` with a movement key for 2x speed, `Space` (or `X`) to stop, and `Q` to quit. Hold a key or tap it repeatedly; a 0.9-second dead-man timeout stops stale commands.

The node reads both kinematic chains and their joint limits from the current expanded URDF, solves alternating foot positions numerically, adds a turn-only joint-3 hip-yaw preference in the IK nullspace, and publishes all 12 positions to `leg_controller`. It also accepts `geometry_msgs/msg/Twist` on `/openleg/walk_cmd` for non-keyboard control. `linear_speed`, `angular_speed`, `sprint_multiplier`, `stride_length`, `step_height`, `joint3_turn`, `turn_stride_length`, and `turn_step_height` are ROS parameters.

This is a Gazebo walking preview: the model-velocity system supplies stable planar body motion while IK animates the legs. It is useful for checking geometry, directions, limits, and commands, but it is not a dynamic-balance controller for the physical robot. Use `spawn_z:=<height>` to change the initial pelvis height and `world:=<file.sdf>` to load another Gazebo world.
