# OpenLeg description

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

Use `W` / `S` for forward and backward, `A` / `D` to turn, `Shift` with a movement key for 2x speed, `Space` (or `X`) to stop, and `Q` to quit. A movement key remains active for 3 simulation seconds. Walking begins immediately; hold or tap repeatedly for continuous walking.

The node reads both kinematic chains and their joint limits from the current expanded URDF, solves alternating foot positions numerically, and publishes all 12 positions to `leg_controller`. Joint 3 is constrained to the turn trajectory instead of being a weak IK preference. During a turn-in-place, joint 2 is held at zero; it may still move during combined walking and turning or as part of IMU balance correction.
The experimental joint-1 pillar preference targets about 0.55 rad on the front leg and 0.05 rad on the rear leg at full stride. It is a soft IK objective: foot placement, body pitch, knee posture, and URDF limits retain priority if the exact joint-1 target is not reachable.

The default gait uses a 75 mm stride amplitude, a 30 mm walking lift, a 50 mm turn lift, and a 1.15 Hz steady cycle. Shift increases the cycle to 1.75 Hz and increases stride up to 101 mm. Forward stepping starts immediately while the gait reaches full amplitude over 0.35 seconds and the body lean builds to 7 degrees over 0.45 seconds. The controller shifts the support targets 60 mm toward `+base X` and crouches 20 mm, reducing the initial forward center-of-mass shove. During walking, only the planted leg enforces body pitch; the airborne leg remains free to follow its reachable swing path. Stopping ramps out over 2.5 seconds. This timing follows Gazebo simulation time. The controller also accepts `geometry_msgs/msg/Twist` on `/openleg/walk_cmd`. Gait, turn, lean, crouch, and IMU feedback gains are exposed as ROS parameters.

The simulated IMU is mounted 70 mm above `base_link` and bridged on `/imu`. Roll, pitch, and angular-rate feedback adjust only the leg IK targets after the body exceeds the 10-degree stability deadband. The current base mass is 12 kg.

Gazebo starts paused, waits for `wasd_ik_teleop`, primes the joint controllers, and then releases physics. After that release there is no external fall-prevention constraint.

There is no Gazebo model-velocity, pose, height, or upright override. Motion and recovery come from the commanded joints, contacts, gravity, and the robot's own IMU feedback, so the robot is allowed to fall. Use `spawn_z:=<height>` to change only its initial pelvis height and `world:=<file.sdf>` to load another Gazebo world.
