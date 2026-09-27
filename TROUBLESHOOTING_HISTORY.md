# OpenLeg URDF and Gazebo Troubleshooting History

This document records the problems encountered while building and tuning the OpenLeg simulation, what caused them, what was changed, and which solution is currently active.

The values below describe the repository state validated on 2026-07-22. Entries marked **superseded** are useful history but are not part of the final presentation gait.

## Current known-good presentation configuration

| Setting | Current value |
|---|---|
| Base mass | 4.0 kg |
| Link masses, per leg | L1 2.0, L2 1.9, L3 0.3, L4 2.75, L5 3.1, L6 1.0, L7 0.5 kg |
| Base-to-Link1 height | 70 mm in +Z |
| Hip mounting spacing | 0 mm during single-support calibration |
| Foot collision | 200 x 140 x 30 mm box, aligned to the foot sole |
| Surface friction | `mu1 = mu2 = 2.5` |
| Restitution | 0.0 |
| Gazebo position proportional gain | 0.2 |
| Spawn height | `spawn_z:=0.58` |
| Narrow high-knee gait | 20 steps in 150.00 s, 150 mm stance, 20 mm clearance, 30 mm placement; one right-left cycle dynamically stable |
| IMU | 70 mm above `base_link`; not used by the fixed presentation gait |

Motor limits currently use the supplied peak ratings: J1, J2, J4, and J5 use 140 Nm and 100 rpm; J3 and J6 use 97 Nm and 40 rpm. The earlier 40 Nm and 30 Nm values are treated as non-peak ratings.

## A. URDF, meshes, and joint geometry

### 1. Parts rotated around the wrong point

- **Symptom:** The STL origin looked correct, but a moving link orbited around an incorrect joint location.
- **Cause:** A mesh origin and a URDF joint origin have different jobs. The child link always rotates around the joint frame, regardless of where its visual mesh appears.
- **Solution:** Re-export the moving STL files with the CAD pivot at the STL origin, keep their mesh transforms simple, and place the child link with the URDF joint origin. Joint1 remains the deliberate exception because its mounting relationship is different.
- **Status:** Solved.

### 2. Positive joint direction was unclear

- **Symptom:** It was unclear whether a positive-radian command should appear clockwise or counter-clockwise.
- **Cause:** CW/CCW depends on the viewing side of the axis.
- **Solution:** Use the URDF right-hand rule: point the right thumb along the joint `<axis>` vector; curled fingers show positive rotation. Verify each mirrored axis in RViz or Gazebo rather than assigning CW/CCW without a viewpoint.
- **Status:** Documented and encoded through mirrored left/right axes.

### 3. Robot colors were inconsistent

- **Symptom:** Different links used different colors.
- **Solution:** All named URDF materials now use gray, RGBA `0.55 0.55 0.55 1`.
- **Status:** Solved.

### 4. Link masses and motor ratings were placeholders

- **Symptom:** Simulation dynamics did not represent the supplied hardware.
- **Solution:** Entered the supplied L1-L7 masses for both legs. Motor effort limits use the later peak-torque values, while velocity limits use 100 rpm for J1/J2/J4/J5 and 40 rpm for J3/J6.
- **Status:** Solved for the supplied data. CAD-derived centers of mass and inertia tensors would still improve realism.

### 5. Joint limits were incorrect or caused trajectory exceptions

- **Symptoms:** J4 and J5 had wrong travel; J2 needed more roll range; J1 needed another 150 degrees from -0.915 rad; one fixed-gait keyframe threw `Joint limit exceeded in keyframe: plant right foot`.
- **Solution:** Updated the URDF limits and added trajectory validation. Current limits are:
  - J1: -0.915 rad to `-0.915 + 150 deg`.
  - Right J2: -90 to +30 deg. Left J2 uses its mirrored mechanical definition, -30 to +20 deg.
  - J3: +/-90 deg.
  - J4: -92.5 to +2.5 deg in the current axis convention.
  - J5: +/-24.5 deg.
  - J6: +/-30 deg. A +/-45 deg experiment was considered, but the current validated model uses +/-30 deg.
- **Status:** Solved for the current model.

### 6. Left leg was missing and initially faced the wrong way

- **Symptom:** Only the right chain existed; early mirroring made the legs face each other.
- **Solution:** Added a separate mirrored left-leg Xacro, corrected its mesh rotations and joint axes, and reused the symmetric Link3 and Link7 STL files on both sides.
- **Status:** Solved.

### 7. Base placement used incorrect units and orientation

- **Symptoms:** The base appeared far above the legs because 70 mm was initially described as 70 cm; it also needed a 90-degree yaw.
- **Solution:** Set the base-to-Link1 Z offset to 0.070 m and rotate the base mesh by 90 degrees yaw.
- **Status:** Solved.

### 8. Leg spacing made single-support transfer difficult

- **Symptom:** A large 600 mm Link1-to-Link1 spacing required an extreme lateral COM shift.
- **Solution:** For the current calibration, both Link1 mounting origins use zero explicit hip spacing. The CAD chain offsets still separate the legs physically.
- **Status:** Solved for simulation calibration; the real pelvis mounting spacing must eventually replace this temporary value.

## B. Gazebo startup, control, and performance

### 9. Robot spawned lying down or with feet below the floor

- **Cause:** The model origin is not located at the bottom of the feet, so a low world Z intersects the ground during spawn.
- **Solution:** Use `spawn_z:=0.58` with the current geometry.
- **Status:** Solved for the current URDF dimensions.

### 10. WASD produced no movement

- **Causes encountered:** The teleop process and joint controller were not always active together, the installed package could be stale after source edits, or the terminal running the keyboard node did not have focus.
- **Solution:** Build and source the workspace, wait for `leg_controller` to become active, then run the teleop node in the focused terminal. The launch process also waits for one of the supported motion nodes before releasing physics.
- **Status:** The controller path works. The later professor-demo work moved to a deterministic fixed gait instead of relying on keyboard timing.

### 11. Turning moved Joint2 instead of Joint3

- **Cause:** The numerical IK could satisfy part of a turn through hip roll because Joint3 was only a weak preference.
- **Solution:** Constrain Joint3 directly to the turn trajectory and keep Joint2 at zero for a pure turn. Joint2 may still move during combined walking/turning or IMU correction.
- **Status:** Solved in `wasd_ik_teleop.py`.

### 12. Walking was too short, slow, and low

- **Solution:** Added longer stride, higher lift, faster steady motion, and a Shift speed multiplier to the WASD controller. The final presentation path instead uses deliberately conservative 30 mm placements and 20 mm clearance for repeatability.
- **Status:** Solved through two modes: faster experimental WASD gait and slower validated presentation gait.

### 13. Gazebo ran at about 2 percent real time

- **Solution:** Launch Gazebo with NVIDIA PRIME offload:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ros2 launch openleg_description gazebo.launch.py spawn_z:=0.58
```

- **Status:** Solved when an NVIDIA GPU and its driver are available.

### 14. IMU correction was too sensitive

- **Symptom:** Small IMU angles caused unwanted correction and jitter.
- **Solution:** Added a deadband so no correction occurs inside the configured threshold. The current WASD controller uses 10 degrees. A 30-degree trial was made and later superseded.
- **Status:** Solved for WASD. The fixed lift and short-step presentation nodes intentionally use no IMU feedback.

## C. Contact, collision, bounce, and sliding

### 15. Robot bounced as if the floor were a trampoline

- **Causes:** Collision/contact geometry and high-energy controller corrections can continuously inject energy. Restitution also must be zero for a non-bouncy floor.
- **Solution:** Kept zero floor restitution, added contact damping/limits, and restored CAD STL collision geometry for non-foot links. Foot collision remains a simple box because it provides a predictable flat sole.
- **Status:** Solved for the validated standing and walking tests.

### 16. Collision geometry did not match the requested CAD behavior

- **History:** Primitive collision substitutions were tried while diagnosing bounce, then reverted because STL collisions were required. A box was retained only for each foot after single-support testing showed that a flat, controllable support polygon was valuable.
- **Current solution:** Non-foot links use their STL meshes for visual and collision geometry. Feet use the aligned 200 x 140 x 30 mm box.
- **Status:** Solved with this documented exception.

### 17. Foot collision appeared 50-100 mm above the visual foot

- **Cause:** The foot box origin was moved without respecting the mesh frame and Joint6 location.
- **Solution:** Re-aligned the box ground-contact face with the visual sole and kept its height at 30 mm so it does not block Joint6.
- **Status:** Solved.

### 18. The widened 300 mm foot was unrealistic

- **History:** A very wide collision was tested to enlarge the support polygon, but it was visibly unrealistic and could interfere with Joint6.
- **Solution:** Reduced the final footprint to a moderate 200 x 140 mm, compared with the original CAD footprint of roughly 140 x 80 mm.
- **Status:** Solved; the CAD foot can later be updated toward this moderate footprint.

### 19. Increasing floor friction did not stop ice-skating motion

- **Symptom:** The robot slid backward or walked in place even with high floor friction.
- **Finding:** Floor and feet already used `mu1 = mu2 = 2.5`, similar to a high-grip mat. More friction could not solve a trajectory that geometrically commanded planted feet to move relative to one another.
- **Solution:** Keep friction at 2.5 and correct the trajectory/controller causes described in Problems 26 and 27.
- **Status:** Solved.

## D. Balance and one-leg support

### 20. Robot could not stand on one leg

- **Symptoms:** Lifting one leg also unloaded or raised the support foot, the body shifted toward the swing side, and the robot fell backward-right.
- **Main causes:** The projected center of mass was outside the support-foot polygon; lateral travel was limited; some poses demanded too much simultaneous correction; and the earlier foot collision interfered with ankle roll.
- **Solution:** Reduce temporary hip spacing, use Joint2 for pelvis translation, use Joint6 to keep the support foot flat, use realistic peak effort limits, and move through a slow, kinematically solved weight-transfer path.
- **Status:** Solved by the dedicated right-leg-lift test.

### 21. Joint2 reached its limit during weight shift

- **Important observation:** Joint2 direction was correct and therefore was not reversed.
- **Solution:** Added the requested extra usable travel and solved poses inside the resulting URDF limits. The left and right numeric limits differ because their mechanical axes are mirrored.
- **Status:** Solved.

### 22. Joint6 ankle roll did not appear useful or collided with the foot

- **Cause:** The tall or misaligned foot collision prevented the ankle from reaching the required roll.
- **Solution:** Correct the foot collision height/origin and include Joint6 in the support-foot leveling solution. The current validated travel is +/-30 degrees and the peak effort limit is 97 Nm.
- **Status:** Solved.

### 23. Both feet moved when only one should lift

- **Cause:** Moving the pelvis without preserving the support chain changes both world foot poses. A stationary set of joint values for the swing leg is not enough; the support leg must actively hold the pelvis and its foot pose.
- **Solution:** Solve the complete 12-joint pose while fixing the planted support-foot world transform, then lift only the swing foot.
- **Status:** Solved.

### 24. Needed a controlled test before attempting a full walk

- **Solution:** Added `right_leg_lift_test`: both feet planted, 2-second left shift, 2-second hold, and a 10 mm right-foot lift over 2 seconds with no forward swing and no IMU.
- **Validation:** The robot stayed upright for more than 47 simulated seconds after the endpoint; measured right-foot clearance was 9.69 mm, base roll was about 3.3 degrees, and peak final effort was about 40.4 Nm.
- **Status:** Historical result only; superseded by the running attempt and the narrow-stance fallback below.

## E. Fixed gait and final walking solution

### 25. Pre-lean used the wrong sign or too much angle

- **Symptoms:** The robot leaned backward before or after Enter, or leaned too far forward and fell.
- **Cause:** The commanded sign was interpreted in the joint-chain frame rather than from the observed base pitch, and a large static lean created a destabilizing initial transient.
- **Solution:** Correct the sign, limit the fixed-gait lean parameter to +/-10 degrees, and replace large lean experiments with gradual support-foot-constrained weight transfer.
- **Status:** Solved and superseded by the contact-locked short-step gait.

### 26. Old double-support keyframes forced the feet to slide

- **Symptom:** During walking, the robot looked like it was ice skating, moved slightly backward, or rotated even though both feet should have been planted.
- **Root cause:** Consecutive double-support keyframes changed the feet relative translation by about 17 mm and changed relative yaw by several degrees. Gazebo could satisfy those incompatible constraints only by slipping or deforming contact.
- **Solution:** Re-solve every transfer pose against fixed world contact poses. The new initial shift uses 25/50/75/100 percent substeps, and double-support foot-to-foot translation error remains within about 0.6 mm, including interpolated points.
- **Status:** Solved.

### 27. High position gain injected sliding and oscillation

- **Symptom:** Even after correcting geometry, the whole robot could skate backward while the feet tracked the requested relative step.
- **Cause:** `gz_ros2_control` uses the position proportional gain to convert joint position error into commanded velocity. The previous high gain made small errors produce aggressive contact forces.
- **Solution:** Reduce `position_proportional_gain` to 0.2.
- **Status:** Solved.

### 28. Feet did not lift clear and the toe rubbed the floor

- **Cause:** Earlier swing poses did not preserve the support transform while creating enough vertical clearance, and whole-body shift consumed some apparent lift.
- **Solution:** Use a vertical lift phase before forward swing and solve for 20 mm world-frame swing-foot clearance.
- **Status:** Solved in the short-step presentation gait.

### 29. The first right step moved, but the left leg stayed behind

- **Cause:** A complete walking cycle requires landing the right foot, transferring support to it, then lifting and advancing the left foot. Earlier sequences did not complete this transfer robustly.
- **Solution:** Added full alternating right/left transfer, lift, swing, plant, and hold phases.
- **Status:** Solved.

### 30. Joint3 correction was needed without commanding a turn

- **Symptom:** Keeping Joint3 at zero produced accumulated yaw over many steps.
- **Solution:** Add small Joint3 feed-forward offsets calculated to cancel kinematic yaw. These offsets keep the robot straight and are not turn commands.
- **Status:** Solved.

### 31. **Superseded:** repeatable 20-step overlap-walk demonstration

- **Solution:** Added separate `short_step_walk_demo.py` and `overlap_walk_demo.py` executables. The first completes every support shift before lifting; the second begins lift at 75% transfer. Both default to 10 right-left cycles, use soft touchdown, and have no IMU or external pose constraint.
- **Final dynamic validation of `overlap_walk_demo` at 1.5x timing:** The updated 128-second sequence stayed upright and advanced the pelvis about 621.5 mm. Its final pose was unchanged after a further 15-second hold, showing no residual bounce or slide after commands stopped.
- **Sequential smoke validation:** `short_step_walk_demo` completed one full right-left cycle upright and held an unchanged final pose after the trajectory.
- **Status:** Historical result only; superseded by the running attempt and the narrow-stance fallback below.

### 32. **Superseded:** overlap-walk touchdown jiggle reduction

- **Symptom:** The robot made a small visible body movement just as the lowered foot reached the floor.
- **Cause:** The first soft-landing profile still covered its final 3.75 mm in 0.5 seconds, about 7.5 mm/s, and settled for only 0.2 seconds. That left enough downward momentum for a small contact bounce.
- **Solution:** In `overlap_walk_demo` only, added a measured 4.60 mm pre-contact waypoint and a 0.68 mm near-contact waypoint. The average final descent rate is 0.975 mm/s, both endpoints command zero velocity, and the planted hold is 0.4 seconds before weight transfer. The sequential 1.5x demo was not changed.
- **Validation:** The complete 20-step, 128-second Gazebo run stayed upright, advanced the pelvis 621.5 mm, and its final XYZ and RPY were unchanged after a 15-second hold.
- **Status:** Historical result only; superseded by the current narrow high-knee sequence.

### 32b. Fixed-flight attempt and narrow-stance fall correction

- **Request:** Make `overlap_walk_demo.py` run instead of performing the earlier concurrent walk.
- **Failure cause:** The original nominal pose places the foot centers 280.7 mm apart even though hip-mount spacing is zero. A fixed flight path therefore required excessive lateral COM travel. Narrow-flight tests corrected the lateral fall but still fell backward without feedback.
- **Active correction:** The demo now settles to a 150 mm stance through Joint2, keeps the support leg fixed during every lift, uses 20 mm knee clearance and 30 mm alternating placements, and removes the unsuccessful flight commands.
- **Safety checks:** All 12 joint limits and conservative spline peak speeds pass at `speed_scale:=1.1`; the largest predicted motor-speed ratio is below 0.08.
- **Dynamic result:** One installed right-left cycle at `spawn_z:=0.58` remained upright and held unchanged for another 10 seconds at XYZ `[-0.0664, -0.0317, 0.5770]` m and RPY `[1.86, -10.64, -1.76]` degrees.
- **Status:** The fall is corrected for the narrow high-knee cycle. True running flight remains unresolved and is not claimed.

## F. ROS launch and repository workflow

### 33. Xacro output failed as a ROS parameter at launch

- **Symptom:** The URDF could pass Xacro checks but fail only when `robot_state_publisher` started because the expanded XML was interpreted as YAML.
- **Cause:** ROS 2 Jazzy launch parameter type inference did not always preserve the expanded Xacro output as a plain string.
- **Solution:** Wrap `robot_description` with `launch_ros.parameter_descriptions.ParameterValue(..., value_type=str)` in both display and Gazebo launch files.
- **Status:** Solved and covered by package tests.

### 34. Source edits did not appear in the running package

- **Cause:** ROS commands use the installed workspace copy, which can be stale if the package was not rebuilt or the terminal was not sourced again.
- **Solution:** Build with `colcon build --symlink-install --packages-select openleg_description`, then source `~/ros2_ws/install/setup.bash` in each new terminal.
- **Status:** Solved and included in the run instructions.

### 35. Generated files should not be committed

- **Problem:** Colcon products, Python caches, Gazebo state, logs, recordings, editor files, and temporary files could clutter the first full project commit.
- **Solution:** Added a package `.gitignore` covering build/install/log outputs, caches, local environments, Gazebo state, rosbag files, logs, crashes, and editor metadata while explicitly keeping robot source assets version-controlled.
- **Status:** Solved.

### 36. Base mass and experimental values changed repeatedly

- **History:** Base values of 5 kg and 12 kg were discussed or tested while investigating COM behavior. IMU thresholds, pre-lean angles, foot sizes, and J6 range also went through temporary experiments.
- **Solution:** Keep the current validated values in the configuration table at the top of this document and label superseded experiments clearly. The current base mass is 4.0 kg.
- **Status:** Solved as a documentation/configuration-control issue.


### 37. WASD forward walking did not match the validated main gait

- **Symptom:** The standalone short-step presentation walked correctly, but pressing `W` still ran the older continuously generated sinusoidal IK gait.
- **Cause:** `wasd_ik_teleop` had its own 30 Hz forward gait generator and did not share the contact-locked keyframes from `short_step_walk_demo`.
- **Solution:** Made `short_step_walk_demo.py` the canonical importable gait source. Keyboard `W` now publishes those exact sequential phases, suspends IK output until the final hold, and `Space` cancels to the measured joint positions. Plain `W` uses 1.0x timing and `Shift+W` uses the validated 1.5x timing.
- **Validation:** The installed `Shift+W` path completed one 29-point right-left cycle in 17.80 seconds, advanced the pelvis 75.7 mm, stayed upright, and had unchanged final XYZ/RPY after a 10-second hold.
- **Status:** Historical result only; superseded by the running attempt and the narrow-stance fallback below.


### 38. Keyboard S, A, and D still used the unstable continuous IK preview

- **Symptom:** `W` used the stable presentation gait, but backward and turn keys could slip or fall because they still drove the old continuously generated IK targets.
- **Cause:** Only the forward keyboard path had been replaced. `S`, `A`, and `D` still published independent 30 Hz swing targets without preserving support-foot world contacts.
- **Solution:** Added fixed contact-aware directional phases. `S` mirrors the forward contact progression for a backward right-left cycle. `A` and `D` place both feet at a new 5-degree yaw in sequence, using Joint 3 for yaw and Joint 2 plus Joint 6 for lateral support transfer. Fixed trajectories suppress the continuous IK publisher until their final hold.
- **Validation at 1.5x timing:** `Shift+S` stayed upright and moved 42.4 mm backward. `Shift+A` reached +5.02 degrees yaw and `Shift+D` reached -4.98 degrees yaw. Roll stayed within 0.19 degrees and pitch near 1.80 degrees; all final poses remained unchanged during their settle checks.
- **Status:** Solved and dynamically validated in fresh Gazebo spawns for each direction.

## Recommended demonstration procedure

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ros2 launch openleg_description gazebo.launch.py spawn_z:=0.58
```

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run openleg_description short_step_walk_demo
```

Press Enter once the controller reports that the 20-step contact-locked walk is ready.

## Checks to run after future geometry or mass edits

```bash
cd ~/ros2_ws/src/openleg_description
python3 scripts/short_step_walk_demo.py --self-test
python3 scripts/overlap_walk_demo.py --self-test
python3 -m py_compile scripts/short_step_walk_demo.py scripts/overlap_walk_demo.py launch/gazebo.launch.py
git diff --check

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select openleg_description
colcon test --packages-select openleg_description
colcon test-result --verbose
```

Any change to mesh origins, joint origins, link masses, inertial origins, foot dimensions, hip spacing, or joint limits invalidates at least part of the current open-loop gait and should be followed by another full Gazebo physics run.

