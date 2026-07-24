#!/usr/bin/env python3
"""Spawn OpenLeg in Gazebo Sim and start its ros2_control controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "openleg_description"
    package_share = get_package_share_directory(package_name)
    model = os.path.join(package_share, "urdf", "main.urdf.xacro")

    world = LaunchConfiguration("world")
    spawn_z = LaunchConfiguration("spawn_z")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controllers_file = os.path.join(package_share, "config", "controllers.yaml")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ", model,
            " use_gazebo:=true",
            " controllers_file:=", controllers_file,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-v 3 ", world]}.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "openleg", "-z", spawn_z],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster", "leg_controller", "--inactive",
            "--controller-manager-timeout", "60",
        ],
    )

    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[controller_spawner])
    )

    unpause_simulation = ExecuteProcess(
        cmd=[
            "gz", "service", "-s", "/world/openleg/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000", "--req", "pause: false",
        ],
        output="screen",
    )
    wait_for_teleop = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "until ros2 node list 2>/dev/null "
            "| grep -Eq '^/(wasd_ik_teleop|fixed_gait_demo|right_leg_lift_test|short_step_walk_demo|overlap_walk_demo|backward_bound_demo)$'; "
            "do sleep 0.1; done",
        ],
        output="screen",
    )
    prime_simulation = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "for _openleg_step in {1..10}; do "
            "gz service -s /world/openleg/control "
            "--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
            "--timeout 5000 --req 'multi_step: 10' >/dev/null; "
            "sleep 0.15; done",
        ],
        output="screen",
    )
    activate_controllers = ExecuteProcess(
        cmd=[
            "ros2", "control", "switch_controllers",
            "--activate", "joint_state_broadcaster", "leg_controller",
            "--strict", "--activate-asap",
        ],
        output="screen",
    )
    wait_for_balance_controller = RegisterEventHandler(
        OnProcessExit(target_action=controller_spawner, on_exit=[wait_for_teleop])
    )
    start_simulation = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_teleop,
            on_exit=[
                TimerAction(period=4.0, actions=[activate_controllers]),
                TimerAction(period=1.5, actions=[prime_simulation]),
                TimerAction(period=5.0, actions=[unpause_simulation]),
            ],
        )
    )

    resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([os.path.dirname(package_share), os.environ.get("GZ_SIM_RESOURCE_PATH", "")]),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare(package_name), "worlds", "openleg.sdf"]
            ),
            description="Gazebo world SDF file",
        ),
        DeclareLaunchArgument("spawn_z", default_value="0.545", description="Initial pelvis height in metres"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        resource_path,
        gazebo,
        state_publisher,
        sensor_bridge,
        spawn_robot,
        start_controllers,
        wait_for_balance_controller,
        start_simulation,
    ])
