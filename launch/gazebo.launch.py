#!/usr/bin/env python3
"""Spawn OpenLeg in Gazebo Sim and start its ros2_control controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
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
        launch_arguments={"gz_args": ["-r -v 3 ", world]}.items(),
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

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
    )
    leg_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["leg_controller", "--controller-manager-timeout", "60"],
    )

    velocity_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/model/openleg/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/openleg/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster, leg_controller])
    )

    resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([os.path.dirname(package_share), os.environ.get("GZ_SIM_RESOURCE_PATH", "")]),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty.sdf", description="Gazebo world SDF file"),
        DeclareLaunchArgument("spawn_z", default_value="0.60", description="Initial pelvis height in metres"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        resource_path,
        gazebo,
        state_publisher,
        velocity_bridge,
        spawn_robot,
        start_controllers,
    ])
