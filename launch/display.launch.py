#!/usr/bin/env python3
"""Display and articulate OpenLeg in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("openleg_description")
    model = os.path.join(package_share, "urdf", "main.urdf.xacro")
    rviz_config = os.path.join(package_share, "rviz", "openleg.rviz")
    use_gui = LaunchConfiguration("use_gui")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model, " use_gazebo:=false"]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),
        Node(package="rviz2", executable="rviz2", output="screen", arguments=["-d", rviz_config]),
    ])
