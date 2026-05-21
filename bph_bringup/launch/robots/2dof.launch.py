"""
robots/2dof.launch.py

Fake 2-DOF arm bringup for testing the spring controller without real hardware.

Starts:
  - robot_state_publisher (publishes /robot_description from urdf_path)
  - fake_position_controller node (accepts joint trajectory commands,
    echoes them back on /joint_states so equilibrium_mover has something
    to read, and simulates slow motion between positions)

No MoveIt — equilibrium_mover will command joints directly via
JointTrajectory when has_moveit=false.

Called by study_launch.py — do not run directly.

Parameters passed in from study_launch.py:
  urdf_path  : str  -- required; path to the 2dof URDF file
  has_moveit : str  -- always false for 2dof; declared for consistency
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


POSITION_CONTROLLER_TOPIC = "/fake_position_controller/joint_trajectory"


def generate_launch_description():

    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        description="Path to the 2-DOF test arm URDF file. Required.",
    )
    # Declared for interface consistency; always false for 2dof
    has_moveit_arg = DeclareLaunchArgument(
        "has_moveit", default_value="false",
    )
    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name", default_value="",
    )
    # robot_ip is ignored for fake hardware but declared for consistency
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="",
    )

    # ----------------------------------------------------------------
    # robot_state_publisher
    # Reads the URDF from the file and publishes /robot_description
    # (transient-local) so virtual_spring_node and equilibrium_mover
    # can load it without needing a urdf_path parameter.
    # ----------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                "cat ", LaunchConfiguration("urdf_path")
            ]),
        }],
    )

    # ----------------------------------------------------------------
    # Fake position controller
    # Accepts JointTrajectory commands on
    # /fake_position_controller/joint_trajectory and publishes
    # joint states on /joint_states, interpolating slowly between
    # positions so equilibrium_mover can read a realistic starting state.
    #
    # This node lives in your_study_pkg — see fake_position_controller.py.
    # ----------------------------------------------------------------
    fake_controller = Node(
        package="your_study_pkg",           # <-- update package name
        executable="fake_position_controller",
        name="fake_position_controller",
        output="screen",
        parameters=[{
            "urdf_path":           LaunchConfiguration("urdf_path"),
            "publish_rate_hz":     50.0,
            "interpolation_sec":   5.0,    # time to move between positions
        }],
    )

    return LaunchDescription([
        urdf_path_arg,
        has_moveit_arg,
        move_group_name_arg,
        robot_ip_arg,
        robot_state_publisher,
        fake_controller,
    ])
