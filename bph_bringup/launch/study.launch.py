"""
study_launch.py

Robot-agnostic top-level launch file for the spring controller study.

Launch sequence:
  1. Robot-specific bringup (hardware driver, /robot_description, MoveIt,
     joint_state_broadcaster, position controller) via robots/<name>.launch.py
  2. equilibrium_mover — solves for spring equilibrium, moves there slowly
  3. On success: switch to effort controller, start virtual_spring_node
  4. On failure: shut everything down cleanly

Usage:
  ros2 launch your_study_pkg study_launch.py \
      robot:=ur3e \
      robot_ip:=10.3.4.10 \
      config_path:=/path/to/springs.yaml \
      srdf_path:=/path/to/robot.srdf

  ros2 launch your_study_pkg study_launch.py \
      robot:=2dof \
      config_path:=/path/to/springs.yaml \
      urdf_path:=/path/to/2dof.urdf
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


POSITION_CONTROLLER = "scaled_joint_trajectory_controller"
EFFORT_CONTROLLER   = "forward_effort_controller"


def generate_launch_description():

    # ----------------------------------------------------------------
    # Robot selection
    # ----------------------------------------------------------------
    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="ur3e",
        description="Robot to use: ur3e | kinova | 2dof",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="",
        description=(
            "Robot IP address. ur3e: e.g. 10.3.4.10. "
            "kinova: leave empty (uses 'local'). 2dof: ignored."
        ),
    )

    # ----------------------------------------------------------------
    # URDF / SRDF — urdf_path only needed for 2dof or when
    # robot_state_publisher is not available.
    # ----------------------------------------------------------------
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="",
        description=(
            "Fallback URDF path. Only needed when /robot_description "
            "topic is unavailable (e.g. 2dof fake arm)."
        ),
    )
    srdf_path_arg = DeclareLaunchArgument(
        "srdf_path",
        default_value="",
        description=(
            "Path to robot SRDF for collision pair filtering. "
            "Optional — if omitted, adjacent-link pairs are not filtered."
        ),
    )

    # ----------------------------------------------------------------
    # Spring controller parameters
    # ----------------------------------------------------------------
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        description="Path to springs YAML config file.",
    )
    danger_threshold_arg = DeclareLaunchArgument(
        "danger_threshold",
        default_value="0.05",
        description=(
            "Distance in metres at which spring torques begin scaling "
            "toward zero to avoid self-collision."
        ),
    )
    recentering_threshold_arg = DeclareLaunchArgument(
        "recentering_threshold_rad",
        default_value="0.3",
        description=(
            "Maximum joint shift (rad) from current position to new "
            "equilibrium before a re-centering move is triggered."
        ),
    )
    gravity_comp_arg = DeclareLaunchArgument(
        "add_gravity_compensation",
        default_value="false",
        description=(
            "Add software gravity compensation torques. Set false for "
            "arms that handle gravity comp in hardware (e.g. UR3e)."
        ),
    )

    # ----------------------------------------------------------------
    # MoveIt / motion parameters
    # ----------------------------------------------------------------
    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name",
        default_value="ur_manipulator",
        description=(
            "MoveIt move group name. "
            "ur3e: ur_manipulator. kinova: manipulator."
        ),
    )
    has_moveit_arg = DeclareLaunchArgument(
        "has_moveit",
        default_value="true",
        description=(
            "Whether this robot has MoveIt available. "
            "Set false for 2dof fake hardware."
        ),
    )
    velocity_scaling_arg = DeclareLaunchArgument(
        "velocity_scaling",
        default_value="0.1",
        description="MoveIt velocity scaling for move-to-equilibrium (0.0-1.0).",
    )
    accel_scaling_arg = DeclareLaunchArgument(
        "accel_scaling",
        default_value="0.1",
        description="MoveIt acceleration scaling for move-to-equilibrium (0.0-1.0).",
    )

    # ----------------------------------------------------------------
    # Robot-specific bringup
    # Delegates to robots/<robot>.launch.py which is responsible for:
    #   - publishing /robot_description
    #   - starting ros2_control + joint_state_broadcaster
    #   - loading POSITION_CONTROLLER in active state
    #   - starting MoveIt (except 2dof)
    # ----------------------------------------------------------------
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("bph_bringup"), 
                "launch", "robots",
                [LaunchConfiguration("robot"), ".launch.py"],
            ])
        ]),
        launch_arguments={
            "robot_ip":        LaunchConfiguration("robot_ip"),
            "urdf_path":       LaunchConfiguration("urdf_path"),
            "move_group_name": LaunchConfiguration("move_group_name"),
            "has_moveit":      LaunchConfiguration("has_moveit"),
        }.items(),
    )

    # ----------------------------------------------------------------
    # equilibrium_mover
    # Reads /robot_description, waits for /joint_states, solves for
    # equilibrium, moves there, then exits 0.
    # ----------------------------------------------------------------
    equilibrium_mover = Node(
        package="your_study_pkg",           # <-- update package name
        executable="equilibrium_mover",
        name="equilibrium_mover",
        output="screen",
        parameters=[{
            "config_path":              LaunchConfiguration("config_path"),
            "urdf_path":                LaunchConfiguration("urdf_path"),
            "srdf_path":                LaunchConfiguration("srdf_path"),
            "danger_threshold":         LaunchConfiguration("danger_threshold"),
            "move_group_name":          LaunchConfiguration("move_group_name"),
            "has_moveit":               LaunchConfiguration("has_moveit"),
            "velocity_scaling":         LaunchConfiguration("velocity_scaling"),
            "accel_scaling":            LaunchConfiguration("accel_scaling"),
            "add_gravity_compensation": LaunchConfiguration("add_gravity_compensation"),
        }],
    )

    # ----------------------------------------------------------------
    # On equilibrium_mover success:
    # switch to effort controller, then start virtual_spring_node
    # ----------------------------------------------------------------
    switch_to_effort = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/controller_manager/switch_controller",
            "controller_manager_msgs/srv/SwitchController",
            "{{"
            f"activate_controllers: [{EFFORT_CONTROLLER}], "
            f"deactivate_controllers: [{POSITION_CONTROLLER}], "
            "strictness: 2"
            "}}",
        ],
        output="screen",
    )

    virtual_spring_node = Node(
        package="springcontroller",
        executable="virtual_spring_node",
        name="virtual_spring_node",
        output="screen",
        parameters=[{
            "config_path":               LaunchConfiguration("config_path"),
            "urdf_path":                 LaunchConfiguration("urdf_path"),
            "srdf_path":                 LaunchConfiguration("srdf_path"),
            "danger_threshold":          LaunchConfiguration("danger_threshold"),
            "add_gravity_compensation":  LaunchConfiguration("add_gravity_compensation"),
            "recentering_threshold_rad": LaunchConfiguration("recentering_threshold_rad"),
        }],
    )

    on_mover_success = RegisterEventHandler(
        OnProcessExit(
            target_action=equilibrium_mover,
            on_exit=[switch_to_effort, virtual_spring_node],
        )
    )

    # ----------------------------------------------------------------
    # On equilibrium_mover failure:
    # shut everything down rather than starting effort control with no
    # safe starting position
    # ----------------------------------------------------------------
    on_mover_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=equilibrium_mover,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason=(
                        "equilibrium_mover failed — "
                        "not starting effort controller"
                    )
                ))
            ],
        )
    )

    return LaunchDescription([
        # Robot selection
        robot_arg,
        robot_ip_arg,
        urdf_path_arg,
        srdf_path_arg,
        # Spring controller
        config_path_arg,
        danger_threshold_arg,
        recentering_threshold_arg,
        gravity_comp_arg,
        # Motion
        move_group_name_arg,
        has_moveit_arg,
        velocity_scaling_arg,
        accel_scaling_arg,
        # Launch sequence
        robot_bringup,
        equilibrium_mover,
        on_mover_success,
        on_mover_failure,
    ])
