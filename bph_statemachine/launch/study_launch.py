"""
study_launch.py

Launch sequence:
  1. robot_state_publisher  (URDF → /robot_description  TF)
  2. ros2_control hardware  joint_state_broadcaster
  3. scaled_joint_trajectory_controller  (position control for safe move-to-start)
  4. equilibrium_mover  (solve  execute slow trajectory to spring equilibrium)
  5. On success: switch to forward_effort_controller, start virtual_spring_node
  6. On failure: shut everything down cleanly

Adjust controller names to match your ros2_control YAML config.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


POSITION_CONTROLLER = "scaled_joint_trajectory_controller"
EFFORT_CONTROLLER   = "forward_effort_controller"


def generate_launch_description():

    config_arg = DeclareLaunchArgument(
        "config_path",
        description="Path to springs YAML config file",
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf_path", default_value="",
        description="Fallback URDF path (only needed without robot_state_publisher)",
    )

    config_path = LaunchConfiguration("config_path")
    urdf_path   = LaunchConfiguration("urdf_path")

    # ----------------------------------------------------------------
    # 1. robot_state_publisher — publishes /robot_description  TF
    #    (Edit xacro_file to point at your robot description package)
    # ----------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        # parameters=[{"robot_description": ...}]  # add your URDF/xacro here
    )

    # ----------------------------------------------------------------
    # 2. ros2_control node  joint state broadcaster
    # ----------------------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        # parameters=[your_ros2_control_config]
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )

    # ----------------------------------------------------------------
    # 3. Start in position control so equilibrium_mover can move safely
    # ----------------------------------------------------------------
    load_position_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", POSITION_CONTROLLER],
        output="screen",
    )

    # ----------------------------------------------------------------
    # 4. equilibrium_mover — exits 0 on success, 1 on failure
    # ----------------------------------------------------------------
    equilibrium_mover = Node(
        package="your_study_pkg",          # <-- update to your package name
        executable="equilibrium_mover",
        name="equilibrium_mover",
        output="screen",
        parameters=[{
            "config_path":      config_path,
            "urdf_path":        urdf_path,
            "move_group_name":  "ur_manipulator",
            "velocity_scaling": 0.1,
            "accel_scaling":    0.1,
        }],
    )

    # ----------------------------------------------------------------
    # 5a. On success: switch controllers, start spring node
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
            "config_path":              config_path,
            "urdf_path":                urdf_path,
            "add_gravity_compensation": False,
            "recentering_threshold_rad": 0.3,
        }],
    )

    on_mover_success = RegisterEventHandler(
        OnProcessExit(
            target_action=equilibrium_mover,
            on_exit=[switch_to_effort, virtual_spring_node],
        )
    )

    # ----------------------------------------------------------------
    # 5b. On failure: shut everything down rather than starting effort
    #     control with no safe starting position
    # ----------------------------------------------------------------
    on_mover_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=equilibrium_mover,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason="equilibrium_mover failed — not starting effort controller"
                ))
            ],
        )
    )

    return LaunchDescription([
        config_arg,
        urdf_arg,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        load_position_controller,
        equilibrium_mover,
        on_mover_success,
        on_mover_failure,
    ])
