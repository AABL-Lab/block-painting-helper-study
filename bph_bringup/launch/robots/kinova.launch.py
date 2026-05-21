"""
robots/kinova.launch.py

Kinova-specific bringup for the spring controller study.

Starts:
  - Kinova ros2_kortex driver (publishes /robot_description, starts
    ros2_control, joint_state_broadcaster)
  - MoveIt (kinova_gen3_moveit_config or equivalent)
  - joint_trajectory_controller in active state

Called by study_launch.py — do not run directly.

Parameters passed in from study_launch.py:
  robot_ip        : str  -- leave empty to use Kinova default "192.168.1.10"
                           or pass "local" convention from your setup
  move_group_name : str  -- Kinova default is "manipulator"
  has_moveit      : str  -- always true for Kinova; declared for consistency

NOTE: Update the package names below to match your Kinova ROS2 driver install.
The kortex_driver package names vary between ros2_kortex versions.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


POSITION_CONTROLLER = "joint_trajectory_controller"
KINOVA_DEFAULT_IP   = "192.168.1.10"


def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value=KINOVA_DEFAULT_IP,
        description=(
            "IP address of the Kinova arm. "
            "Leave at default or pass empty string to use kortex default."
        ),
    )
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path", default_value="",
    )
    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name", default_value="manipulator",
    )
    has_moveit_arg = DeclareLaunchArgument(
        "has_moveit", default_value="true",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="false",
    )

    # ----------------------------------------------------------------
    # Kinova kortex driver
    # Publishes /robot_description, starts ros2_control hardware,
    # and joint_state_broadcaster.
    # Update package/launch names to match your kortex install.
    # ----------------------------------------------------------------
    kinova_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("kortex_bringup"),  # <-- update if needed
                "launch", "gen3.launch.py",          # <-- update if needed
            ])
        ]),
        launch_arguments={
            "robot_ip":   LaunchConfiguration("robot_ip"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    # ----------------------------------------------------------------
    # MoveIt
    # ----------------------------------------------------------------
    moveit_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("kinova_gen3_moveit_config"),  # <-- update if needed
                        "launch", "move_group.launch.py",
                    ])
                ]),
                launch_arguments={
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                }.items(),
            )
        ],
    )

    # ----------------------------------------------------------------
    # Position controller
    # ----------------------------------------------------------------
    load_position_controller = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "control", "load_controller",
                    "--set-state", "active",
                    POSITION_CONTROLLER,
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        robot_ip_arg,
        urdf_path_arg,
        move_group_name_arg,
        has_moveit_arg,
        launch_rviz_arg,
        kinova_driver,
        moveit_bringup,
        load_position_controller,
    ])
