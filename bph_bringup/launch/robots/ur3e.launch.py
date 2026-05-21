"""
robots/ur3e.launch.py

UR3e-specific bringup for the spring controller study.

Starts:
  - UR robot driver (publishes /robot_description, starts ros2_control,
    joint_state_broadcaster)
  - MoveIt (ur_moveit_config)
  - scaled_joint_trajectory_controller in active state

Called by study_launch.py — do not run directly.

Parameters passed in from study_launch.py:
  robot_ip        : str  -- IP address of the UR3e
  urdf_path       : str  -- unused (UR driver publishes /robot_description)
  move_group_name : str  -- passed through for reference; UR default is ur_manipulator
  has_moveit      : str  -- always true for UR3e; declared for interface consistency
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


POSITION_CONTROLLER = "scaled_joint_trajectory_controller"


def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="10.3.4.10",
        description="IP address of the UR3e.",
    )
    # Declared for interface consistency with study_launch.py;
    # not used here since the UR driver publishes /robot_description itself.
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path", default_value=""
    )
    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name", default_value="ur_manipulator"
    )
    has_moveit_arg = DeclareLaunchArgument(
        "has_moveit", default_value="true"
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="false"
    )
    kinematics_params_arg = DeclareLaunchArgument(
        "kinematics_params",
        default_value="",
        description=(
            "Path to kinematics calibration YAML from "
            "ros2 launch ur_calibration calibration_correction.launch.py"
        ),
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Use fake UR hardware for testing without a real robot.",
    )

    # ----------------------------------------------------------------
    # UR robot driver
    # Publishes /robot_description, starts ros2_control hardware,
    # joint_state_broadcaster, and the UR-specific controllers.
    # ----------------------------------------------------------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch", "ur3e.launch.py",
            ])
        ]),
        launch_arguments={
            "robot_ip":          LaunchConfiguration("robot_ip"),
            "launch_rviz":       LaunchConfiguration("launch_rviz"),
            "kinematics_params": LaunchConfiguration("kinematics_params"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        }.items(),
    )

    # ----------------------------------------------------------------
    # MoveIt
    # Delayed slightly to let the driver finish starting up.
    # ----------------------------------------------------------------
    moveit_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("ur_moveit_config"),
                        "launch", "ur_moveit.launch.py",
                    ])
                ]),
                launch_arguments={
                    "ur_type":    "ur3e",
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                }.items(),
            )
        ],
    )

    # ----------------------------------------------------------------
    # Position controller
    # Loaded after the driver starts. equilibrium_mover needs this
    # active before it can plan and execute a trajectory.
    # ----------------------------------------------------------------
    load_position_controller = TimerAction(
        period=8.0,   # give driver + MoveIt time to come up
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
        kinematics_params_arg,
        use_fake_hardware_arg,
        ur_driver,
        moveit_bringup,
        load_position_controller,
    ])
