#!/usr/bin/env python3
"""
demo.launch.py
--------------
Launches the full block-painting-helper demo stack:
  0. rosbridge          — turtlebot communication
  1. navstack           - load map and Turtlebot bridge
  1.5 navigator_node navigation server
  2. person_tracker      — camera, static TF, person tracker, top-down viz
  3. arm.launch.py       — UR3e driver + virtual spring controller + torque relay 4. moveit              - UR3e moveit server
  5. bph_pickmeup_node   — arm pick-and-place action server
  5.5 overhead webcam - v4l2_camera camera node publishing on /image_raw
                         currently loaded separately from different computer
  6. color_picker_node   — overhead-camera colour-based object localisation
  7. simple_sm_node      — top-level SMACH state machine

FIXME: document all the parameters here

Example:
  ros2 launch bph_statemachine demo.launch.py use_sim_time:=false
  ros2 launch bph_statemachine demo.launch.py cam_z:=2.0 
"""

import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler, EmitEvent, Shutdown
from launch.events import Shutdown as ShutdownEvent
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess

SetEnvironmentVariable('CYCLONEDDS_URI', 'file:///home/katallen/cyclonedds.xml')


def generate_launch_description():
    nav_pkg     = get_package_share_directory("nav_to_goal")
    tracker_pkg = get_package_share_directory("person_tracker")
    # bph_statemachine shares its launch dir with arm.launch.py
    bph_sm_pkg  = get_package_share_directory("bph_statemachine")
    moveit_pkg = get_package_share_directory("ur_moveit_config")
    nav2_pkg = get_package_share_directory("nav2_bringup")
    slam_pkg = get_package_share_directory("slam_toolbox")
    
    # ── Inject venv site-packages so all nodes pick up extra dependencies ────
    _venv_site = (
        "/home/katallen/.ros_venv/lib/"
        f"python{sys.version_info.major}.{sys.version_info.minor}"
        "/site-packages"
    )
    set_pythonpath = SetEnvironmentVariable(
        name="PYTHONPATH",
        value=_venv_site + ":" + os.environ.get("PYTHONPATH", ""),
    )

    # ── Shared / navigation args ─────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock",
    )

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(nav_pkg, "config", "slam_params.yaml"),
        description="slam_toolbox parameter YAML",
    )
    goal_x_arg = DeclareLaunchArgument("goal_x", default_value="2.0")
    goal_y_arg = DeclareLaunchArgument("goal_y", default_value="1.0")
    goal_yaw_arg = DeclareLaunchArgument("goal_yaw", default_value="0.0")

    depth_image_topic_arg = DeclareLaunchArgument(
        "depth_image_topic", default_value="/tb-camera/depth/image_raw",
    )
    depth_info_topic_arg = DeclareLaunchArgument(
        "depth_info_topic", default_value="/tb-camera/depth/camera_info",
    )
    robot_base_frame_arg = DeclareLaunchArgument(
        "robot_base_frame", default_value="turtlebot/base_link",
    )
    
    # ── Overhead camera pose args ────────────────────────────────────────────
    cam_x_arg     = DeclareLaunchArgument("cam_x",     default_value="0.0")
    cam_y_arg     = DeclareLaunchArgument("cam_y",     default_value="0.0")
    cam_z_arg     = DeclareLaunchArgument("cam_z",     default_value="2.5")
    cam_roll_arg  = DeclareLaunchArgument("cam_roll",  default_value="0.0")
    cam_pitch_arg = DeclareLaunchArgument(
        "cam_pitch", default_value=str(math.pi / 2),
    )
    cam_yaw_arg   = DeclareLaunchArgument("cam_yaw",   default_value="0.0")

    # ── UR arm / spring-controller args ─────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="10.3.4.10",
        description="IP address of the UR3e robot.",
    )
    kinematics_params = DeclareLaunchArgument(
        "kinematics_params",
        default_value="/home/katallen/my_robot_calibration.yaml",
        description="Kinematics Calibration file made from ros2 launch ur_calibration calibration_correction.launch.py",
        )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Whether to launch RViz alongside the UR driver.",
    )

    moveit_params_file = PathJoinSubstitution([
            EnvironmentVariable("ROS_WS", default_value="/home/katallen/sandbox"),
            "src/block-painting-helper/config/moveit_params.yaml"])

    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution([
            EnvironmentVariable("ROS_WS", default_value="/home/katallen/sandbox"),
            "src/springcontroller/springcontroller/flat_urdf_files/ceeorobot_flat.urdf"
    ]),
        description="Absolute path to the URDF used by the virtual spring node.",
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use fake hardware for UR arm",
    )


    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([EnvironmentVariable("ROS_WS", default_value="/home/katallen/sandbox"),
            "src/block-painting-helper/hayley-nav2/nav2_params.yaml"
    ]),
        description="Parameter file for nav2",
        )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([EnvironmentVariable("ROS_WS",
        default_value="/home/katallen/sandbox"),
        "src/block-painting-helper/maps/kat_lab_map.yaml"]),
        description="map file for Nav2",
        )
    
    springconfig_arg = DeclareLaunchArgument(
        "springconfig",
        default_value=PathJoinSubstitution([EnvironmentVariable("ROS_WS", default_value="/home/katallen/sandbox"),
            "src/springcontroller/springcontroller/config/demosprings.yaml"
    ]),
    )
            

    joint_order_arg = DeclareLaunchArgument(
        "joint_order",
        default_value=(
            "[elbow_joint, shoulder_lift_joint, shoulder_pan_joint,"
            " wrist_1_joint, wrist_2_joint, wrist_3_joint]"
        ),
        description="Ordered joint names for the torque relay.",
    )
    torque_topic_arg = DeclareLaunchArgument(
        "torque_topic",
        default_value="/virtual_spring_node/joint_torques",
        description="Input topic carrying spring torques (sensor_msgs/JointState).",
    )
    command_topic_arg = DeclareLaunchArgument(
        "command_topic",
        default_value="/forward_effort_controller/commands",
        description="Output topic sent to the effort controller.",
    )

    # ── Colour-picker args ───────────────────────────────────────────────────
    color_image_topic_arg = DeclareLaunchArgument(
        "color_image_topic",
        default_value="/bph_overhead_camera/image_raw",
        description=(
            "RGB image topic for the colour-picker node.  "
            "Should be the same physical camera used by person_tracker."
        ),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")


    
    # ── 0. ROSBridge ─────────────────────────────────────────────────────────
    turtlebridge_bringup = Node(
        package="nav_to_goal",
        executable="turtlebot_bridge",
        name="turtle_bridge",
        remappings=[
        ('/joint_states', '/turtlebot/joint_states'),
        ],
        output="screen",
    )

    # ── 1. Nav2   ──────────────────────────────────────
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time":      use_sim_time,
            "params_file":       LaunchConfiguration("params_file"),
            "map":               LaunchConfiguration("map_file"),
        }.items(),
    )


    # ── 4. Navigator goal node ─────────────────────────────────────────────
    navigator_node = Node(
        package="nav_to_goal",
        executable="navigator_node",
        name="navigator_node",
        output="screen",
        parameters=[{
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "goal_yaw": LaunchConfiguration("goal_yaw"),
            "robot_base_frame": LaunchConfiguration("robot_base_frame"),
            "use_sim_time": use_sim_time,
        }],
    )

    
# #    slam_bringup = IncludeLaunchDescription(
# #        PythonLaunchDescriptionSource(
# #            os.path.join(slam_pkg,
#                          "launch",
#                          "online_async_launch.py")
#         ),
#             launch_arguments={
#                 "use_sim_time": use_sim_time,
#             }.items(),
#             )
    
    # IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav_pkg, "launch", "bringup.launch.py")
    #     ),
    #     launch_arguments={
    #         "slam_params_file":  LaunchConfiguration("slam_params_file"),
    #         "depth_image_topic": LaunchConfiguration("depth_image_topic"),
    #         "depth_info_topic":  LaunchConfiguration("depth_info_topic"),
    #         "robot_base_frame":  LaunchConfiguration("robot_base_frame"),
    #         "navigate_on_start": "false",   # state machine drives navigation
    #     }.items(),
    # )

    # ── 2. Overhead camera + person tracker + top-down viz ───────────────────
    person_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tracker_pkg, "launch", "person_tracker.launch.py")
        ),
        launch_arguments={
            "cam_x":     LaunchConfiguration("cam_x"),
            "cam_y":     LaunchConfiguration("cam_y"),
            "cam_z":     LaunchConfiguration("cam_z"),
            "cam_roll":  LaunchConfiguration("cam_roll"),
            "cam_pitch": LaunchConfiguration("cam_pitch"),
            "cam_yaw":   LaunchConfiguration("cam_yaw"),
            "color_image_topic": LaunchConfiguration("color_image_topic"),
        }.items(),
    )

    # ── 3. Top-level SMACH state machine ─────────────────────────────────────
    state_machine_node = Node(
        package="bph_statemachine",
        executable="simple_sm_node",
        name="robot_fetch_smach",
        output="screen",
        parameters=[{
        "supply_closet_x": -1.109,   # actual supply closet position
        "supply_closet_y": -0.803,
        "supply_closet_yaw": 0.0,
        }],
    )


    shutdown_on_sm_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=state_machine_node,
            on_exit=[EmitEvent(event=ShutdownEvent())]
        )
)
    
    # ── 4. Arm pick-and-place action server ───────────────────────────────────
    pickmeup_node = Node(
        package="bph_pickmeup",
        executable="bph_pickmeup_actionserver",
        name="bph_pickmeup_action",
        output="screen",
    )

    moveit_bringup = IncludeLaunchDescription(  
         PythonLaunchDescriptionSource(
             os.path.join(bph_sm_pkg, "launch","ur_moveit.launch.py")),
         launch_arguments={
             "ur_type": "ur3e",
             "robot_ip": LaunchConfiguration("robot_ip"),
             "launch_rviz": LaunchConfiguration("launch_rviz")}.items(),
        
     )

#    set_start_tolerance = TimerAction(
#        period=10.0,  # give MoveIt time to fully start
#        actions=[ExecuteProcess(
#            cmd=['ros2', 'param', 'set', '/move_group',
#                 'trajectory_execution.allowed_start_tolerance', '0.05'],
#            output='screen',
#        )]
#    )

    collision_scene_publisher = Node(
        package="bph_pickmeup",
        executable="bph_collision_scene",
        name="bph_pickmeup_collision_scene",
        output="screen",
    )


    # ── 5. UR3e driver + virtual spring + torque relay ───────────────────────
    #
    #  arm.launch.py lives in the same launch/ directory as this file.
    #  It starts: ur_robot_driver, virtual_spring_node, torque_relay.
    #
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bph_sm_pkg, "launch", "arm.launch.py")
        ),
        launch_arguments={
            "robot_ip":      LaunchConfiguration("robot_ip"),
            "launch_rviz":   LaunchConfiguration("launch_rviz"),
            "urdf_path":     LaunchConfiguration("urdf_path"),
            "config":        LaunchConfiguration("springconfig"),
            "joint_order":   LaunchConfiguration("joint_order"),
            "torque_topic":  LaunchConfiguration("torque_topic"),
            "command_topic": LaunchConfiguration("command_topic"),
            "kinematics_params": LaunchConfiguration("kinematics_params"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "springconfig": LaunchConfiguration("springconfig")
        }.items(),
    )

    
    # ── 6. Colour-based object picker (perception) ───────────────────────────
    #
    #  Node lives in bph_perception package.
    #  It shares the overhead camera RGB + depth streams with person_tracker.
    #  Exposes:
    #    ~/get_target_pose  (service)  — called by the state machine
    #    ~/set_target_color (service)  — lets the SM change colour at runtime
    #    ~/debug_image      (topic)    — annotated image for rviz / debugging
    #
    color_picker_node = Node(
        package="bph_perception",
        executable="color_picker",
        name="color_picker",
        output="screen",
        parameters=[{
            "color_image_topic": LaunchConfiguration("color_image_topic"),
        }],
    )

    arm_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arm_base_to_map_tf',
        arguments=[
            '0.2176', '0.665', '0.936',  # x, y, z in map frame
            '0.0', '0.0', '0.0',         # yaw, pitch, roll
            'map', 'world'
        ]
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_arm_tf',
        arguments=[
            '0.044', '-0.010', '1.785',   # x, y, z above arm base
            '0.0', str(math.pi), '0.0',   # yaw, pitch, roll — pointing down
            'world',
            'bph_overhead_camera_optical_frame'  # must match camera_info frame_id
        ]
    )

    initial_pose = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
             'geometry_msgs/msg/PoseWithCovarianceStamped',
             '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0}}, "covariance": [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.07]}}'],
        output='screen',
    )

#################User Interface ##############

    UI_rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='UI_rosbridge_websocket',
        parameters=[{
            "port": 9405,
            "qos_overrides./button.subscription.durability":"volatile",
            "qos_overrides./requestedmaterial.subscription.durability":"volatile"}
                    ]
    )

    UI_server =  ExecuteProcess(
            cmd=['python3', '/home/katallen/sandbox/src/block-painting-helper/bph_userinterface/bph_ui_server.py'],
            output='screen',
            cwd='/home/katallen/sandbox/src/block-painting-helper/',  # if it needs relative paths
        )
    
    
    
    return LaunchDescription([
        set_pythonpath,             # must come first

        # declare all args
        use_sim_time_arg,
        params_file_arg,
        slam_params_file_arg,
        map_file_arg,
        depth_image_topic_arg,
        depth_info_topic_arg,
        robot_base_frame_arg,
        goal_x_arg, goal_y_arg, goal_yaw_arg,
        cam_x_arg, cam_y_arg, cam_z_arg,
        cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
        robot_ip_arg,
        launch_rviz_arg,
        kinematics_params,
        urdf_path_arg,
        use_fake_hardware_arg,
        springconfig_arg,
        joint_order_arg,
        torque_topic_arg,
        command_topic_arg,
        color_image_topic_arg,
        #set_start_tolerance, # gives moveit more tolerance for arm joint position error
        shutdown_on_sm_exit,


        # start nodes / sub-launches
        LogInfo(msg="[demo] Starting ROSBridge ..."),
        turtlebridge_bringup,

        LogInfo(msg="[demo] Starting Nav2 + localization ..."),
        
        TimerAction(
            period=5.0,
            actions=[nav_bringup],
        ),

        TimerAction(period=15.0, actions=[initial_pose]),
        navigator_node,
        
        LogInfo(msg="[demo] Check that v4l2_camera is loaded..."),
        #camera_bringup,

        #LogInfo(msg="[demo] Starting person tracker ..."),
        #person_tracker,
        LogInfo(msg="[demo] Skipping person tracker ..."),

        LogInfo(msg="[demo] Starting UR3e arm + spring controller ..."),
        arm_bringup,
        collision_scene_publisher,
        LogInfo(msg="[demo] Starting UR3e MoveIt ..."),
        moveit_bringup, 

        
        LogInfo(msg="[demo] Starting arm pick-and-place server ..."),
        pickmeup_node,

        LogInfo(msg="[demo] Starting colour-picker perception node ..."),
        color_picker_node,

        LogInfo(msg="[demo] Starting SMACH state machine ..."),
        state_machine_node,

        LogInfo(msg="[demo] Loading static transforms for camera and world to map..."),
        arm_base_tf,
        camera_tf,
        UI_rosbridge,
        UI_server,
    ])
