#!/usr/bin/env python3
"""
Main Hardware Bringup Launch File
Launches all hardware components for the Steve robot
"""

import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def execution_stage(
    context: LaunchContext,
    robot_namespace,
    arm_type,
    robot_ip,
    enable_camera,
    enable_pan_tilt,
):

    arm_typ = str(arm_type.perform(context))
    
    # Normalize booleans to lowercase string for xacro
    enable_cam = str(enable_camera.perform(context)).lower()
    enable_pt = str(enable_pan_tilt.perform(context)).lower()

    rp_ns = ""
    if robot_namespace.perform(context) != "/":
        rp_ns = robot_namespace.perform(context) + "/"

    launches = []
    pkg_share = get_package_share_directory("steve_hardware_bringup")

    # Process URDF with xacro (use main URDF from steve_simulation directly)
    # The wrapper mmo_700_real.urdf.xacro doesn't properly instantiate the robot
    neo_sim_pkg = get_package_share_directory("steve_simulation")
    urdf_file = os.path.join(neo_sim_pkg, "robots", "mmo_700", "mmo_700.urdf.xacro")

    if not os.path.exists(urdf_file):
        raise FileNotFoundError(
            f"URDF xacro file not found at {urdf_file}. "
            "Please ensure steve_simulation package is installed"
        )

    # Process xacro with appropriate arguments for real robot
    robot_description_content = xacro.process_file(
        urdf_file,
        mappings={
            "use_gazebo": "false",
            "arm_type": arm_typ,
            "include_wrist_camera": enable_cam,
            "include_depth_camera": "false",
            "include_pan_tilt": enable_pt,
        },
    ).toxml()

    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="steve_robot_state_publisher",
        output="screen",
        namespace=robot_namespace,
        parameters=[
            {"robot_description": robot_description_content, "frame_prefix": rp_ns}
        ],
    )

    launches.append(start_robot_state_publisher_cmd)

    # 1. Robot Base (Relayboard + Kinematics)
    robot_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "robot_base.launch.py")
        ),
        launch_arguments={"namespace": robot_namespace}.items(),
    )
    launches.append(robot_base)

    # 2. LiDAR Sensors
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "lidar.launch.py")
        ),
        launch_arguments={"namespace": robot_namespace}.items(),
    )
    launches.append(lidar)

    # 3. Teleop
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "teleop.launch.py")
        ),
        launch_arguments={"namespace": robot_namespace}.items(),
    )
    launches.append(teleop)

    # 4. UR5e Arm
    if arm_typ in ["ur5", "ur10", "ur5e", "ur10e"]:
        ur_arm = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "ur5e_arm.launch.py")
            ),
            launch_arguments={
                "ur_type": arm_typ,
                "robot_ip": robot_ip,
                "tf_prefix": arm_typ,
                "use_tool_communication": "true",
            }.items(),
        )
        launches.append(ur_arm)

    # 5. Pan-Tilt Unit
    if enable_pt == "true" or enable_pt == "True":
        pan_tilt = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "pan_tilt.launch.py")
            ),
            launch_arguments={
                "namespace": robot_namespace,
                "enable_camera": enable_camera,
            }.items(),
        )
        launches.append(pan_tilt)

    # Relaying lidar data to /scan topic
    relay_topic_lidar1 = Node(
        package="topic_tools",
        executable="relay",
        name="steve_relay_lidar1",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            {
                "input_topic": robot_namespace.perform(context)
                + "lidar_1/scan_filtered",
                "output_topic": robot_namespace.perform(context) + "scan",
            }
        ],
    )

    relay_topic_lidar2 = Node(
        package="topic_tools",
        executable="relay",
        name="steve_relay_lidar2",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            {
                "input_topic": robot_namespace.perform(context)
                + "lidar_2/scan_filtered",
                "output_topic": robot_namespace.perform(context) + "scan",
            }
        ],
    )

    launches.append(relay_topic_lidar1)
    launches.append(relay_topic_lidar2)

    return launches


def generate_launch_description():
    # Launch configurations
    robot_namespace = LaunchConfiguration("robot_namespace")
    arm_type = LaunchConfiguration("arm_type")
    robot_ip = LaunchConfiguration("robot_ip")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_pan_tilt = LaunchConfiguration("enable_pan_tilt")

    context_arguments = [
        robot_namespace,
        arm_type,
        robot_ip,
        enable_camera,
        enable_pan_tilt,
    ]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Top-level namespace for the robot",
    )

    declare_arm_cmd = DeclareLaunchArgument(
        "arm_type",
        default_value="ur5e",
        description="Arm type - Options: ur5, ur5e, ur10, ur10e",
    )

    declare_robot_ip_cmd = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.102",
        description="IP address of the UR arm",
    )

    declare_camera_cmd = DeclareLaunchArgument(
        "enable_camera",
        default_value="true",
        description="Enable RealSense L515 camera - Options: true/false",
    )

    declare_pan_tilt_cmd = DeclareLaunchArgument(
        "enable_pan_tilt",
        default_value="true",
        description="Enable pan-tilt motors - Options: true/false",
    )

    # Opaque function for configuring all hardware
    opq_function = OpaqueFunction(function=execution_stage, args=context_arguments)

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_arm_cmd)
    ld.add_action(declare_robot_ip_cmd)
    ld.add_action(declare_camera_cmd)
    ld.add_action(declare_pan_tilt_cmd)
    ld.add_action(opq_function)

    return ld
