import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # --- CONFIGURATION ---
    # Using the symlinks you found in /dev/
    LIDAR_1_PORT = '/dev/neo-s300-1'
    LIDAR_2_PORT = '/dev/neo-s300-2'
    # ---------------------

    # LiDAR 1 - Front Right (usually neo-s300-1)
    lidar_1_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_s300_lidar_1',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_s300',
            'port': LIDAR_1_PORT,
            'hostname': '127.0.0.1',  # Dummy IP required by the driver logic
            'frame_id': 'lidar_1_link',
            'range_min': 0.1,
            'range_max': 30.0,
            'use_binary_protocol': True,
        }],
        remappings=[
            ('scan', '/lidar_1/scan'),  # Remapping to match your topic list
        ]
    )

    # LiDAR 2 - Back Left (usually neo-s300-2)
    lidar_2_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_s300_lidar_2',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_s300',
            'port': LIDAR_2_PORT,
            'hostname': '127.0.0.1',  # Dummy IP required by the driver logic
            'frame_id': 'lidar_2_link',
            'range_min': 0.1,
            'range_max': 30.0,
            'use_binary_protocol': True,
        }],
        remappings=[
            ('scan', '/lidar_2/scan'), # Remapping to match your topic list
        ]
    )

    return LaunchDescription([
        lidar_1_node,
        lidar_2_node
    ])
