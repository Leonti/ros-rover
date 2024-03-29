import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    lidar = Node(
        parameters=[   
            get_package_share_directory("rover") + '/config/rplidar.yaml'
        ],
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar',
        emulate_tty=True,
        output='screen'
    )    

    pico_bridge = Node(
        package='pico_bridge',
        executable='pico_bridge',
        name='pico_bridge',
        emulate_tty=True,
        output='screen'
    )

    base_to_scan = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_scan',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'scan'],
            output='screen')

    # temporary to investigate tf2 issues
    hardware_control = Node(
        package='hardware_control',
        executable='hardware_control',
        name='hardware_control',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        lidar,
        pico_bridge,
    #    hardware_control,
        base_to_scan
    ])
