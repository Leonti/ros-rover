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

    arduino_bridge = Node(
        package='arduino_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        emulate_tty=True,
        output='screen'
    )

    rpi_bumper = Node(
        package='rpi_bumper',
        executable='rpi_bumper',
        name='rpi_bumper',
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

    return LaunchDescription([
        lidar,
        arduino_bridge,
        rpi_bumper,
        base_to_scan
    ])
