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
    pkg_rover_gazebo = get_package_share_directory('rover_gazebo')

    pico_bridge = Node(
        package='pico_bridge',
        executable='pico_bridge',
        name='pico_bridge',
        emulate_tty=True,
        output='screen'
    )

    hardware_control = Node(
        package='hardware_control',
        executable='hardware_control',
        name='hardware_control',
        emulate_tty=True,
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_rover_gazebo, 'rviz', 'rover_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_rover_gazebo, 'worlds', 'rover_empty.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        rviz,
    ])
