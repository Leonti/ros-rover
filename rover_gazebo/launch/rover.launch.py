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

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rover_gazebo = get_package_share_directory('rover_gazebo')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # RViz
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        arguments=['-d', os.path.join(pkg_rover_gazebo, 'rviz', 'rover_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    slam = Node(
        parameters=[
            get_package_share_directory("rover") + '/config/slam.yaml'
        ],
        package='slam_toolbox',
        node_executable='sync_slam_toolbox_node',
        #prefix=['gdb -ex=r --args'],
#        prefix=['valgrind'],
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_rover_gazebo, 'worlds', 'rover_empty.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        rviz,
        slam
    ])
