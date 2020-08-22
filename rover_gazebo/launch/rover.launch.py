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
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={'namespace': '',
                              'use_sim_time': 'false',
                              'autostart': 'true',
                              'params_file': get_package_share_directory("rover") + '/config/nav2_params.yaml',
                              'bt_xml_file': os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items())

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

    slam_localization = Node(
        parameters=[
            get_package_share_directory("rover_gazebo") + '/config/slam_localization.yaml'#,
         #   {'use_sim_time': True}
        ],
        package='slam_toolbox',
        node_executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        emulate_tty=True,
        output='screen'
    )

    slam_mapping = Node(
        parameters=[
            get_package_share_directory("rover_gazebo") + '/config/slam_mapping.yaml'
        ],
        package='slam_toolbox',
        node_executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        emulate_tty=True,
        output='screen'
    )

    bumper2pc = Node(
        package="bumper2pc",
        executable="bumper2pc",
        name="bumper2pc",
        output="screen",
        emulate_tty=True,
        parameters=[
            {   "pointcloud_radius": 0.35,
                "pointcloud_height": 0.01,
                "side_point_angle": 1.3,
                "base_link_frame": "base_link"#,
#                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_rover_gazebo, 'worlds', 'rover_empty.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
#        gazebo,
        rviz,
#        slam_mapping,
        bumper2pc,
        slam_localization,
        navigation,
    ])
