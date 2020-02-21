## Install

Install instructions for Ubuntu Bionic.

1. Install the appropriate ROS 2 version as instructed [here](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).

1. Install `gazebo_ros_pkgs`

1. Clone repo:

        mkdir -p ~/ws/src
        cd ~/ws/src
        git clone https://github.com/Leonti/ros-rover

1. Build and install:

        cd ~/ws
        colcon build --symlink-install

## Run

1. Setup environment variables:

        . /usr/share/gazebo/setup.sh
        . /home/leonti/ros2_eloquent/install/setup.bash
        . /home/leonti/development/ros2_tutorial/install/setup.bash
        . /home/leonti/slam_ws/install/setup.bash

2. Launch rover:

        ros2 launch rover_gazebo rover.launch.py world:=rover_room.world

3. Install `teleop_twist_keyboard` https://github.com/ros2/teleop_twist_keyboard and 
use `from rclpy.qos import qos_profile_services_default`, modify topic to be `/rover/cmd_vel` and run with 

        ros2 run teleop_twist_keyboard teleop_twist_keyboard

3. In `slam_toolbox` worspace run:
 
        . /home/leonti/ros2_eloquent/install/setup.bash
        . /home/leonti/slam_ws/install/setup.bash
        
4. Launch `slam_toolbox`:
        ros2 launch slam_toolbox online_async_launch.py


## Credits
This project is based on the awesome "Dolly" project: https://github.com/chapulina/dolly
     