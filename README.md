## Install

Install instructions for Ubuntu 20.04.

1. Install ROS 2 [Foxy Fitzroy](https://index.ros.org/doc/ros2/Installation/Foxy/).

2. Install dependencies:
`gazebo_ros_pkgs`: https://github.com/ros-simulation/gazebo_ros_pkgs
`slam_toolbox`: https://github.com/SteveMacenski/slam_toolbox
`navigation2`: https://github.com/ros-planning/navigation2

1. Clone repo:

        mkdir -p ~/ws/src
        cd ~/ws/src
        git clone https://github.com/Leonti/ros-rover

2. Build and install:

        cd ~/ws
        . /opt/ros/foxy/setup.bash
        colcon build --symlink-install

## Run

1. Setup environment variables:
```bash
. /usr/share/gazebo/setup.sh
. /opt/ros/foxy/setup.bash
. ~/development/ros2_tutorial/install/setup.bash
. ~/slam_ws/install/setup.bash
```

2. Launch rover (will launch Slam Toolbox as well):

```
ROS_DOMAIN_ID=45 ros2 launch rover_gazebo rover.launch.py world:=rover_room.world
```

3. Launch Navigation  

```bash
cd ~/navigation2_ws
. ~/navigation2_ws/install/setup.bash  
ROS_DOMAIN_ID=45 ros2 launch nav2_bringup navigation_launch.py
```

### Teleop
Install `teleop_twist_keyboard` https://github.com/ros2/teleop_twist_keyboard and 
use `from rclpy.qos import qos_profile_services_default`, and run with 

```bash
ROS_DOMAIN_ID=45 ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To debug TF:  

```bash
ROS_DOMAIN_ID=45 ros2 run tf2_tools view_frames.py
```

```bash
ROS_DOMAIN_ID=45 ros2 run tf2_ros tf2_echo map odom
```

To start with real lidar:

```bash
ROS_DOMAIN_ID=45 ros2 launch rover rover.py
```

To run QUI for topics and services:  

```bash
ROS_DOMAIN_ID=45 rqt
```

To build a single package:

```bash
colcon build --symlink-install --packages-select rplidar_ros
```

To publish to a topic:

```bash
ROS_DOMAIN_ID=45 ros2 topic pub -1 /pico_command std_msgs/msg/String "{data: 'C0:Some message'}"
```

### Navigation tutorial

```bash
source ../turtlebot_ws/install/setup.bash
source install/setup.bash
. /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/leonti/turtlebot_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup nav2_tb3_simulation_launch.py
```

## Electronics
Here is the schematic:
![Schematic](https://raw.githubusercontent.com/Leonti/ros-rover/master/wiring_schem.png)

Or in breadboard format:
![Schematic Breadboard](https://raw.githubusercontent.com/Leonti/ros-rover/master/wiring_bb.png)

## Credits
This project is based on the awesome "Dolly" project: https://github.com/chapulina/dolly
     
