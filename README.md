## Install

Install instructions for Ubuntu 19.10.

1. Install ros2 from source [here](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).

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
        . ~/ros2_dashing/install/setup.bash
        colcon build --symlink-install

## Run

1. Setup environment variables:
```bash
. /usr/share/gazebo/setup.sh
. ~/ros2_dashing/install/setup.bash
. ~/development/ros2_tutorial/install/setup.bash
. ~/slam_ws/install/setup.bash
. ~/navigation2_ws/install/setup.bash
```

2. Launch rover (will launch Slam Toolbox as well):

        ros2 launch rover_gazebo rover.launch.py world:=rover_room.world

3. Launch Navigation  

```bash
cd . ~/navigation2_ws
. ~/navigation2_ws/install/setup.bash  
ros2 launch nav2_bringup nav2_navigation_launch.py
```

### Teleop
Install `teleop_twist_keyboard` https://github.com/ros2/teleop_twist_keyboard and 
use `from rclpy.qos import qos_profile_services_default`, and run with 

```bash
        ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To debug TF:  

```bash
ros2 run tf2_tools view_frames.py
```

To start with real lidar:

```bash
ros2 launch rover rover.py
```

To build a single package:

```bash
colcon build --symlink-install --packages-select rplidar_ros
```

### Navigation tutorial

```bash
cd ~/navigation2_ws/
source ../turtlebot_ws/install/setup.bash
source install/setup.bash
. /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/leonti/turtlebot_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup nav2_tb3_simulation_launch.py
```


## Credits
This project is based on the awesome "Dolly" project: https://github.com/chapulina/dolly
     