FROM balenalib/raspberrypi3-ubuntu:bionic-build

RUN apt-get clean && apt-get update && apt-get install -y locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

RUN apt update && sudo apt install -y curl gnupg2 lsb-release 

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && echo "deb [arch=armhf] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
  && apt update \
  && DEBIAN_FRONTEND=noninteractive apt install -y ros-dashing-ros-base \
  ros-dashing-slam-toolbox ros-dashing-navigation2 ros-dashing-nav2-bringup

RUN apt update && sudo apt install -y git python3-colcon-common-extensions

WORKDIR /ws
RUN mkdir src
RUN git clone https://github.com/Leonti/rplidar_ros.git src/rplidar_ros \
  && cd src/rplidar_ros \
  && git checkout dashing \
  && cd ../../

COPY rover src/rover

RUN /bin/bash -c "source /opt/ros/dashing/setup.bash; colcon build"

CMD /bin/bash -c "source /opt/ros/dashing/setup.bash; source ./install/setup.bash; ros2 launch rover rover.py"
