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

RUN apt update && sudo apt install -y git
RUN git clone git@github.com:youtalk/rplidar_ros.git

RUN cd rplidar_ros \
  && git checkout dashing \
  && colcon build
