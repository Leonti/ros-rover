FROM balenalib/raspberrypi3-ubuntu:focal-build

RUN apt-get clean && apt-get update && apt-get install -y locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

RUN apt update && sudo apt install -y curl gnupg2 lsb-release 

RUN dpkg --print-architecture
RUN apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  && apt-add-repository http://packages.ros.org/ros2/ubuntu \
  && DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-ros-base
#  ros-eloquent-navigation2 ros-eloquent-nav2-bringup

RUN apt update && sudo apt install -y git python3-colcon-common-extensions python3-pip

WORKDIR /ws
RUN mkdir src
RUN git clone https://github.com/Leonti/rplidar_ros.git src/rplidar_ros \
  && cd src/rplidar_ros \
  && git checkout dashing \
  && cd ../../


COPY arduino-bridge/requirements.txt src/arduino-bridge/requirements.txt
RUN pip3 install Cython
RUN pip3 install -r src/arduino-bridge/requirements.txt

COPY rover src/rover
COPY arduino-bridge src/arduino-bridge
COPY 99-usb-serial.rules /etc/udev/rules.d/
ENV UDEV=1

RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash; colcon build"

CMD /bin/bash -c "source /opt/ros/eloquent/setup.bash; source ./install/setup.bash; ROS_DOMAIN_ID=45 ros2 launch rover rover.py"
