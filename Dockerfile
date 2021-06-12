FROM balenalib/generic-aarch64-ubuntu:focal-build-20200518

RUN dpkg --print-architecture
RUN apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  && apt-add-repository http://packages.ros.org/ros2/ubuntu \
  && DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-ros-base

RUN apt-get update && apt-get install -y locales \
  curl gnupg2 lsb-release software-properties-common \
  git python3-colcon-common-extensions python3-pip \
  && apt-get clean && rm -f /var/lib/apt/lists/*_* \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

#RUN apt update && sudo apt install -y curl gnupg2 lsb-release software-properties-common


#  ros-eloquent-navigation2 ros-eloquent-nav2-bringup

#RUN apt update && sudo apt install -y git python3-colcon-common-extensions python3-pip

WORKDIR /ws
RUN mkdir src
RUN git clone https://github.com/Leonti/rplidar_ros.git src/rplidar_ros \
  && cd src/rplidar_ros \
  && git checkout rpi_control \
  && cd ../../

RUN git clone https://github.com/Leonti/ros-bumper-interfaces.git src/bumper_interfaces \
  && cd src/bumper_interfaces \
  && cd ../../

COPY pico_bridge/requirements.txt src/pico_bridge/requirements.txt
RUN pip3 install Cython
RUN pip3 install -r src/pico_bridge/requirements.txt

COPY rpi-bumper/requirements.txt src/rpi-bumper/requirements.txt
RUN pip3 install -r src/rpi-bumper/requirements.txt

COPY rover src/rover
COPY pico_bridge src/pico_bridge
COPY hardware_control src/hardware_control
COPY rpi-bumper src/rpi-bumper
COPY 99-usb-serial.rules /etc/udev/rules.d/
ENV UDEV=1

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build"

CMD /bin/bash -c "source /opt/ros/foxy/setup.bash; source ./install/setup.bash; ROS_DOMAIN_ID=45 ros2 launch rover rover.py"
