#!/usr/bin/env bash

#
# Copyright (C) 2023 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

ARG base
FROM ${base}

ENV DEBIAN_FRONTEND=noninteractive

# Necessary tools
RUN apt update \
 && apt install -y \
        apt-utils \
        build-essential \
        cmake \
        cppcheck \
        curl \
        doxygen \
        gdb \
        git \
        gnupg2 \
        locales \
        lsb-release \
        python3-pip \
        sudo \
        vim \
        wget \
 && apt clean

# Set Locale for ROS 2
RUN locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8

# Add ROS 2 apt repository
# Set up keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Set up sources.list
RUN /bin/sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
 && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# Set up Gazebo keys and install
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
 && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null' \
 && apt update \
 && apt install -y \
        python3-rosdep \
        python3-vcstool \
        python3-colcon-common-extensions \
        ros-humble-desktop \
        ros-humble-rmw-cyclonedds-cpp \
        gz-garden \
 && apt clean

# For timing in tests, need to use cyclonedds for ROS 2 rather than default
# rmw provider
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
# Using non-official Gazebo + ROS combination, set it explicitly
ENV GZ_VERSION garden

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# Create project directory and import packages
ENV BUOY_WS /home/$USERNAME/buoy_ws
RUN mkdir -p ${BUOY_WS}/src \
 && cd ${BUOY_WS}/src/ \
 && wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml \
 && vcs import < buoy_all.yaml

# Install rosdep dependencies
RUN sudo apt update \
 && cd ${BUOY_WS} \
 && sudo rosdep init \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y -i --rosdistro humble \
 && sudo rm -rf /var/lib/apt/lists/* \
 && sudo apt clean

# Build the project
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \
  && cd ${BUOY_WS} \
  && colcon build'

# Add quick access scripts
ENV SETUP_SH /home/$USERNAME/setup.bash
RUN touch ${SETUP_SH} \
 && chmod 755 ${SETUP_SH} \
 && echo ". ${BUOY_WS}/install/setup.bash" >> ${SETUP_SH}
ENV RUN_SH /home/$USERNAME/run_simulation.bash
RUN touch ${RUN_SH} \
 && chmod 755 ${RUN_SH} \
 && echo ". ${BUOY_WS}/install/setup.bash" >> ${RUN_SH} \
 && echo "ros2 launch buoy_gazebo mbari_wec.launch.py" >> ${RUN_SH}

# Start the container at a bash prompt
ENTRYPOINT ["/bin/bash" , "-c" , "source ${BUOY_WS}/install/setup.bash && /bin/bash"]
