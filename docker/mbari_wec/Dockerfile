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

# ppa for libfshydrodynamics free surface hydrodynamics dependency until
# alternative in this PR: https://github.com/osrf/mbari_wec_gz/pull/138
RUN curl -s --compressed "https://hamilton8415.github.io/ppa/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ppa.gpg >/dev/null \
 && sudo curl -s --compressed -o /etc/apt/sources.list.d/my_list_file.list "https://hamilton8415.github.io/ppa/my_list_file.list"

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
        libfshydrodynamics=1.2.3 \
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
ENV MBARI_WEC_WS /home/$USERNAME/mbari_wec_ws
RUN mkdir -p ${MBARI_WEC_WS}/src \
 && cd ${MBARI_WEC_WS}/src/ \
 && wget https://raw.githubusercontent.com/osrf/mbari_wec/main/mbari_wec_all.yaml \
 && vcs import < mbari_wec_all.yaml

# Install rosdep dependencies
RUN sudo apt update \
 && cd ${MBARI_WEC_WS} \
 && sudo rosdep init \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y -i --rosdistro humble \
 && sudo rm -rf /var/lib/apt/lists/* \
 && sudo apt clean

# Build the project
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \
  && cd ${MBARI_WEC_WS} \
  && colcon build'

# Add quick access scripts
ENV SETUP_SH /home/$USERNAME/setup.bash
RUN touch ${SETUP_SH} \
 && chmod 755 ${SETUP_SH} \
 && echo ". ${MBARI_WEC_WS}/install/setup.bash" >> ${SETUP_SH} \
 && echo "export SDF_PATH=\$GZ_SIM_RESOURCE_PATH" >> ${SETUP_SH}
ENV RUN_SH /home/$USERNAME/run_simulation.bash
RUN touch ${RUN_SH} \
 && chmod 755 ${RUN_SH} \
 && echo ". ${SETUP_SH}" >> ${RUN_SH} \
 && echo "ros2 launch buoy_gazebo mbari_wec.launch.py" >> ${RUN_SH}

# Start the container at a bash prompt
ENTRYPOINT ["/bin/bash" , "-c", "source ${SETUP_SH} && /bin/bash"]
