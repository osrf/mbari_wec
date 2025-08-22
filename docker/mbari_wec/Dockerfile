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
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
 && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
RUN dpkg -i /tmp/ros2-apt-source.deb

# ppa for libfshydrodynamics free surface hydrodynamics dependency until
# alternative in this PR: https://github.com/osrf/mbari_wec_gz/pull/138
RUN curl -s --compressed "https://hamilton8415.github.io/ppa/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ppa.gpg >/dev/null \
 && sudo curl -s --compressed -o /etc/apt/sources.list.d/my_list_file.list "https://hamilton8415.github.io/ppa/my_list_file.list"

# Install ROS 2 Jazzy and Gazebo Harmonic (from ros-gz)
RUN apt update \
 && apt install -y \
        python3-rosdep \
        python3-vcstool \
        python3-colcon-common-extensions \
        ros-jazzy-desktop \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-sdformat-urdf \
        ros-jazzy-plotjuggler \
        ros-jazzy-plotjuggler-msgs \
        ros-jazzy-plotjuggler-ros \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-sim-vendor \
        ros-jazzy-gz-math-vendor \
        libfshydrodynamics=1.4.0 \
 && apt clean

# For timing in tests, need to use cyclonedds for ROS 2 rather than default
# rmw provider
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
# Using non-official Gazebo + ROS combination, set it explicitly
ENV GZ_VERSION harmonic

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

RUN ls
# Create project directory and import packages
ENV MBARI_WEC_WS /home/$USERNAME/mbari_wec_ws
RUN mkdir -p ${MBARI_WEC_WS}/src \
 && cd ${MBARI_WEC_WS}/src/ \
 && wget https://raw.githubusercontent.com/osrf/mbari_wec/v2.0.0-rc1/mbari_wec_all.yaml \
 && cat mbari_wec_all.yaml

RUN cd ${MBARI_WEC_WS}/src \
 && vcs import < mbari_wec_all.yaml

# Workaround for gz sim and math python bindings
RUN python3 -m pip install -i https://mbari-org.github.io/gz-python-bindings/simple gz-python-bindings --break-system-packages

# Install rosdep dependencies
RUN sudo apt update \
 && cd ${MBARI_WEC_WS} \
 && sudo rosdep init \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y -i --rosdistro jazzy \
 && sudo rm -rf /var/lib/apt/lists/* \
 && sudo apt clean

# Build the project
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash \
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
 && echo "ros2 launch buoy_gazebo mbari_wec.launch.py pbloghome:=/logs rosbag2:=true" >> ${RUN_SH}

# Start the container at a bash prompt
ENTRYPOINT ["/bin/bash" , "-c", "source ${SETUP_SH} && /bin/bash"]
