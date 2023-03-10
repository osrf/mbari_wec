FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# Necessary tools
RUN apt update \
 && apt install -y \
        apt-utils \
        vim \
        wget

# Garden only has nightlies for now
RUN /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list' \
 && /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -' \
 && apt update \
 && apt install -y \
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
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y -i \
 && sudo rm -rf /var/lib/apt/lists/* \
 && sudo apt clean

# Build the project
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
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
