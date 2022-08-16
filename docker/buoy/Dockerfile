FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# Necessary tools
RUN apt update \
 && apt install -y \
        apt-utils \
        wget

# Using non-official Gazebo + ROS combination, set it explicitly
ENV IGNITION_VERSION garden
ENV GZ_VERSION garden

# Garden only has nightlies for now
RUN /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list' \
 && /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -' \
 && apt-get update \
 && apt-get install -y gz-garden

# Create project directory and import packages
RUN mkdir -p /tmp/buoy_ws/src \
  && cd /tmp/buoy_ws/src/ \
  && wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml \
  && vcs import < buoy_all.yaml

# Install rosdep dependencies - this installs Gazebo and other packages
RUN sudo apt update \
  && cd /tmp/buoy_ws \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y -i \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean

# Build the project
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
  && cd /tmp/buoy_ws \
  && colcon build'

ENTRYPOINT ["/bin/bash" , "-c" , "source /tmp/buoy_ws/install/setup.bash && /bin/bash"]
