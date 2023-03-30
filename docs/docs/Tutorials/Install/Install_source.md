## Install Requirements
Use Ubuntu 22.04.

1. Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
    Buoy Sim is tested against the cyclonedds rmw implementation, so set that up as follows:
        ```
        sudo apt install -y ros-humble-rmw-cyclonedds-cpp
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        ```

2. Install [Gazebo Garden](https://gazebosim.org/docs/garden)

3. Install necessary tools
        ```
        sudo apt install python3-vcstool python3-colcon-common-extensions python3-pip git wget
        ```

4. Install necessary libraries
        ```
        curl -s --compressed "https://hamilton8415.github.io/ppa/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ppa.gpg >/dev/null
        sudo curl -s --compressed -o /etc/apt/sources.list.d/my_list_file.list "https://hamilton8415.github.io/ppa/my_list_file.list"
        sudo apt update
        sudo apt install libfshydrodynamics=1.2.3
        ```


## Buoy Simulation Software Build

1. Create a workspace, for example:
        ```
        mkdir -p ~/mbari_wec_ws/src
        cd ~/mbari_wec_ws/src
        ```

2. Clone all source repos with the help of `vcstool`:
        ```
        wget https://raw.githubusercontent.com/osrf/mbari_wec/main/mbari_wec_all.yaml
        vcs import < mbari_wec_all.yaml
        cd ~/mbari_wec_ws
        ```

3. Set the Gazebo version to Garden. This is needed because we're not using an
   official ROS + Gazebo combination (place this in ~/.bashrc for convenience if rebuilding often):
        ```
        export GZ_VERSION=garden
        ```

4. Install ROS dependencies
        ```
        sudo pip3 install -U rosdep
        sudo rosdep init
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y -i
        ```

5. Build and install
        ```
        source /opt/ros/humble/setup.bash
        cd ~/mbari_wec_ws
        colcon build
        ```

   The simulation software should build without errors.  To run and test, proceed to the
   [Run the Simulator](../../../tutorials/#running-the-simulator) tutorial series.  Or run a quick
   test as described below to confirm all has worked as expected.

## Run an example to test

1. In a new terminal, source the workspace
        ```
        . ~/mbari_wec_ws/install/setup.sh`
        ```

1. Launch the simulation
        ```
        ros2 launch buoy_gazebo mbari_wec.launch.py`
        ```
