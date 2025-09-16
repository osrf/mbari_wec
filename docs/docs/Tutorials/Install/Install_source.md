# Install from source

## Requirements

### Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
!!! info "NOTE:"
    These are the current recommended requirements. Ubuntu 22.04, ROS 2 Humble, Gazebo Garden
    instructions are no longer maintained for `mbari_wec` as Gazebo Garden is EOL.

Follow instructions for [Installing Gazebo with ROS](https://gazebosim.org/docs/harmonic/ros_installation/).

1. Go to the following to [Install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) ROS 2 Jazzy, if you do not already have it installed on your machine.

    Following your ROS installation do the following:
   
    MBARI WEC works best with the cyclonedds rmw implementation, so set that up as follows:
   
    ```
    sudo apt install ros-jazzy-rmw-cyclonedds-cpp
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

    Running the sim with rosbag2 logging uses mcap storage. We also need sdformat-urdf.
    So, install those as well:

    ```
    sudo apt install ros-jazzy-rosbag2-storage-mcap ros-jazzy-sdformat-urdf
    ```

    To use plotjuggler to view ros2 data live plots (recommended in these tutorials), install like so:

    ```
    sudo apt install ros-jazzy-plotjuggler ros-jazzy-plotjuggler-msgs ros-jazzy-plotjuggler-ros
    ```

3. Install Gazebo Harmonic by installing `ros_gz` from the `ros-jazzy-*` apt repos.
   
    ```
    sudo apt update
    sudo apt install ros-jazzy-ros-gz
    ```

    When using Gazebo Harmonic and ROS 2 Jazzy together, Gazebo packages (`gz-*`) now come from
    ROS 2 Jazzy apt repos (`ros-jazzy-gz-*`). We also need to use the `sdformat-urdf` package.
    See [here](https://gazebosim.org/docs/harmonic/ros_installation/) for more info.

4. Set the Gazebo version to Harmonic. (place this in ~/.bashrc for convenience if rebuilding often):
   
    ```
    export GZ_VERSION=harmonic
    ```

5. gz-python-bindings

    Python bindings for `gz.math` and `gz.sim` are used in `mbari_wec` but are not distributed via apt or pip.
    This repository, [gz-python-bindings](https://github.com/mbari-org/gz-python-bindings) builds the python
    bindings and packages them up in a simple `PyPI` index url for an easy pip install.

    1. Ensure these packages are also installed
    
       ```
       sudo apt update
       sudo apt install ros-jazzy-gz-sim-vendor ros-jazzy-gz-math-vendor
       ```

    2. pip install

        NOTE: `--break-system-packages` might sound scary, but don't be alarmed -- nothing bad will happen in this case

        ```
        python3 -m pip install -i https://mbari-org.github.io/gz-python-bindings/simple gz-python-bindings --break-system-packages
        ```
        
        If you get a "No module named pip" error, run the following - and then re-run the previous command

        ```
       sudo apt-get install python3-pip
        ```
       
        

6. Install necessary tools
   
    ```
    sudo apt install python3-vcstool python3-colcon-common-extensions python3-pip git wget
    ```

7. Install necessary libraries
   
    ```
    curl -s --compressed "https://hamilton8415.github.io/ppa/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ppa.gpg >/dev/null
    sudo curl -s --compressed -o /etc/apt/sources.list.d/my_list_file.list "https://hamilton8415.github.io/ppa/my_list_file.list"
    sudo apt update
    sudo apt install libfshydrodynamics=1.4.0
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

3. Install ROS dependencies

    NOTE: do not be alarmed if you recieve "error: externally managed environment" following the first line here
   
    ```
    sudo pip3 install -U rosdep
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y -i
    ```

4. Build and install
    
    ```
    source /opt/ros/jazzy/setup.bash
    cd ~/mbari_wec_ws
    colcon build
    ```

   The simulation software should build without errors.  To run and test, proceed to the
   [Run the Simulator](../../tutorials.md#running-the-simulator) tutorial series.  Or run a quick
   test as described below to confirm all has worked as expected.

## Run an example to test

1. In a new terminal, source the workspace
   
    ```
    . ~/mbari_wec_ws/install/setup.bash
    ```

2. Set `SDF_PATH` to allow `robot_state_publisher` parse the robot description
   from the sdformat model (place this in ~/.bashrc for convenience if rebuilding often):

   ```
   export SDF_PATH=$GZ_SIM_RESOURCE_PATH
   ```

3. Launch the simulation
   
    ```
    ros2 launch buoy_gazebo mbari_wec.launch.py
    ```

## Post-Build Environment Setup
1. Place these lines in your ~/.bashrc for convenience to simplify running the sim in the future:
   
    ```
    source /opt/ros/jazzy/setup.bash
    source /path/to/mbari_wec_ws/install/setup.bash
    export GZ_VERSION=harmonic
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export SDF_PATH=$GZ_SIM_RESOURCE_PATH
    ```
(NOTE: Remember to replace `/path/to/mbari_wec_ws/install/setup.bash` with your path to your mbari wec workspace)
