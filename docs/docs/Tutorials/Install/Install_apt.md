##### Requirements
At the moment, only source installation is supported. Use Ubuntu Focal.

1. Install [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html)

1. Install [Gazebo Fortress](https://ignitionrobotics.org/docs/fortress)

1. Install necessary tools

    ```
    sudo apt install python3-vcstool python3-colcon-common-extensions python3-pip git wget
    ```

##### Usage

1. Create a workspace, for example:

    ```
    mkdir -p ~/buoy_ws/src
    cd ~/buoy_ws/src
    ```

1. Clone all source repos with the help of `vcstool`:

    ```
    wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml
    vcs import < buoy_all.yaml
    cd ~/buoy_ws
    ```

1. Set the Gazebo version to Fortress. This is needed because we're not using an
   official ROS + Gazebo combination:

    ```
    export IGNITION_VERSION=fortress
    export GZ_VERSION=fortress
    ```

1. Install ROS dependencies

    ```
    sudo pip3 install -U rosdep
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y -i
    ```

1. Build and install

    ```
    source /opt/ros/galactic/setup.bash
    cd ~/buoy_ws
    colcon build
    ```
