# Install using Docker

Docker images that include the neccessary software and dependencies have been created for convenience. 

## Requirements

1. Install Docker using [installation instructions](https://docs.docker.com/engine/install/ubuntu/).

1. Complete the [Linux Postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/) to allow you to manage Docker as a non-root user.

1. If you have an NVIDIA graphics card, it can help speed up rendering. Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

### Use Existing Image on DockerHub
MBARI maintains Docker images for the two most recent releases on their DockerHub:
  - `mbari/mbari_wec:latest`
  - `mbari/mbari_wec:previous`

1. Get `run.bash` script.

   ```
   git clone -b v1.1.0 https://github.com/osrf/mbari_wec.git
   cd ~/mbari_wec/docker/
   ```
   Or
   ```
   wget https://raw.githubusercontent.com/osrf/mbari_wec/v1.1.0/docker/run.bash
   chmod +x run.bash
   ```

3. Run the container

   If you have an NVIDIA graphics card
   ```
   ./run.bash mbari/mbari_wec:latest
   ```
   Otherwise
   ```
   ./run.bash mbari/mbari_wec:latest --no-nvidia
   ```

### Build from Dockerfile
An alternative to using the images from MBARI's DockerHub would be to build from a Dockerfile. This is convenient if you would like to make any changes.

1. Clone the mbari_wec repository to download the latest Dockerfile.

   ```
   git clone -b v1.1.0 https://github.com/osrf/mbari_wec.git
   cd ~/mbari_wec/docker/
   ```

2. Build the docker image

   If you have an NVIDIA graphics card
   ```
   ./build.bash nvidia_opengl_ubuntu22
   ./build.bash mbari_wec
   ```
   Otherwise
   ```
   ./build.bash mbari_wec --no-nvidia
   ```

3. Run the container

   If you have an NVIDIA graphics card
   ```
   ./run.bash mbari_wec
   ```
   Otherwise
   ```
   ./run.bash mbari_wec --no-nvidia
   ```

4. To have another window running the same docker container, run this command in a new terminal:

   ```
   ./join.bash mbari_wec
   ```

### Quick start

Quick start scripts are provided in the home directory:

This sources the compiled workspace:
```
. setup.bash
```

This sources the compiled workspace and launches the simulation:
```
./run_simulation.bash
```

## Run an example to test

1. In a new terminal (whether on host machine or in Docker container), source the workspace
        ```
        . ~/mbari_wec_ws/install/setup.bash
        ```

1. Launch the simulation
        ```
        ros2 launch buoy_gazebo mbari_wec.launch.py
        ```

The simulation software should now be available.  To run and test, proceed to the [Run the Simulator](../../../tutorials/#running-the-simulator) tutorial series. 

