This is the entrypoint for the wave energy harvesting buoy project.

See [documentation here](https://osrf.github.io/mbari_wec/main/).

And MBARI-WEC in action using Gazebo simulator here:

![](docs/docs/images/buoy_sim.gif)


# Simulation Repositories

These are the repositories for the project:

* [mbari_wec_utils](https://github.com/osrf/mbari_wec_utils): ROS 2 messages, interface API, and examples for
  receiving and sending data to a physical or simulated buoy.
    * [buoy_interfaces](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp): ROS 2 messages
      to recieve and send data to a physical or simulated buoy
    * [buoy_api_cpp](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp): C++ Interface to
      MBARI Power Buoy including Controller examples to run against a physical or simulated buoy.
    * [buoy_api_py](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_py): Python Interface to
      MBARI Power Buoy including Controller examples to run against a physical or simulated buoy.
* [mbari_wec_gz](https://github.com/osrf/mbari_wec_gz)
    * [buoy_description](https://github.com/osrf/mbari_wec_gz/tree/main/buoy_description):
      Buoy model description.
    * [buoy_gazebo](https://github.com/osrf/mbari_wec_gz/tree/main/buoy_gazebo):
      Gazebo plugins, worlds and launch files to simulate the buoy.

# Interfaces and Examples

There are two GitHub
[template](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)
repositories set up (cpp/python) for a quick start on writing a
custom controller utilizing
[buoy_api_cpp](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp) and
[buoy_api_py](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_py). Please see
[cpp examples](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp/examples) and
[python examples](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_py/buoy_api/examples) for example
controller implementations.

* [mbari_wec_template_cpp](https://github.com/mbari-org/mbari_wec_template_cpp)
* [mbari_wec_template_py](https://github.com/mbari-org/mbari_wec_template_py)

# Install

At the moment, MBARI WEC is supported by source and Docker installation only.

### Source Install On Host System

[Tutorial: Install from source](https://osrf.github.io/mbari_wec/main/Tutorials/Install/Install_source/#install-from-source)

### Using Docker

[Tutorial: Install using Docker](https://osrf.github.io/mbari_wec/main/Tutorials/Install/Install_docker/#install-using-docker)

# Run the Simulator

[Tutorial: Running the Simulator](https://osrf.github.io/mbari_wec/main/Tutorials/Simulation/RunSimulator/#running-the-simulator)

# For maintainers only: To upload to DockerHub

Make sure you have permissions to push to the
[MBARI organization on DockerHub](https://hub.docker.com/u/mbari).
This permission is given by the MBARI administrator.

Build the `mbari_wec` Docker image, as detailed above.

Find the image ID for `mbari_wec`:
```
docker images
```

Tag the image with the destination name:
```
docker tag <IMAGE ID> mbari/mbari_wec:latest
```

Push to the [`mbari/mbari_wec` public image](https://hub.docker.com/r/mbari/mbari_wec).
```
docker push mbari/mbari_wec:latest
```

You may have to log in for it to recognize your permissions:
```
docker login
```

# Publications
```
Dizon, Chris, Ryan Coe, Andrew Hamilton, Dominic Forbush, Michael Anderson, Ted Brekken, and Giorgio
Bacelli. 2024. "Analysis on Evaluations of Monterey Bay Aquarium Research Institute’s Wave Energy
Converter’s Field Data Using WEC-Sim and Gazebo: A Simulation Tool Comparison" Applied Sciences 14,
no. 23: 11169. https://doi.org/10.3390/app142311169
```
