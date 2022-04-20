#!/usr/bin/env bash

#
# Copyright (C) 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
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

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   nvidia-docker
#   an X server
#   rocker
# Recommended:
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

# Display Help
Help()
{
   echo "Runs a docker container with the image created by build.bash."
   echo
   echo "Syntax: script [-s|t] image_name"
   echo "options:"
   echo "d     Use for development with host system volume mount."
   echo "s     Simulation purposes only."
   echo
}

if [ $# -lt 2 ]
then
    echo "Usage: $0 <options [-s|t]> <image_name>"
    Help
    exit 1
fi

while getopts ":ds" option; do
  case $option in
    d) # Build image for development
      ROCKER_ARGS="--devices /dev/dri/ --dev-helpers --nvidia --x11 --user --home --git ";;
    s) # Build image for Simulation
      echo "Building Simulation image"
      ROCKER_ARGS="--devices /dev/dri/ --dev-helpers --nvidia --x11 --user --git";;
  esac
done

IMG_NAME=${@:$OPTIND:1}

# Replace `:` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr ':' '_' <<< "$IMG_NAME")_runtime"
ROCKER_ARGS="${ROCKER_ARGS} --name $CONTAINER_NAME"
echo "Using image <$IMG_NAME> to start container <$CONTAINER_NAME>"

rocker ${ROCKER_ARGS} $IMG_NAME
