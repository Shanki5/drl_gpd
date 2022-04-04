#!/usr/bin/env bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
SRC_DIR=$(dirname $(dirname $(dirname $SCRIPT)))

DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="$SRC_DIR/drl_gpd":"/root/drl_grasping/fyp_ws/src/drl_gpd":rw \
"
DOCKER_ENV_VARS="
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
"
DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}

docker run -it --net=host --gpus all $DOCKER_ARGS $1 bash
