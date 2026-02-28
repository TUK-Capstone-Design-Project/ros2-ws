#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# GUI 허용
xhost +local:docker

CONTAINER_NAME="raspbot-nav2-dev"
IMAGE_NAME="raspbot-ros2-humble"

# 이미 존재하면 삭제하고 새로 실행 (충돌 방지)
docker rm -f $CONTAINER_NAME 2>/dev/null

docker run -it \
    --name $CONTAINER_NAME \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $SCRIPT_DIR:/root/colcon_ws \
    $IMAGE_NAME
