#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. 호스트 X 서버 접근 허용
xhost +local:podman

# 2. 컨테이너 및 이미지 이름 설정
CONTAINER_NAME="raspbot-nav2-dev"
IMAGE_NAME="raspbot-ros2-humble"

# 3. 기존 컨테이너 삭제
podman rm -f $CONTAINER_NAME 2>/dev/null

# 4. Podman 실행 (Docker와 거의 같지만 --userns 설정이 중요)
podman run -dt \
    --name $CONTAINER_NAME \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $SCRIPT_DIR:$HOME/colcon_ws:Z \
    --device /dev/dri:/dev/dri \
    --userns=keep-id \
    $IMAGE_NAME

$SCRIPT_DIR/enter-container.sh
