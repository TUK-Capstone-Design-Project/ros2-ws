#!/bin/bash
# 명령 실패 시 즉시 종료
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. 사용할 도구(Docker 또는 Podman) 자동 감지
if command -v podman >/dev/null 2>&1; then
    DOCKER_CMD="podman"
    XHOST_TYPE="podman"
    # Podman 전용 옵션 (Rootless 권한 및 볼륨 레이블)
    EXTRA_OPTS="--userns=keep-id"
    VOL_OPTS=":Z"
elif command -v docker >/dev/null 2>&1; then
    DOCKER_CMD="docker"
    XHOST_TYPE="docker"
    EXTRA_OPTS=""
    VOL_OPTS=""
else
    echo "에러: 시스템에 docker 또는 podman이 설치되어 있지 않습니다."
    exit 1
fi

# 2. GUI 서버 접근 허용
xhost +local:$XHOST_TYPE

# 3. 설정 정의
CONTAINER_NAME="raspbot-nav2-dev"
IMAGE_NAME="raspbot-ros2-humble"

# 4. 기존 컨테이너 정리 (깔끔한 새 시작을 위해)
echo "--- 기존 컨테이너 '$CONTAINER_NAME' 정리 중... ---"
$DOCKER_CMD rm -f $CONTAINER_NAME 2>/dev/null || true

# 5. 컨테이너 실행 (백그라운드 -d 모드)
echo "--- [$DOCKER_CMD] 컨테이너 실행 시작 ---"
$DOCKER_CMD run -dt \
    --name $CONTAINER_NAME \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$SCRIPT_DIR:/home/$USER/colcon_ws$VOL_OPTS" \
    --device /dev/dri:/dev/dri \
    --gpus all \
    $EXTRA_OPTS \
    $IMAGE_NAME

# 6. 실행 확인 및 자동 접속
echo "--- 컨테이너 접속 중... ---"
if [ -f "$SCRIPT_DIR/enter-container.sh" ]; then
    exec "$SCRIPT_DIR/enter-container.sh"
else
    $DOCKER_CMD exec -it $CONTAINER_NAME /bin/bash
fi
