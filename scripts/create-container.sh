#!/bin/bash
# 명령 실패 시 즉시 종료
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/config-container.sh"

# 로컬에 정확한 버전(IMAGE_TAG)이 있는지 검사
if ! $DOCKER_CMD image inspect "$IMAGE_NAME:$IMAGE_TAG" >/dev/null 2>&1; then
    echo "--- 로컬에 이미지가 없습니다. 원격에서 시도합니다... ---"
    if ! $DOCKER_CMD pull $IMAGE_REMOTE; then
        echo "--- 원격 이미지도 없습니다. 직접 빌드합니다... ---"
        $SCRIPT_DIR/build-image.sh
    fi
else
    echo "--- 로컬 버전이 이미 존재합니다. ($IMAGE_NAME:$IMAGE_TAG) ---"
fi

# 기존 컨테이너 정리 (깔끔한 새 시작을 위해)
echo "--- 기존 컨테이너 '$CONTAINER_NAME' 정리 중... ---"
$DOCKER_CMD rm -f $CONTAINER_NAME 2>/dev/null || true

# =================================================================
# NVIDIA GPU를 깨우고 디바이스 노드 강제 생성
GPU_OPTS=""
if command -v nvidia-smi >/dev/null 2>&1; then
    echo "--- NVIDIA GPU 절전 모드 해제 및 장치 초기화 중... ---"
    # nvidia-smi를 실행하면 GPU가 깨어나고 디바이스 파일이 생성됨
    nvidia-smi >/dev/null 2>&1
    
    # 혹시라도 파일이 생성되지 않을 경우를 대비한 안전장치 (modprobe)
    command -v nvidia-modprobe >/dev/null 2>&1 && nvidia-modprobe -m -u 2>/dev/null || true
    
    # 장치 파일이 완전히 생성될 때까지 약간의 대기 시간 부여
    sleep 1

    GPU_OPTS="--gpus all"
fi
# =================================================================

# GUI 서버 접근 허용
if command -v xhost >/dev/null 2>&1; then
    xhost +local:$XHOST_TYPE >/dev/null 2>&1
fi

# 컨테이너 실행 (백그라운드 -d 모드)
echo "--- [$DOCKER_CMD] 컨테이너 실행 시작 ---"
$DOCKER_CMD run -dt \
    --name $CONTAINER_NAME \
    --privileged \
    --net=host \
    --shm-size=2gb \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=/run/user/1000 \
    -e DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/1000/bus" \
    -v /run/user/1000/bus:/run/user/1000/bus:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/run/user/1000/$WAYLAND_DISPLAY:rw \
    -v "$PROJECT_DIR:/home/$USER_NAME/colcon_ws$VOL_OPTS" \
    --device /dev/dri:/dev/dri \
    $GPU_OPTS \
    $EXTRA_OPTS \
    $IMAGE_NAME:$IMAGE_TAG

# 실행 확인 및 자동 접속
echo "--- 컨테이너 접속 중... ---"
if [ -f "$SCRIPT_DIR/enter-container.sh" ]; then
    exec "$SCRIPT_DIR/enter-container.sh"
else
    $DOCKER_CMD exec -it $CONTAINER_NAME /bin/bash
fi
