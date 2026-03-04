#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. 사용할 도구(Docker 또는 Podman) 감지
if command -v docker >/dev/null 2>&1; then
    DOCKER_CMD="docker"
elif command -v podman >/dev/null 2>&1; then
    DOCKER_CMD="podman"
else
    echo "에러: 시스템에 docker 또는 podman이 설치되어 있지 않습니다."
    exit 1
fi

# 2. 호스트 정보 자동 감지
HOST_USER=$(whoami)
HOST_UID=$(id -u)
HOST_GID=$(id -g)


# [수정 부분] 만약 현재 사용자가 root(0)라면 이름을 변경하여 충돌 방지
if [ "$HOST_USER" = "root" ] || [ "$HOST_UID" = "0" ]; then
    echo "--- 호스트가 root 계정이므로 컨테이너 사용자명을 'dev'로 설정합니다. ---"
    HOST_USER="dev"
    HOST_UID=1000
    HOST_GID=1000
fi

# 2. 이미지 이름 설정
IMAGE_NAME="raspbot-ros2-humble"
TAG="latest"

echo "--- [$DOCKER_CMD]를 사용하여 이미지를 빌드합니다: $IMAGE_NAME:$TAG ---"

CONTAINER_NAME="raspbot-nav2-dev"

$DOCKER_CMD rm -f $CONTAINER_NAME 2>/dev/null || true
$DOCKER_CMD rmi raspbot-ros2-humble || true

# 3. 빌드 수행
# --no-cache 옵션은 필요할 때만 추가하세요.
$DOCKER_CMD build \
    --build-arg USERNAME=$HOST_USER \
    --build-arg USER_UID=$HOST_UID \
    --build-arg USER_GID=$HOST_GID \
    -t "$IMAGE_NAME:$TAG" $SCRIPT_DIR

# 4. 결과 확인
if [ $? -eq 0 ]; then
    echo "--- 빌드 성공! ---"
    $DOCKER_CMD images | grep "$IMAGE_NAME"
else
    echo "--- 빌드 실패 ---"
    exit 1
fi

