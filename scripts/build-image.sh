#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/config-container.sh"

echo "--- [$DOCKER_CMD]를 사용하여 이미지를 빌드합니다: $IMAGE_NAME:$IMAGE_TAG ---"

$DOCKER_CMD rm -f $CONTAINER_NAME 2>/dev/null || true
# $DOCKER_CMD rmi -f $IMAGE_NAME:$IMAGE_TAG || true

# 빌드 수행
# --no-cache 옵션은 필요할 때만 추가하세요.
$DOCKER_CMD build \
    --pull=newer \
    --build-arg USER_NAME=$USER_NAME \
    --build-arg USER_UID=$USER_UID \
    --build-arg USER_GID=$USER_GID \
    -t "$IMAGE_NAME:$IMAGE_TAG" $PROJECT_DIR

# 결과 확인
if [ $? -eq 0 ]; then
    echo "--- 빌드 성공! ---"
else
    echo "--- 빌드 실패 ---"
    exit 1
fi
