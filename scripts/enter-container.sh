#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/config-container.sh"

$SCRIPT_DIR/run-container.sh

echo "--- [$DOCKER_CMD] '$CONTAINER_NAME' 컨테이너에 접속합니다..."

# GUI 설정을 위한 xhost 권한 부여 (필요한 경우)
if command -v xhost >/dev/null 2>&1; then
    xhost +local:$XHOST_TYPE >/dev/null 2>&1
fi

# 컨테이너 내부로 진입
$DOCKER_CMD exec -it -u $USER_NAME "$CONTAINER_NAME" /bin/bash -l
