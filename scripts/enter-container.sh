#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/config-container.sh"

# 컨테이너가 존재하는지 확인 (실행 중이 아니어도 생성되어 있는지 확인)
# -a: 모든 컨테이너, -q: ID만, -f: 필터링 (이름이 정확히 일치하도록 ^/이름$ 사용)
CONTAINER_EXISTS=$($DOCKER_CMD ps -a -q -f name="^/${CONTAINER_NAME}$")

if [ -z "$CONTAINER_EXISTS" ]; then
    echo "에러: '$CONTAINER_NAME' 컨테이너가 존재하지 않습니다."
    echo "먼저 './build-container.sh'를 실행하여 컨테이너를 생성하세요."
    exit 1
fi

# 컨테이너가 실행 중인지 확인
IS_RUNNING=$($DOCKER_CMD ps -q -f name="^/${CONTAINER_NAME}$")

if [ -z "$IS_RUNNING" ]; then
    echo "--- '$CONTAINER_NAME' 컨테이너가 중지되어 있습니다. 시작하는 중... ---"
    $DOCKER_CMD start "$CONTAINER_NAME"
    
    # 시작 실패 시 에러 처리
    if [ $? -ne 0 ]; then
        echo "에러: 컨테이너를 시작하지 못했습니다."
        exit 1
    fi
fi

echo "--- [$DOCKER_CMD] '$CONTAINER_NAME' 컨테이너에 접속합니다... ---"

# GUI 설정을 위한 xhost 권한 부여 (필요한 경우)
if command -v xhost >/dev/null 2>&1; then
    xhost +local:$XHOST_TYPE >/dev/null 2>&1
fi

# 컨테이너 내부로 진입
$DOCKER_CMD exec -it "$CONTAINER_NAME" /bin/bash
