#!/bin/bash

# 1. 사용할 도구(Docker 또는 Podman) 감지
if command -v docker >/dev/null 2>&1; then
    DOCKER_CMD="docker"
elif command -v podman >/dev/null 2>&1; then
    DOCKER_CMD="podman"
else
    echo "에러: 시스템에 docker 또는 podman이 설치되어 있지 않습니다."
    exit 1
fi

# 2. 접속할 컨테이너 이름 설정 (run 스크립트에서 설정한 이름과 일치해야 함)
CONTAINER_NAME="raspbot-nav2-dev"

# 3. 컨테이너가 실행 중인지 확인
IS_RUNNING=$($DOCKER_CMD ps -q -f name=$CONTAINER_NAME)

if [ -z "$IS_RUNNING" ]; then
    echo "에러: '$CONTAINER_NAME' 컨테이너가 실행 중이 아닙니다."
    echo "먼저 './run-podman.sh' (또는 run-docker.sh)를 실행하세요."
    exit 1
fi

echo "--- [$DOCKER_CMD] '$CONTAINER_NAME' 컨테이너에 접속합니다... ---"

xhost +local:$DOCKER_CMD

# 4. 컨테이너 내부로 진입
$DOCKER_CMD exec -it "$CONTAINER_NAME" /bin/bash
