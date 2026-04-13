#!/bin/bash

# 프로젝트 경로 (현재 config.sh가 있는 위치 기준)
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 이미지 및 컨테이너 이름 설정
IMAGE_NAME="ros2-sdk"
IMAGE_TAG="26.4.13"
CONTAINER_NAME="ros2-workspace"

# 원격 설정 (Docker Hub 또는 개인 레지스트리)
IMAGE_REPO="docker.io/hanch2396"
IMAGE_REMOTE="$IMAGE_REPO/$IMAGE_NAME:$IMAGE_TAG"

# 컨테이너 사용자 설정
USER_NAME="developer"
USER_UID=1000
USER_GID=1000

# 사용할 도구(Docker 또는 Podman) 자동 감지
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
