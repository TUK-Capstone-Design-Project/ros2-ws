#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/config-container.sh"

echo "--- 이미지 태깅 중... ($IMAGE_NAME:$IMAGE_TAG -> $IMAGE_REMOTE)"
$DOCKER_CMD tag $IMAGE_NAME:$IMAGE_TAG $IMAGE_REMOTE

echo "--- 원격 레지스트리로 이미지 업로드 중... ($IMAGE_REMOTE)"
$DOCKER_CMD push $IMAGE_REMOTE

echo "--- 이미지 업로드 완료!"
