#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/config-container.sh"

$SCRIPT_DIR/run-container.sh

# Hex 변환 및 원격 주소(Authority) 정의
HEX_NAME=$(printf '{"containerName":"%s"}' "$CONTAINER_NAME" | xxd -p | tr -d '\n')
REMOTE_AUTH="attached-container+${HEX_NAME}"

# 컨테이너 접속 및 프로젝트 디렉토리 열기
code --folder-uri "vscode-remote://${REMOTE_AUTH}${WORKSPACE_DIR}"
