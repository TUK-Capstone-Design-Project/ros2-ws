#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/config-container.sh"

$SCRIPT_DIR/run-container.sh

# VS Code Dev Containers 설정 경로 지정
CONFIG_DIR="$HOME/.config/Code/User/globalStorage/ms-vscode-remote.remote-containers/nameConfigs"

# 디렉토리가 없으면 생성
mkdir -p "$CONFIG_DIR"

# 해당 컨테이너 이름으로 json 파일 생성
# - remoteUser: 호스트 사용자와 동일한 계정으로 접속
# - extensions: 컨테이너 접속 시 자동으로 설치할 확장 프로그램 목록 지정
cat << EOF > "$CONFIG_DIR/${CONTAINER_NAME}.json"
{
    "remoteUser": "${USER_NAME}",
    "extensions": [
        ${VSCODE_EXTENSIONS}
    ]
}
EOF
echo "--- VS Code 접속 설정 적용 완료: $CONFIG_DIR/${CONTAINER_NAME}.json"

# Hex 변환 및 원격 주소(Authority) 정의
HEX_NAME=$(printf '{"containerName":"%s"}' "$CONTAINER_NAME" | xxd -p | tr -d '\n')
REMOTE_AUTH="attached-container+${HEX_NAME}"

# 컨테이너 접속 및 프로젝트 디렉토리 열기
code --folder-uri "vscode-remote://${REMOTE_AUTH}/home/$USER_NAME/workspace"
