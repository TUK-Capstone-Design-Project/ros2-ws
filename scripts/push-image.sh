#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/config-container.sh"

$DOCKER_CMD tag $IMAGE_NAME:$IMAGE_TAG $IMAGE_REMOTE
$DOCKER_CMD push $IMAGE_REMOTE
