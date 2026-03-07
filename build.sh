#!/bin/bash
set -e

colcon build

# 모든 패키지의 compile_commands.json을 ./build/compile_commands.json로 병합
find ./build -name "compile_commands.json" -exec cat {} + | jq -s 'add' > ./build/compile_commands.json
