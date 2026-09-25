#!/usr/bin/env zsh

# POSIX shell (.sh) does not provide "the path of the script file"; bash & zsh do.
# bash: `SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"``
SCRIPT_DIR="${0:A:h}" # zsh: :A 解析符号链接成绝对路径，:h 取父目录 (https://yuanbao.tencent.com/chat/naQivTmsDa/0Qg35bYAqgK)
cd "${SCRIPT_DIR}/../mini_robot"
cmake -S . -B build
cmake --build build
# Run `./build/robot_demo` after compilation.