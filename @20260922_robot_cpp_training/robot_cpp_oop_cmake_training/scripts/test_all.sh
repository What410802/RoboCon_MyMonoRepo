#!/usr/bin/env bash

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

for example_name in \
    01_hello_cpp \
    02_basic_motor \
    03_encapsulation \
    04_constructor \
    05_abstract_motor \
    06_inheritance \
    07_polymorphism \
    08_robot_composition; do
    example_dir="$ROOT_DIR/examples/$example_name"
    (
        cd "$example_dir"
        g++ main.cpp \
            -std=c++17 \
            -Wall \
            -Wextra \
            -pedantic \
            -o demo
        ./demo
    )
    echo "[PASS] $example_name"
done

(
    cd "$ROOT_DIR/mini_robot"
    g++ \
        apps/main.cpp \
        src/dm_motor.cpp \
        src/unitree_motor.cpp \
        src/robot.cpp \
        -Iinclude \
        -std=c++17 \
        -Wall \
        -Wextra \
        -pedantic \
        -o robot_demo_manual
    ./robot_demo_manual
)
echo "[PASS] mini_robot manual g++"

(
    cd "$ROOT_DIR/mini_robot"
    cmake -S . -B build_test
    cmake --build build_test
    ./build_test/robot_demo
)
echo "[PASS] mini_robot CMake"
echo
echo "All examples passed."
