cmake_minimum_required(VERSION 3.22)
project(slope LANGUAGES CXX)

# 与 cpp_task2 / cpp_stand 共用同一个 pixi 环境
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 复用现成的头：args.h / record.h 来自 cpp_task2，control.h（同一个 PD 控制律）来自 cpp_stand；
# .cpp 是 cpp_stand/src/stand.cpp 的副本 + 倾斜地面改动（不为共享而抽象，两边各自演进）。
add_executable(slope src/slope.cpp)
target_include_directories(slope PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_task2/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_stand/src)
target_link_libraries(slope PRIVATE mujoco::mujoco mujoco::libmujoco_simulate Threads::Threads)
target_compile_options(slope PRIVATE -Wall -Wextra)
