cmake_minimum_required(VERSION 3.22)
project(stand LANGUAGES CXX)

# MuJoCo 与 glfw 都由 pixi 环境提供（与 cpp_task2 同一个环境）
find_package(mujoco REQUIRED)
# record.h 里开隐藏窗口拿 GL 上下文、view 模式的官方界面，都要 glfw
find_package(glfw3 REQUIRED)
# view 模式的物理线程要 -pthread
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)    # 给 VS Code 的 C++ 智能提示用

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 复用 cpp_task2 里的两个公共头（args.h 解析命令行、record.h 离屏录像），不复制第二份
# _simulate 提供官方界面（Simulate + GlfwAdapter）以及 <simulate/...> 头（自带 glfw）
add_executable(stand src/stand.cpp)
target_include_directories(stand PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_task2/src)
target_link_libraries(stand PRIVATE mujoco::mujoco mujoco::libmujoco_simulate Threads::Threads)
target_compile_options(stand PRIVATE -Wall -Wextra)
