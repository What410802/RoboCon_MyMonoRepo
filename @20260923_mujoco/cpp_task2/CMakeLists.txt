cmake_minimum_required(VERSION 3.22)
project(dog_sim LANGUAGES CXX)

# MuJoCo 由 pixi 环境提供：libmujoco.so + include/mujoco/*.h + lib/cmake/mujoco/mujocoConfig.cmake。
# 因此这里用 find_package，不要写 /usr 或 ~/.mujoco 之类的绝对路径（规范 §9）。
find_package(mujoco REQUIRED)
# 离屏录像要一个 GL 上下文（record.h 自己开隐藏窗口），所以还要 glfw
find_package(glfw3 REQUIRED)
# view 模式的物理线程要 -pthread
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)    # 给 VS Code 的 C++ 智能提示用

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 最小仿真：加载场景 → 零力矩跑 N 秒，三种模式（sim / record / view）——对标 python/ 侧
# view 要官方界面（Simulate + GlfwAdapter），由 mujoco::libmujoco_simulate 提供（自带 glfw）
add_executable(dog_sim src/main.cpp)
target_link_libraries(dog_sim PRIVATE mujoco::mujoco mujoco::libmujoco_simulate Threads::Threads)

# 零力矩静止判定：读 rest keyframe，判“是否静止趴住”——对标 scripts/agent_scripts/rest_check.py
add_executable(rest_check src/rest_check.cpp)
target_link_libraries(rest_check PRIVATE mujoco::mujoco)

foreach(target dog_sim rest_check)
    target_compile_options(${target} PRIVATE -Wall -Wextra)
endforeach()
