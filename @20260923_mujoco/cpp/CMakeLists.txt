cmake_minimum_required(VERSION 3.22)
project(dog_sim LANGUAGES CXX)

# MuJoCo 由 pixi 环境提供：libmujoco.so + include/mujoco/*.h + lib/cmake/mujoco/mujocoConfig.cmake。
# 因此这里用 find_package，不要写 /usr 或 ~/.mujoco 之类的绝对路径（规范 §9）。
find_package(mujoco REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # 给 VS Code 的 C++ 智能提示用

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(dog_sim src/main.cpp)
target_link_libraries(dog_sim PRIVATE mujoco::mujoco)
target_compile_options(dog_sim PRIVATE -Wall -Wextra)
