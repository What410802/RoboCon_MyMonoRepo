cmake_minimum_required(VERSION 3.16)

project(mini_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_core
    src/dji_motor.cpp
    src/dm_motor.cpp
    src/unitree_motor.cpp
    src/robot.cpp
)

target_include_directories(robot_core
    PUBLIC
    include
)

add_executable(robot_demo
    apps/main.cpp
)

target_link_libraries(robot_demo
    PRIVATE
    robot_core
)
