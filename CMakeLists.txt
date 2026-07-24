cmake_minimum_required(VERSION 3.10)
project(unitree_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
)

# unitree_sdk2 是一个独立的、非 catkin 的 CMake 包，装在 /opt/unitree_robotics
# （官方文档默认安装路径）。显式指定 unitree_sdk2_DIR，不依赖 CMAKE_PREFIX_PATH。
set(unitree_sdk2_DIR "/opt/unitree_robotics/lib/cmake/unitree_sdk2")
find_package(unitree_sdk2 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(unitree_bridge_node
        src/main.cpp
        src/CmdVelBridge.cpp
        include/unitree_bridge/CmdVelBridge.h
)

add_dependencies(unitree_bridge_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(unitree_bridge_node
        ${catkin_LIBRARIES}
        unitree_sdk2
)

install(TARGETS unitree_bridge_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL
)
