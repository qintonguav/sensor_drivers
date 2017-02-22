cmake_minimum_required(VERSION 2.8.3)
project(bluefox2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure
    )

find_package(mvIMPACT REQUIRED)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
    cfg/on_off.cfg
    )

message(STATUS "aaaaaa: " ${mvIMPACT_INCLUDE_DIRS})
catkin_package(
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${mvIMPACT_INCLUDE_DIRS}
    )

add_executable(bluefox2 
    src/camera_node.cpp
    src/camera.cpp
    )

# make sure configure headers are built before any node using them
add_dependencies(bluefox2 ${PROJECT_NAME}_gencfg)

target_link_libraries(bluefox2
    ${catkin_LIBRARIES}
    ${mvIMPACT_LIBRARIES}
    )

