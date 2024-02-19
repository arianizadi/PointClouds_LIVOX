cmake_minimum_required(VERSION 3.0)

project(pointcloudtest)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_test main.cpp)

target_link_libraries(point_cloud_test
        livox_lidar_sdk_static
        ${PCL_LIBRARIES}
)
