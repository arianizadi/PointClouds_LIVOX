cmake_minimum_required(VERSION 3.0)

project(pointcloudtest)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_executable(point_cloud_test main.cpp)

target_link_libraries(point_cloud_test
        livox_lidar_sdk_static
	)
