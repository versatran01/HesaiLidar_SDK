cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
project(SDKTool)

cmake_policy(SET CMP0053 NEW)
set(CMAKE_BUILD_TYPE ON)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_compile_options(-Wall)

set(DISENABLE_TEST_CC ON CACHE BOOL "not build test")

include_directories("${PROJECT_BINARY_DIR}")
add_subdirectory(../ Hesai_lidar_sdk)

add_executable(packet_loss_tool ./packet_loss_tool.cc)

target_link_libraries(packet_loss_tool hesai_sdk_lib)

find_package(PCL COMPONENTS common visualization io QUIET REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(pcl_tool ./pcl_tool.cc)

target_link_libraries(pcl_tool hesai_sdk_lib ${PCL_LIBRARIES})

find_package(LASlib)
if(${LASlib_FOUND})
    include_directories(${LASlib_INCLUDE_DIRS})
    link_directories(${LASlib_INCLUDE_DIRS})
    add_executable(las_tool ./las_tool.cc)

    target_link_libraries(las_tool hesai_sdk_lib LASlib ${PCL_LIBRARIES})
else(${LASlib_FOUND})
    message("-- LASlib not found")
endif(${LASlib_FOUND})
