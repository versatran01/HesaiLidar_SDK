cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
project(Template)

set(WITH_PTCS_USE OFF CACHE BOOL "Use Ptcs")

cmake_policy(SET CMP0053 NEW)
set(CMAKE_BUILD_TYPE ON)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_compile_options(-Wall)

add_subdirectory(libhesai)

add_library(hesai_sdk_lib driver/hesai_lidar_sdk.cc)
target_link_libraries(hesai_sdk_lib PUBLIC lidar_lib ${OTHER_LIB})
set_target_properties(
    hesai_sdk_lib
    PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14
)
target_include_directories(hesai_sdk_lib PUBLIC ${OTHER_INCLUDE_DIRS} driver ./)

if(DISENABLE_TEST_CC)
else()
    add_executable(sample test/test.cc)

    target_link_libraries(sample hesai_sdk_lib)

    add_executable(multi_test test/multi_test.cc)

    target_link_libraries(multi_test hesai_sdk_lib)
endif()
