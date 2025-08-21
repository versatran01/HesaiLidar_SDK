find_package(fmt REQUIRED)
find_package(glog REQUIRED)
find_package(absl REQUIRED)

add_executable(play main.cpp)
target_link_libraries(play PUBLIC hesai_sdk_lib fmt::fmt glog::glog)
