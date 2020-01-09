cmake_minimum_required(VERSION 3.0)

project(
    realsense-network-hardware-video-encoder
)

# build the libraries tree
add_subdirectory(network-hardware-video-encoder)

# this is our main target
add_executable(realsense-nhve-h264 rnhve_h264.cpp)
target_include_directories(realsense-nhve-h264 PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-h264 nhve realsense2)



