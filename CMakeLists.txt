cmake_minimum_required(VERSION 3.0)

project(
    realsense-network-hardware-video-encoder
)

# build the libraries tree
add_subdirectory(network-hardware-video-encoder)

# those are our main targets
add_executable(realsense-nhve-h264 rnhve_h264.cpp)
target_include_directories(realsense-nhve-h264 PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-h264 nhve realsense2)

add_executable(realsense-nhve-hevc10 rnhve_hevc10.cpp)
target_include_directories(realsense-nhve-hevc10 PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-hevc10 nhve realsense2)


