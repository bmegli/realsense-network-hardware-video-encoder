cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)

project(
    realsense-network-hardware-video-encoder
)

# build the libraries tree
add_subdirectory(network-hardware-video-encoder)

# those are our main targets
add_executable(realsense-nhve-h264 rnhve_h264.cpp)
target_include_directories(realsense-nhve-h264 PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-h264 nhve realsense2)

add_executable(realsense-nhve-hevc rnhve_hevc.cpp)
target_include_directories(realsense-nhve-hevc PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-hevc nhve realsense2)

add_executable(realsense-nhve-depth-ir rnhve_depth_ir.cpp)
target_include_directories(realsense-nhve-depth-ir PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-depth-ir nhve realsense2)

add_executable(realsense-nhve-depth-color rnhve_depth_color.cpp)
target_include_directories(realsense-nhve-depth-color PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-depth-color nhve realsense2 realsense2-gl)

