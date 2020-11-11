cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)

project(
    realsense-network-hardware-video-encoder
)

find_package (Threads REQUIRED)

# build the libraries tree
add_subdirectory(network-hardware-video-encoder)
add_subdirectory(roboclaw)
add_subdirectory(vmu931)
add_subdirectory(imu-enhanced-odometry)

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
target_link_libraries(realsense-nhve-depth-color nhve realsense2)

add_executable(robot-rnhve robot_rnhve.cpp robot.cpp)
target_include_directories(robot-rnhve PRIVATE network-hardware-video-encoder network-hardware-video-encoder/minimal-latency-streaming-protocol roboclaw vmu931 imu-enhanced-odometry)
target_link_libraries(robot-rnhve nhve realsense2 roboclaw vmu931 ieo Threads::Threads)

