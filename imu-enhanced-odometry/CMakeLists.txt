cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)

project(
    imu-enhanced-odometry
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(ieo ieo.cpp)
target_link_libraries(ieo Eigen3::Eigen)

