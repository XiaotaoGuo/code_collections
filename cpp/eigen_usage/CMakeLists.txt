cmake_minimum_required(VERSION 3.0.0)
project(EigenUtils VERSION 0.1.0)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Eigen utils
add_executable(eigen_utils main.cpp)
target_link_libraries (eigen_utils Eigen3::Eigen)