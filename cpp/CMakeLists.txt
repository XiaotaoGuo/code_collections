cmake_minimum_required(VERSION 3.10.0)
project(CodeCollection VERSION 0.1.0)

set (CMAKE_CXX_STANDARD 17)

include(cmake/FindCholmod.cmake)
add_subdirectory(eigen_usage)

add_subdirectory(opencv_usage)

# add_subdirectory(cpp_learnings)

# add_subdirectory(pointcloud_process)