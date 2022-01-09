cmake_minimum_required(VERSION 3.2.0)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create Project
project(PointCloudProcess)

# Find Packages
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(utils)
set(SRC_FILES "utils/pcl_visualization_utils.cpp")


add_executable(pointcloud_basic basic.cpp ${SRC_FILES})
target_link_libraries(pointcloud_basic ${PCL_LIBRARIES})

############## Filtering ################
add_executable(box_filtering filtering/box_filtering.cpp ${SRC_FILES})
target_link_libraries(box_filtering ${PCL_LIBRARIES})

add_executable(box_filtering_pcl filtering/box_filtering_pcl.cpp ${SRC_FILES})
target_link_libraries(box_filtering_pcl ${PCL_LIBRARIES})

add_executable(downsample_filtering filtering/downsample_filtering.cpp ${SRC_FILES})
target_link_libraries(downsample_filtering ${PCL_LIBRARIES})

add_executable(downsample_filtering_pcl filtering/downsample_filtering_pcl.cpp ${SRC_FILES})
target_link_libraries(downsample_filtering_pcl ${PCL_LIBRARIES})