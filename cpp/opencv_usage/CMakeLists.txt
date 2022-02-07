cmake_minimum_required(VERSION 3.0.0)
project(OpenCVUsage VERSION 0.1.0)

find_package( OpenCV REQUIRED )
find_package(Boost)

include_directories(
                     ${OpenCV_INCLUDE_DIRS}
                     ${Boost_INCLUDE_DIRS} )


# Eigen utils
add_executable(opnecv_usage main.cpp)
target_link_libraries (opnecv_usage ${OpenCV_LIBS})