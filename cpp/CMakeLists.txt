cmake_minimum_required(VERSION 3.0.0)
project(EigenUsage VERSION 0.1.0)

include(CTest)
enable_testing()
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(EigenUsage main.cpp)
target_link_libraries (EigenUsage Eigen3::Eigen)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
