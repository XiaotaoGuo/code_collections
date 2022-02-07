cmake_minimum_required(VERSION 3.0.0)
project(EigenUtils VERSION 0.1.0)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package( OpenCV REQUIRED )
find_package(Boost)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
                     utils
                     ${CHOLMOD_INCLUDES}
                     ${OpenCV_INCLUDE_DIRS}
                     ${Boost_INCLUDE_DIRS} )


# Eigen utils
add_executable(eigen_utils main.cpp)
target_link_libraries (eigen_utils Eigen3::Eigen)

# Matrix Decomposition
add_executable(matrix_decomposition matrix_decomposition/matrix_decomposition.cpp)
target_link_libraries (matrix_decomposition Eigen3::Eigen ${OpenCV_LIBS})

# Matrix Decomposition
add_executable(simple_example matrix_decomposition/simple_example.cpp)
target_link_libraries (simple_example Eigen3::Eigen ${Boost_LIBRARIES})

# Solve Dense Linear Square Equation
add_executable(dense_linear_least_square matrix_decomposition/dense_linear_least_square.cpp)
target_link_libraries (dense_linear_least_square Eigen3::Eigen ${Boost_LIBRARIES})

add_executable(sparse_linear_least_square matrix_decomposition/sparse_linear_least_square.cpp)
target_link_libraries (sparse_linear_least_square Eigen3::Eigen ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CHOLMOD_LIBRARIES})