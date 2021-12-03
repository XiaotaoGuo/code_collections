#include <iostream>
#include <Eigen/Dense>

int main(int, char**) {
    Eigen::Matrix3d mat;
    mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << "mat: \n" << mat << std::endl;
    std::cout << "inversed mat: \n" << mat.inverse() << std::endl;
    std::cout << "reversed mat: \n" << mat.reverse() << std::endl;
}
