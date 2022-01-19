#include <Eigen/Dense>
#include <iostream>

int main(int, char**) {
    // inverse vs reverse
    {
        std::cout << "inverse vs reverse.\n";
        Eigen::Matrix3d mat;
        mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
        std::cout << "mat: \n" << mat << std::endl;
        std::cout << "inversed mat: \n" << mat.inverse() << std::endl;
        std::cout << "reversed mat: \n" << mat.reverse() << std::endl;
    }

    {
        std::cout << "Condition based count.\n";
        Eigen::Vector4f vec;
        vec << 1.01, 0.99, 1.5, 0.9;
        std::cout << (vec.array() < 1).count() << std::endl;
    }
}
