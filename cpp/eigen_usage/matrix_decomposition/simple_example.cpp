#include <chrono>
#include <iostream>
#include <regex>

#include <boost/type_index.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

template <typename T, typename F>
void evaluate(const T& A, const T& b) {
    F dec(A);

    auto start = std::chrono::high_resolution_clock::now();

    T x = dec.solve(b);

    auto end = std::chrono::high_resolution_clock::now();

    T diff = A * x - b;

    std::string decomposition_type = boost::typeindex::type_id<F>().pretty_name();
    std::cmatch match_result;
    std::string label = "undefined.\n";
    if (std::regex_match(decomposition_type.c_str(), match_result, std::regex("Eigen::(.*)<Eigen.*"))) {
        label = match_result[1];
    }

    std::cout << "--------- " << label << " ---------\n";
    std::cout << "Time used: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";
    std::cout << "Loss: " << diff.norm() << std::endl;
    std::cout << "--------- " << label << " ---------\n";

    return;
}

int main() {
    Eigen::MatrixXf A = Eigen::Matrix<float, 1000, 1000>::Random();
    Eigen::MatrixXf b = Eigen::Matrix<float, 1000, 1>::Random();

    // LU
    evaluate<Eigen::MatrixXf, Eigen::FullPivLU<Eigen::MatrixXf>>(A, b);
    evaluate<Eigen::MatrixXf, Eigen::PartialPivLU<Eigen::MatrixXf>>(A, b);

    // QR
    evaluate<Eigen::MatrixXf, Eigen::HouseholderQR<Eigen::MatrixXf>>(A, b);
    evaluate<Eigen::MatrixXf, Eigen::FullPivHouseholderQR<Eigen::MatrixXf>>(A, b);
    evaluate<Eigen::MatrixXf, Eigen::ColPivHouseholderQR<Eigen::MatrixXf>>(A, b);

    return 0;
}