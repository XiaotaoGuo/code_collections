#include <chrono>
#include <iostream>

#include <Eigen/CholmodSupport>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>
#include "utils/utils.hpp"

template <typename matrix_type, typename vector_type, typename F>
void evaluate(const matrix_type& H, const vector_type& b, F& solver, const std::string& label) {
    auto start = std::chrono::high_resolution_clock::now();

    solver.compute(H);
    solver.solve(b);

    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "--------- " << label << " ---------\n";
    std::cout << "Time used: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";
    std::cout << "--------- " << label << " ---------\n";
}

int main(int argc, char** argv) {
    Eigen::MatrixXd H = utils::load_csv<Eigen::MatrixXd>(
        "/home/guo/personal/code_collections/cpp/eigen_usage/matrix_decomposition/data/graph_optimization_example/"
        "input_INTEL_H.csv");

    Eigen::MatrixXd b = utils::load_csv<Eigen::MatrixXd>(
        "/home/guo/personal/code_collections/cpp/eigen_usage/matrix_decomposition/data/graph_optimization_example/"
        "input_INTEL_b.csv");

    Eigen::SparseMatrix<double> H_sparse = utils::convertDenseToSparse<double>(H);

    std::cout << "Size of H: " << H.rows() << " x " << H.cols() << std::endl;
    std::cout << "Size of b: " << b.rows() << " x " << b.cols() << std::endl;

    // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::AMDOrdering<int>> qr_solver;
    // evaluate(H_sparse, b, qr_solver, "Sparse QR");

    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> llt_solver;
    evaluate(H_sparse, b, llt_solver, "Sparse LLT");

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> CG_solver;
    evaluate(H_sparse, b, CG_solver, "Conjugate Gradient");

    Eigen::CholmodSimplicialLLT<Eigen::SparseMatrix<double>> cholmod_llt_solver;
    evaluate(H_sparse, b, cholmod_llt_solver, "Cholmod LLT");
}
