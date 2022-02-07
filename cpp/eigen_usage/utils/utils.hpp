#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "sciplot/sciplot.hpp"

namespace utils {

/**
 *  load a Eigen matrix from csv file
 *  ref: https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix/39146048
 */
template <typename M>
M load_csv(const std::string& path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Eigen::Map<
        const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(
        values.data(), rows, values.size() / rows);
}

template <typename element_type>
Eigen::SparseMatrix<element_type> convertDenseToSparse(Eigen::Matrix<element_type, Eigen::Dynamic, Eigen::Dynamic>& A) {
    int rows = A.rows();
    int cols = A.cols();

    Eigen::SparseMatrix<element_type> sparse(rows, cols);
    std::vector<Eigen::Triplet<element_type>> triplets;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (A(i, j) != 0) {
                triplets.push_back(Eigen::Triplet<element_type>(i, j, A(i, j)));
            }
        }
    }

    sparse.setFromTriplets(triplets.begin(), triplets.end());
    sparse.makeCompressed();

    return sparse;
}

template <typename M>
cv::Mat visualize_matrix_structure(const M& mat, double scale = 1.0) {
    int height    = mat.rows();
    int width     = mat.cols();
    cv::Mat graph = cv::Mat::zeros(height * scale, width * scale, CV_8UC1);

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            if (mat(row, col) != 0) {
                graph.at<uint8_t>(row * scale, col * scale) = 255;
            }
        }
    }

    cv::imshow("graph", graph);
    cv::waitKey(0);

    return graph;
}

}  // namespace utils
