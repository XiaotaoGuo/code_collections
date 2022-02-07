#include <eigen3/Eigen/Core>
#include <iostream>
#include "utils/utils.hpp"

int main() {
    // {
    //     // small
    //     Eigen::MatrixXd H = utils::load_csv<Eigen::MatrixXd>(
    //         "/home/guo/personal/code_collections/cpp/eigen_usage/matrix_decomposition/data/graph_optimization_example/"
    //         "input_MITb_H.csv");

    //     cv::Mat mat_img = utils::visualize_matrix_structure<Eigen::MatrixXd>(H, 0.5);
    //     cv::imwrite("small_graph.png", mat_img);
    // }

    // {
    //     // median
    //     Eigen::MatrixXd H = utils::load_csv<Eigen::MatrixXd>(
    //         "/home/guo/personal/code_collections/cpp/eigen_usage/matrix_decomposition/data/graph_optimization_example/"
    //         "input_INTEL_H.csv");

    //     cv::Mat mat_img = utils::visualize_matrix_structure<Eigen::MatrixXd>(H, 0.3);
    //     cv::imwrite("median_graph.png", mat_img);
    // }

    // {
    //     // large
    //     Eigen::MatrixXd H = utils::load_csv<Eigen::MatrixXd>(
    //         "/home/guo/personal/code_collections/cpp/eigen_usage/matrix_decomposition/data/graph_optimization_example/"
    //         "input_M3500_H.csv");

    //     cv::Mat mat_img = utils::visualize_matrix_structure<Eigen::MatrixXd>(H, 0.1);
    //     cv::imwrite("large_graph.png", mat_img);
    // }

    {
        // bal - vo
        Eigen::MatrixXd H = utils::load_csv<Eigen::MatrixXd>(
            "/home/guo/personal/code_collections/cpp/eigen_usage/matrix_decomposition/data/graph_optimization_example/"
            "bal_H.csv");

        cv::Mat mat_img = utils::visualize_matrix_structure<Eigen::MatrixXd>(H, 0.05);
        cv::imwrite("bal_graph.png", mat_img);
    }

    // std::cout << H << std::endl;
}