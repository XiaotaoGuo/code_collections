#include <chrono>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "utils/sciplot/sciplot.hpp"

#include <sciplot/sciplot.hpp>

template <typename matrix_type, typename vector_type, typename F>
vector_type evaluate(const matrix_type& A,
                     const vector_type& b,
                     const vector_type& gt,
                     F solve,
                     const std::string& label) {
    auto start = std::chrono::high_resolution_clock::now();

    vector_type coeff = solve(A, b);

    auto end = std::chrono::high_resolution_clock::now();

    vector_type diff = coeff - gt;

    std::cout << "--------- " << label << " ---------\n";
    std::cout << "Time used: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";
    std::cout << "Ground Truth: " << gt.transpose() << std::endl;
    std::cout << "Fitted: " << coeff.transpose() << std::endl;
    std::cout << "Loss: " << diff.norm() << std::endl;
    std::cout << "--------- " << label << " ---------\n";

    return coeff;
}

int main(int argc, char** argv) {
    const int order = 3;

    // y = c[0] + c[1]x + c[2]x^2 + ...
    Eigen::VectorXd gt_coeffcients(order + 1);
    gt_coeffcients << 20.0, -0.002, 0.005, -0.0008;

    int number_data = 5000;
    Eigen::VectorXd x_data(number_data);  // perfect  x
    Eigen::VectorXd y_data(number_data);  // perfect y
    Eigen::VectorXd b(number_data);
    x_data.array() = Eigen::ArrayXd::LinSpaced(number_data, -60.0, 60.0);
    Eigen::MatrixXd A(number_data, order + 1);

    for (size_t i = 0; i <= order; ++i) {
        A.col(i).array() = x_data.array().pow(i);
    }

    y_data    = A * gt_coeffcients;
    b.array() = y_data.array() * (1.0 + Eigen::ArrayXd::Random(number_data) / 10.0);

    /********  SVD **************/
    auto svd_solver = [](const Eigen::MatrixXd& A, const Eigen::VectorXd& b) -> Eigen::VectorXd {
        return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    };
    Eigen::VectorXd svd_coeff  = evaluate(A, b, gt_coeffcients, svd_solver, "SVD");
    Eigen::VectorXd svd_y_data = A * svd_coeff;

    /******** QR **********/
    auto qr_solver = [](const Eigen::MatrixXd& A, const Eigen::VectorXd& b) -> Eigen::VectorXd {
        return A.colPivHouseholderQr().solve(b);
    };
    Eigen::VectorXd qr_coeff  = evaluate(A, b, gt_coeffcients, qr_solver, "QR");
    Eigen::VectorXd qr_y_data = A * qr_coeff;

    /******** Normal Equation *******/
    auto normal_solver = [](const Eigen::MatrixXd& A, const Eigen::VectorXd& b) -> Eigen::VectorXd {
        return (A.transpose() * A).ldlt().solve(A.transpose() * b);
    };
    Eigen::VectorXd normal_coeff  = evaluate(A, b, gt_coeffcients, normal_solver, "Normal Equation");
    Eigen::VectorXd normal_y_data = A * normal_coeff;

    // Create a Plot object
    sciplot::Plot plot;
    plot.fontSize(12);
    plot.size(1080, 720);

    sciplot::Vec x_vec(x_data.data(), x_data.size());
    sciplot::Vec b_vec(b.data(), b.size());

    sciplot::Vec gt_y_vec(y_data.data(), y_data.size());
    sciplot::Vec svd_y_vec(svd_y_data.data(), svd_y_data.size());
    sciplot::Vec qr_y_vec(qr_y_data.data(), qr_y_data.size());
    sciplot::Vec normal_y_vec(normal_y_data.data(), normal_y_data.size());

    // Set the x and y labels
    plot.xlabel("x");
    plot.ylabel("y");

    // Set the x and y ranges
    // plot.xrange(0.0, PI);
    // plot.yrange(0.0, 1.0);

    // Set the legend to be on the bottom along the horizontal
    plot.legend().atTopRight().displayVertical().displayExpandWidthBy(2);
    plot.drawDots(x_vec, b_vec).label("Data");
    plot.drawCurve(x_vec, gt_y_vec).label("Ground Truth");
    plot.drawCurve(x_vec, svd_y_data).label("SVD");
    plot.drawCurve(x_vec, svd_y_data).label("QR");
    plot.drawCurve(x_vec, normal_y_data).label("Normal Equation");

    // Show the plot in a pop-up window
    // plot.show();

    // Save the plot to a PNG file
    plot.save("dense_linear_square.png");
}
