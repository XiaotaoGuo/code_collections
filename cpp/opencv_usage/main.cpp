#include <chrono>
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat mat(1200, 1200, CV_8UC1);

    cv::randu(mat, cv::Scalar(-100), cv::Scalar(100));

    cv::Mat another_mat(1200, 1200, CV_8UC1);

    auto start = std::chrono::high_resolution_clock::now();

    // mat.copyTo(another_mat);

    for (int i = 0; i < 1200; ++i) {
        another_mat.ptr<uint8_t>(i)[(i + 1) % 1200] = i;
    }
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Time used: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " ms\n";
}