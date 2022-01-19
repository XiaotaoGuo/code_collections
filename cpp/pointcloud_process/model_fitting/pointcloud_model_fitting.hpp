#pragma once

#include <unordered_map>
#include <unordered_set>

#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace model_fitting {

template <class PointType>
void PCA(boost::shared_ptr<pcl::PointCloud<PointType>>& cloud_ptr,
         Eigen::Vector3f& center,
         Eigen::Vector3f& eigen_values,
         Eigen::Matrix3f& eigen_vectors) {
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f mean;

    int n = cloud_ptr->points.size();

    for (const auto& pt : cloud_ptr->points) {
        sum.x() += pt.x;
        sum.y() += pt.y;
        sum.z() += pt.z;
    }

    mean = sum / n;

    Eigen::Matrix3f covariances;
    for (const auto& pt : cloud_ptr->points) {
        Eigen::Vector3f pt_center{pt.x - mean.x(), pt.y - mean.y(), pt.z - mean.z()};
        covariances += pt_center * pt_center.transpose();
    }

    covariances /= n;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariances);

    center        = mean;
    eigen_values  = saes.eigenvalues();
    eigen_vectors = saes.eigenvectors();
}

template <class PointType>
bool RANSACWithPlane(boost::shared_ptr<pcl::PointCloud<PointType>>& cloud_ptr,
                     std::vector<int>& inliers,
                     Eigen::Vector4f& coefficients,
                     double threshold = 0.1) {
    static float success_prob = 0.95f;
    static int max_iterations = 100;

    int iteration_thres = max_iterations;

    auto cloud_map  = cloud_ptr->getMatrixXfMap();
    int points_size = cloud_ptr->points.size();

    inliers.clear();

    Eigen::Matrix<float, 1, Eigen::Dynamic> distances;
    Eigen::Matrix<int, 1, Eigen::Dynamic> indices;

    distances.resize(1, points_size);
    indices.resize(1, points_size);
    indices.array() = 0;

    int best_inlier_count = 3;

    int iteration      = 0;
    float log_prob     = std::log(1.0 - success_prob);
    float inverse_size = 1.0f / static_cast<float>(points_size);

    while (iteration < iteration_thres) {
        iteration++;

        // randomly choose 3 points from point cloud
        int idx1 = std::rand() % points_size;
        int idx2 = std::rand() % points_size;
        int idx3 = std::rand() % points_size;

        const Eigen::Vector3f& p0 = cloud_map.template block<3, 1>(0, idx1);
        const Eigen::Vector3f& p1 = cloud_map.template block<3, 1>(0, idx2);
        const Eigen::Vector3f& p2 = cloud_map.template block<3, 1>(0, idx3);

        // determine whether three points can form a plane
        if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) continue;

        Eigen::Vector3f p01   = p1 - p0;
        Eigen::Vector3f p02   = p2 - p0;
        Eigen::Vector3f scale = p01.array() / p02.array();

        // determine if two line are parallel
        if (scale(0) == scale(1) && scale(1) == scale(2)) continue;

        // fit a plane: ax + by + cz + d = 0, normal = (a, b, c)
        Eigen::Vector4f coeff;
        auto normal = coeff.template block<3, 1>(0, 0);

        normal = p01.cross(p02);
        normal.normalize();
        coeff(3) = -p0.transpose() * normal;

        distances = coeff.transpose() * cloud_map;

        int inliers_count = (distances.array().abs() < threshold).count();

        if (inliers_count > best_inlier_count) {
            indices           = (distances.array().abs() < threshold).template cast<int>();
            best_inlier_count = inliers_count;
            coefficients      = coeff;

            float w = static_cast<float>(inliers_count) * inverse_size;

            float prob_not_all_inliers =
                std::max(std::numeric_limits<float>::epsilon(),
                         std::min(1.0f - std::numeric_limits<float>::epsilon(), 1.0f - w * w * w));

            iteration_thres = log_prob / std::log(prob_not_all_inliers);
        }

        if (iteration > max_iterations) {
            return false;
        }
    }

    // updated inliers
    for (int i = 0; i < points_size; ++i) {
        if (indices(i) == 1) {
            inliers.push_back(i);
        }
    }

    return !inliers.empty();
}

}  // namespace model_fitting
