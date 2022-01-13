#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>

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

}  // namespace model_fitting
