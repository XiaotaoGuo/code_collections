#pragma once

#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>

namespace noise_removal {

template <typename PointType>
void radius_outlier_removal(boost::shared_ptr<pcl::PointCloud<PointType>>& src_cloud_ptr,
                            boost::shared_ptr<pcl::PointCloud<PointType>>& dst_cloud_ptr,
                            double search_radius,
                            int min_neighbors,
                            boost::shared_ptr<std::vector<int>> removed_indices = nullptr) {
    boost::shared_ptr<pcl::KdTreeFLANN<PointType>> kdtree(new pcl::KdTreeFLANN<PointType>());
    kdtree->setInputCloud(src_cloud_ptr);

    boost::shared_ptr<pcl::PointCloud<PointType>> cloud_ptr(new pcl::PointCloud<PointType>());

    cloud_ptr->points.reserve(src_cloud_ptr->points.size());
    if (removed_indices) removed_indices->reserve(src_cloud_ptr->points.size());

    std::vector<int> neighbors(min_neighbors);
    std::vector<float> distances(min_neighbors);
    for (size_t i = 0; i < src_cloud_ptr->points.size(); ++i) {
        if (kdtree->nearestKSearch(i, min_neighbors, neighbors, distances) > 0 &&
            distances.back() * distances.back() < search_radius) {
            cloud_ptr->points.push_back(src_cloud_ptr->points[i]);
        } else {
            if (removed_indices) removed_indices->push_back(i);
        }
    }
}

template <typename PointType>
void filter_occuluded_points(boost::shared_ptr<pcl::PointCloud<PointType>>& src_cloud_ptr,
                             boost::shared_ptr<pcl::PointCloud<PointType>>& dst_cloud_ptr,
                             int neighbor_count,
                             float distance_threshold,
                             float horizontal_angle_diff_threshold,
                             boost::shared_ptr<std::vector<int>> removed_indices = nullptr) {
    int cloud_size     = src_cloud_ptr->points.size();
    distance_threshold = std::fabs(distance_threshold);

    boost::shared_ptr<pcl::PointCloud<PointType>> cloud_ptr(new pcl::PointCloud<PointType>());
    cloud_ptr->points.reserve(cloud_size);

    std::vector<int> status(cloud_size, 0);

    for (int i = neighbor_count; i < cloud_size - neighbor_count; ++i) {
        const PointType& pt1 = src_cloud_ptr->points[i];
        const PointType& pt2 = src_cloud_ptr->points[i + 1];

        double horizontal_angle_1 = std::atan2(pt1.y, pt1.x) / M_PI * 180.0;
        double horizontal_angle_2 = std::atan2(pt2.y, pt2.x) / M_PI * 180.0;

        if (std::fabs(horizontal_angle_1 - horizontal_angle_2) > horizontal_angle_diff_threshold) continue;

        float range1 = std::sqrt(pt1.x * pt1.x + pt1.y * pt1.y + pt1.z * pt1.z);
        float range2 = std::sqrt(pt2.x * pt2.x + pt2.y * pt2.y + pt2.z * pt2.z);

        if (range1 - range2 > distance_threshold)  // pt1 is occluded
        {
            for (int j = i; j >= i - neighbor_count; j--) {
                status[j] = 1;
            }
        } else if (range2 - range1 > distance_threshold) {  // pt2 is occluded
            for (int j = i + 1; j <= i + neighbor_count; j++) {
                status[j] = 1;
            }
        }
    }

    for (int i = 0; i < cloud_size; ++i) {
        if (status[i] == 0) {
            cloud_ptr->points.push_back(src_cloud_ptr->points[i]);
        } else if (removed_indices != nullptr) {
            removed_indices->push_back(i);
        }
    }

    dst_cloud_ptr = cloud_ptr;
}

template <typename PointType>
void filter_parallel_points(boost::shared_ptr<pcl::PointCloud<PointType>>& src_cloud_ptr,
                            boost::shared_ptr<pcl::PointCloud<PointType>>& dst_cloud_ptr,
                            float distance_ratio,
                            float horizontal_angle_diff_threshold,
                            boost::shared_ptr<std::vector<int>> removed_indices = nullptr) {
    int cloud_size = src_cloud_ptr->points.size();

    boost::shared_ptr<pcl::PointCloud<PointType>> cloud_ptr(new pcl::PointCloud<PointType>());
    cloud_ptr->points.reserve(cloud_size);

    std::vector<int> status(cloud_size, 0);

    for (int i = 1; i < cloud_size - 1; ++i) {
        const PointType& pt0 = src_cloud_ptr->points[i];
        const PointType& pt1 = src_cloud_ptr->points[i - 1];
        const PointType& pt2 = src_cloud_ptr->points[i + 1];

        double horizontal_angle_0 = std::atan2(pt0.y, pt0.x) / M_PI * 180.0;
        double horizontal_angle_1 = std::atan2(pt1.y, pt1.x) / M_PI * 180.0;
        double horizontal_angle_2 = std::atan2(pt2.y, pt2.x) / M_PI * 180.0;

        if (std::fabs(horizontal_angle_0 - horizontal_angle_1) > horizontal_angle_diff_threshold ||
            std::fabs(horizontal_angle_0 - horizontal_angle_2) > horizontal_angle_diff_threshold)
            continue;

        float range0 = std::sqrt(pt0.x * pt0.x + pt0.y * pt0.y + pt0.z * pt0.z);
        float range1 = std::sqrt(pt1.x * pt1.x + pt1.y * pt1.y + pt1.z * pt1.z);
        float range2 = std::sqrt(pt2.x * pt2.x + pt2.y * pt2.y + pt2.z * pt2.z);

        float diff1 = std::fabs(range0 - range1);
        float diff2 = std::fabs(range0 - range2);

        if (diff1 > distance_ratio * range0 && diff2 > distance_ratio * range0) {
            status[i] = 1;
        }
    }

    for (int i = 0; i < cloud_size; ++i) {
        if (status[i] == 0) {
            cloud_ptr->points.push_back(src_cloud_ptr->points[i]);
        } else if (removed_indices != nullptr) {
            removed_indices->push_back(i);
        }
    }

    dst_cloud_ptr = cloud_ptr;
}

}  // namespace noise_removal
