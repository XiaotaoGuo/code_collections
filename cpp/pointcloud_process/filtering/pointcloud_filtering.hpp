#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>

// #include <pcl/filters/cropbox.h>

namespace filtering {

/**
 * @brief filter points within a given range
 * @param src_cloud_ptr source pointcloud
 * @param dst_cloud_ptr destination pointcloud
 * @param x_range min/max x range
 * @param y_range min/max y range
 * @param z_range min/max z range
 * @param remove if true, remove points inside the region, otherwise only keep those points
 */
template <class PointType>
void filterPointsWithinRegion(boost::shared_ptr<pcl::PointCloud<PointType>>& src_cloud_ptr,
                              boost::shared_ptr<pcl::PointCloud<PointType>>& dst_cloud_ptr,
                              const std::pair<float, float>& x_range,
                              const std::pair<float, float>& y_range,
                              const std::pair<float, float>& z_range,
                              bool remove) {
    int num_points = src_cloud_ptr->points.size();
    boost::shared_ptr<pcl::PointCloud<PointType>> cloud_ptr(new pcl::PointCloud<PointType>());
    cloud_ptr->points.reserve(num_points);

    for (const auto& pt : src_cloud_ptr->points) {
        bool inside = (pt.x >= x_range.first && pt.x <= x_range.second && pt.y >= y_range.first &&
                       pt.y <= y_range.second && pt.z >= z_range.first && pt.z <= z_range.second);

        if (inside ^ remove) {
            cloud_ptr->points.push_back(pt);
        }
    }

    dst_cloud_ptr = cloud_ptr;
}

template <class PointType>
void voxelFiltering(boost::shared_ptr<pcl::PointCloud<PointType>>& src_cloud_ptr,
                    boost::shared_ptr<pcl::PointCloud<PointType>>& dst_cloud_ptr,
                    float resolution,
                    bool use_geometric_center = true) {
    // find pointcloud range
    float max_x = std::numeric_limits<float>::min(), min_x = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::min(), min_y = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min(), min_z = std::numeric_limits<float>::max();
    for (const auto& pt : src_cloud_ptr->points) {
        max_x = std::max(max_x, pt.x);
        min_x = std::min(min_x, pt.x);
        max_y = std::max(max_y, pt.y);
        min_y = std::min(min_y, pt.y);
        max_z = std::max(max_z, pt.z);
        min_z = std::min(min_z, pt.z);
    }

    // calculate pointcloud center
    float center_x = (max_x + min_x) / 2.0;
    float center_y = (max_y + min_y) / 2.0;
    float center_z = (max_z + min_z) / 2.0;

    // calculate needed grid nums in each axis
    int x_grid_nums = std::ceil((max_x - min_x) / resolution);
    int y_grid_nums = std::ceil((max_y - min_y) / resolution);
    int z_grid_nums = std::ceil((max_z - min_z) / resolution);

    // map: x_idx -> {y_idx -> {z_idx -> {sum_points, count}}}
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::pair<PointType, int>>>> status;

    // function for handling float grid idx
    auto process_idx = [](float idx, int grid_num) -> int {
        if (grid_num % 2 == 1) {
            idx += 0.5;
        }

        return std::floor(idx);
    };

    auto reverse_idx = [resolution](int idx, int grid_num, double center) -> float {
        float coord = center + idx * resolution;
        if (grid_num % 2 == 0) {
            coord += 0.5 * resolution;
        }

        return coord;
    };

    // for each point in pointcloud, find its grid index and accumulate points
    for (const auto& pt : src_cloud_ptr->points) {
        float x_idx = (pt.x - center_x) / resolution;
        float y_idx = (pt.y - center_y) / resolution;
        float z_idx = (pt.z - center_z) / resolution;

        int x_idx_clipped = process_idx(x_idx, x_grid_nums);
        int y_idx_clipped = process_idx(y_idx, y_grid_nums);
        int z_idx_clipped = process_idx(z_idx, z_grid_nums);

        if (status[x_idx_clipped][y_idx_clipped][z_idx_clipped].second == 0) {
            status[x_idx_clipped][y_idx_clipped][z_idx_clipped] = {pt, 1};
        } else {
            status[x_idx_clipped][y_idx_clipped][z_idx_clipped].first.x += pt.x;
            status[x_idx_clipped][y_idx_clipped][z_idx_clipped].first.y += pt.y;
            status[x_idx_clipped][y_idx_clipped][z_idx_clipped].first.z += pt.z;
            status[x_idx_clipped][y_idx_clipped][z_idx_clipped].second += 1;
        }
    }

    boost::shared_ptr<pcl::PointCloud<PointType>> cloud_ptr(new pcl::PointCloud<PointType>());
    cloud_ptr->points.reserve(x_grid_nums * y_grid_nums * z_grid_nums);
    for (const auto& x_yz : status) {
        int x_idx = x_yz.first;
        for (const auto& y_z : x_yz.second) {
            int y_idx = y_z.first;
            for (const auto& z_pt : y_z.second) {
                int z_idx = z_pt.first;
                PointType pt;
                if (use_geometric_center) {
                    pt.x = z_pt.second.first.x / z_pt.second.second;
                    pt.y = z_pt.second.first.y / z_pt.second.second;
                    pt.z = z_pt.second.first.z / z_pt.second.second;
                } else {
                    pt.x = reverse_idx(x_idx, x_grid_nums, center_x);
                    pt.y = reverse_idx(y_idx, y_grid_nums, center_y);
                    pt.z = reverse_idx(z_idx, z_grid_nums, center_z);
                }
                cloud_ptr->points.push_back(pt);
            }
        }
    }

    dst_cloud_ptr = cloud_ptr;
}

}  // namespace filtering
