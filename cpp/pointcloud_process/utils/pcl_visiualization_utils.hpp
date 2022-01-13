#pragma once

#include <pcl/visualization/pcl_visualizer.h>

namespace visualization_utils {

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
               const std::vector<float>& center,
               const std::vector<float>& size,
               const std::vector<float>& color,
               bool fill     = false,
               int view_port = 0);

void renderLine(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
                const std::vector<float>& start_pt,
                const std::vector<float>& end_pt,
                const std::vector<float>& color,
                int view_port = 0);

void renderLine(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
                const std::vector<float>& start_pt,
                const std::vector<float>& direction,
                float scale,
                const std::vector<float>& color,
                int view_port = 0);

void renderCube(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
                const std::vector<float>& center,
                const std::vector<float>& size,
                const std::vector<float>& color,
                float opacity = 0.5,
                int view_port = 0);

template <class PointType>
void renderPoindCloud(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
                      boost::shared_ptr<pcl::PointCloud<PointType>>& cloud_ptr,
                      std::string id                     = "temp",
                      int view_port                      = 0,
                      const std::vector<uint8_t>& colors = {255, 255, 255}) {
    if (cloud_ptr == nullptr) return;
    pcl::visualization::PointCloudColorHandlerCustom<PointType> color(cloud_ptr, colors[0], colors[1], colors[2]);
    viewer_ptr->addPointCloud<PointType>(cloud_ptr, color, id, view_port);
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id, view_port);
}

void renderRGBPoindCloud(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
                         boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud_ptr,
                         std::string id = "temp",
                         int view_port  = 0);

template <class PointType>
void renderGrid(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
                boost::shared_ptr<pcl::PointCloud<PointType>>& cloud_ptr,
                float resolution,
                bool fill     = false,
                int view_port = 0) {
    float max_x = std::numeric_limits<float>::min(), min_x = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::min(), min_y = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min(), min_z = std::numeric_limits<float>::max();

    for (const auto& pt : cloud_ptr->points) {
        max_x = std::max(max_x, pt.x);
        min_x = std::min(min_x, pt.x);
        max_y = std::max(max_y, pt.y);
        min_y = std::min(min_y, pt.y);
        max_z = std::max(max_z, pt.z);
        min_z = std::min(min_z, pt.z);
    }

    float center_x = (max_x + min_x) / 2.0;
    float center_y = (max_y + min_y) / 2.0;
    float center_z = (max_z + min_z) / 2.0;

    int x_grid_nums = std::ceil((max_x - min_x) / resolution);
    int y_grid_nums = std::ceil((max_y - min_y) / resolution);
    int z_grid_nums = std::ceil((max_z - min_z) / resolution);

    float grid_min_x = center_x - resolution * (x_grid_nums / 2.0);
    float grid_min_y = center_y - resolution * (y_grid_nums / 2.0);
    float grid_min_z = center_z - resolution * (z_grid_nums / 2.0);
    float grid_max_x = center_x + resolution * (x_grid_nums / 2.0);
    float grid_max_y = center_y + resolution * (y_grid_nums / 2.0);
    float grid_max_z = center_z + resolution * (z_grid_nums / 2.0);

    for (float z = grid_min_z; z <= grid_max_z; z += resolution) {
        for (float x = grid_min_x; x <= grid_max_x; x += resolution) {
            renderLine(viewer_ptr, {x, grid_min_y, z}, {x, grid_max_y, z}, {0.5, 0.8, 0.5}, view_port);
        }
    }

    for (float z = grid_min_z; z <= grid_max_z; z += resolution) {
        for (float y = grid_min_y; y <= grid_max_y; y += resolution) {
            renderLine(viewer_ptr, {grid_min_x, y, z}, {grid_max_x, y, z}, {0.5, 0.8, 0.5}, view_port);
        }
    }

    for (float x = grid_min_x; x <= grid_max_x; x += resolution) {
        for (float y = grid_min_y; y <= grid_max_y; y += resolution) {
            renderLine(viewer_ptr, {x, y, grid_min_z}, {x, y, grid_max_z}, {0.5, 0.8, 0.5}, view_port);
        }
    }
}

/***
 * @brief get start index for each line from kitti pointcloud data
 */
template <typename PointType>
void sortPointCloudByScanLine(boost::shared_ptr<pcl::PointCloud<PointType>>& src_cloud_ptr,
                              std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& dst_cloud_ptr_vec) {
    const int N_SCAN  = 64;
    dst_cloud_ptr_vec = std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>(N_SCAN);
    PointType pt;
    int line_id;

    for (int i = 0; i < src_cloud_ptr->points.size(); ++i) {
        pt      = src_cloud_ptr->points[i];
        line_id = 0;

        double elevation_angle = std::atan2(pt.z, std::sqrt(pt.x * pt.x + pt.y * pt.y)) / M_PI * 180.0;

        // adapt from a-loam
        if (elevation_angle >= -8.83)
            line_id = int((2 - elevation_angle) * 3.0 + 0.5);
        else
            line_id = N_SCAN / 2 + int((-8.83 - elevation_angle) * 2.0 + 0.5);

        if (elevation_angle > 2 || elevation_angle < -24.33 || line_id > 63 || line_id < 0) {
            continue;
        }

        if (dst_cloud_ptr_vec[line_id] == nullptr)
            dst_cloud_ptr_vec[line_id] = boost::make_shared<pcl::PointCloud<PointType>>();
        dst_cloud_ptr_vec[line_id]->points.push_back(pt);
    }
}
}  // namespace visualization_utils
