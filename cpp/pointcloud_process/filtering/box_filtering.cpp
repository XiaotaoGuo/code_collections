#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_visiualization_utils.hpp"
#include "pointcloud_filtering.hpp"

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            "/home/guo/personal/code_collections/cpp/pointcloud_process/sample_data/key_frame_20.pcd", *cloud_ptr) ==
        -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    // min/max x/y/z to keep points
    std::pair<float, float> keep_x_range{-30, 30};
    std::pair<float, float> keep_y_range{-15, 15};
    std::pair<float, float> keep_z_range{-2, 2};
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    filtering::filterPointsWithinRegion(cloud_ptr, temp_cloud_ptr, keep_x_range, keep_y_range, keep_z_range, false);

    // min/max x/y/z to remove points
    std::pair<float, float> remove_x_range{-5, 5};
    std::pair<float, float> remove_y_range{-2, 2};
    std::pair<float, float> remove_z_range{-2, 2};
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    filtering::filterPointsWithinRegion(
        temp_cloud_ptr, filtered_cloud_ptr, remove_x_range, remove_y_range, remove_z_range, true);

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_ptr->setBackgroundColor(0, 0, 0);
    viewer_ptr->addCoordinateSystem(1.0);

    int v1(0);
    viewer_ptr->createViewPort(0.0, 0.0, 0.33, 1.0, v1);
    viewer_ptr->setBackgroundColor(0, 0, 0, v1);
    visualization_utils::renderPoindCloud(viewer_ptr, cloud_ptr, "original");
    visualization_utils::renderBox(viewer_ptr,
                                   {(keep_x_range.second + keep_x_range.first) / (float)2.0,
                                    (keep_y_range.second + keep_y_range.first) / (float)2.0,
                                    (keep_z_range.second + keep_z_range.first) / (float)2.0},
                                   {keep_x_range.second - keep_x_range.first,
                                    keep_y_range.second - keep_y_range.first,
                                    keep_z_range.second - keep_z_range.first},
                                   {0.5, 0.8, 0.5},
                                   true,
                                   v1);

    int v2(1);
    viewer_ptr->createViewPort(0.33, 0.0, 0.66, 1.0, v2);
    viewer_ptr->setBackgroundColor(0, 0, 0, v2);
    visualization_utils::renderPoindCloud(viewer_ptr, temp_cloud_ptr, "temp", v2);
    visualization_utils::renderBox(viewer_ptr,
                                   {(remove_x_range.second + remove_x_range.first) / (float)2.0,
                                    (remove_y_range.second + remove_y_range.first) / (float)2.0,
                                    (remove_z_range.second + remove_z_range.first) / (float)2.0},
                                   {remove_x_range.second - remove_x_range.first,
                                    remove_y_range.second - remove_y_range.first,
                                    remove_z_range.second - remove_z_range.first},
                                   {0.8, 0.5, 0.5},
                                   true,
                                   v2);

    int v3(2);
    viewer_ptr->createViewPort(0.66, 0.0, 1.0, 1.0, v3);
    viewer_ptr->setBackgroundColor(0, 0, 0, v3);
    visualization_utils::renderPoindCloud(viewer_ptr, filtered_cloud_ptr, "filtered", v3);

    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}