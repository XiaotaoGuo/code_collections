#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_visiualization_utils.hpp"

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            "/home/guo/personal/code_collections/cpp/pointcloud_process/sample_data/key_frame_20.pcd", *cloud_ptr) ==
        -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);

    visualization_utils::renderPoindCloud(viewer, cloud_ptr, "original");

    std::pair<float, float> x_range{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};
    std::pair<float, float> y_range{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};
    std::pair<float, float> z_range{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};

    for (const auto& point : cloud_ptr->points) {
        x_range.first  = std::min((float)point.x, x_range.first);
        x_range.second = std::max((float)point.x, x_range.second);
        y_range.first  = std::min((float)point.y, y_range.first);
        y_range.second = std::max((float)point.y, y_range.second);
        z_range.first  = std::min((float)point.z, z_range.first);
        z_range.second = std::max((float)point.z, z_range.second);
    }

    std::cout << x_range.first << ", " << x_range.second << ", " << cloud_ptr->points.size() << std::endl;

    std::vector<float> center{(x_range.second + x_range.first) / (float)2.0,
                              (y_range.second + y_range.first) / (float)2.0,
                              (z_range.second + z_range.first) / (float)2.0};

    std::vector<float> size{
        (x_range.second - x_range.first), (y_range.second - y_range.first), (z_range.second - z_range.first)};

    visualization_utils::renderBox(viewer, center, size, {0.5, 0.8, 0.5}, true);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}