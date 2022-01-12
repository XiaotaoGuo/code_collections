#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_visiualization_utils.hpp"

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    std::string data_path = "pointcloud_process/sample_data/key_frame_20.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(data_path, *cloud_ptr) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read test data \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    rgb_ptr->points.resize(cloud_ptr->points.size());

    // index-based colorization
    // pcl::PointXYZRGB pt;
    // int n = cloud_ptr->points.size();
    // for (int i = 0; i < n / 3; ++i) {
    //     pt.x               = cloud_ptr->points[i].x;
    //     pt.y               = cloud_ptr->points[i].y;
    //     pt.z               = cloud_ptr->points[i].z;
    //     pt.r               = i % 255;
    //     pt.g               = 0;
    //     pt.b               = 0;
    //     rgb_ptr->points[i] = pt;
    // }

    // for (int i = n / 3; i < n / 3 * 2; ++i) {
    //     pt.x               = cloud_ptr->points[i].x;
    //     pt.y               = cloud_ptr->points[i].y;
    //     pt.z               = cloud_ptr->points[i].z;
    //     pt.r               = 0;
    //     pt.g               = i % 255;
    //     pt.b               = 0;
    //     rgb_ptr->points[i] = pt;
    // }

    // for (int i = n / 3 * 2; i < n; ++i) {
    //     pt.x               = cloud_ptr->points[i].x;
    //     pt.y               = cloud_ptr->points[i].y;
    //     pt.z               = cloud_ptr->points[i].z;
    //     pt.r               = 0;
    //     pt.g               = 0;
    //     pt.b               = i % 255;
    //     rgb_ptr->points[i] = pt;
    // }

    pcl::PointXYZ first_pt = cloud_ptr->points[0];
    pcl::PointXYZ last_pt  = cloud_ptr->points.back();
    std::cout << "first point: " << std::atan2(first_pt.y, first_pt.x) / M_PI * 180.0 << std::endl;
    std::cout << "last point: " << std::atan2(last_pt.y, last_pt.x) / M_PI * 180.0 << std::endl;
    // line-based colorization
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sorted_cloud_ptr_vec;
    visualization_utils::sortPointCloudByScanLine(cloud_ptr, sorted_cloud_ptr_vec);

    std::vector<std::vector<uint8_t>> rgb_groups(64, {0, 0, 0});
    for (int i = 0; i < 64; ++i) {
        rgb_groups[i][0] = rand() % 255;
        rgb_groups[i][1] = rand() % 255;
        rgb_groups[i][2] = rand() % 255;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_ptr->setBackgroundColor(0.0, 0.0, 0.0);
    viewer_ptr->addCoordinateSystem(1.0);

    int v1(0);
    viewer_ptr->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer_ptr->setBackgroundColor(0.0, 0.0, 0.0, v1);
    viewer_ptr->addCoordinateSystem(1.0, "reference", v1);
    visualization_utils::renderPoindCloud(viewer_ptr, cloud_ptr, "original", v1, {255, 255, 255});

    int v2(1);
    viewer_ptr->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer_ptr->setBackgroundColor(0.0, 0.0, 0.0, v2);
    viewer_ptr->addCoordinateSystem(1.0, "reference", v2);
    for (int i = 0; i < 64; i++) {
        if (sorted_cloud_ptr_vec[i] == nullptr) continue;
        std::cout << i << ": " << sorted_cloud_ptr_vec[i]->points.size() << std::endl;
        visualization_utils::renderPoindCloud(
            viewer_ptr, sorted_cloud_ptr_vec[i], "scan_" + std::to_string(i), v2, rgb_groups[i]);
    }
    // visualization_utils::renderRGBPoindCloud(viewer_ptr, rgb_ptr, "colored", v2);

    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}