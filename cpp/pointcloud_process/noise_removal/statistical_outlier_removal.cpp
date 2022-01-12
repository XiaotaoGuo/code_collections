#include <iostream>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_visiualization_utils.hpp"

#include "pointcloud_noise_removal.hpp"

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    std::string data_path = "pointcloud_process/sample_data/key_frame_20.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(data_path, *cloud_ptr) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read test data \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // PCL built-in radius removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>::Ptr statistical_outlier_removal(
        new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>(true));
    statistical_outlier_removal->setInputCloud(cloud_ptr);
    statistical_outlier_removal->setMeanK(20);
    statistical_outlier_removal->setStddevMulThresh(2.0);
    statistical_outlier_removal->filter(*filtered_cloud_ptr);

    auto removed_indices = statistical_outlier_removal->getRemovedIndices();

    for (int ind : *removed_indices) {
        removed_cloud_ptr->points.push_back(cloud_ptr->points[ind]);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_ptr->setBackgroundColor(0, 0, 0);
    viewer_ptr->addCoordinateSystem(1.0);

    int v1(0);
    viewer_ptr->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer_ptr->setBackgroundColor(0, 0, 0, v1);
    visualization_utils::renderPoindCloud(viewer_ptr, cloud_ptr, "original", v1, {255, 255, 255});

    int v2(1);
    viewer_ptr->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer_ptr->setBackgroundColor(0, 0, 0, v2);
    visualization_utils::renderPoindCloud(viewer_ptr, filtered_cloud_ptr, "filtered", v2, {255, 255, 255});
    visualization_utils::renderPoindCloud(viewer_ptr, removed_cloud_ptr, "removed", v2, {255, 0, 0});

    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}