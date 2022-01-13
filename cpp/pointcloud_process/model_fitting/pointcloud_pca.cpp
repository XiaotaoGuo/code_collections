#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>

#include "pcl_visiualization_utils.hpp"
#include "pointcloud_model_fitting.hpp"

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    std::string data_path = "pointcloud_process/sample_data/key_frame_20.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(data_path, *cloud_ptr) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read test data \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pole_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud_ptr);

    // crop plane
    crop_box.setMin(Eigen::Vector4f(10, -2, -10, 1.0));
    crop_box.setMax(Eigen::Vector4f(15, 2, 10, 1.0));
    crop_box.filter(*plane_ptr);

    // crop pole
    crop_box.setMin(Eigen::Vector4f(7, -5, -10, 1.0));
    crop_box.setMax(Eigen::Vector4f(8, -4.5, 10, 1.0));
    crop_box.filter(*pole_ptr);

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_ptr->setBackgroundColor(0, 0, 0);
    viewer_ptr->addCoordinateSystem(1.0);

    int v1(0);
    viewer_ptr->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    viewer_ptr->setBackgroundColor(0, 0, 0, v1);
    visualization_utils::renderPoindCloud(viewer_ptr, cloud_ptr, "original", v1);
    visualization_utils::renderPoindCloud(viewer_ptr, plane_ptr, "plane", v1, {255, 0, 0});
    visualization_utils::renderPoindCloud(viewer_ptr, pole_ptr, "pole", v1, {0, 255, 0});

    Eigen::Vector3f center;
    Eigen::Vector3f eigen_values;
    Eigen::Matrix3f eigen_vectors;

    model_fitting::PCA(plane_ptr, center, eigen_values, eigen_vectors);
    eigen_values /= eigen_values(2);
    for (int i = 0; i < 3; ++i) {
        float scale = std::max((float)0.1, std::min(eigen_values(i), (float)10.0));
        std::cout << scale << " ";
        visualization_utils::renderLine(viewer_ptr,
                                        {center.x(), center.y(), center.z()},
                                        {eigen_vectors.col(i).x(), eigen_vectors.col(i).y(), eigen_vectors.col(i).z()},
                                        scale,
                                        {255, 255, 255});
    }

    model_fitting::PCA(pole_ptr, center, eigen_values, eigen_vectors);
    eigen_values /= eigen_values(2);
    for (int i = 0; i < 3; ++i) {
        float scale = std::max((float)0.1, std::min(eigen_values(i), (float)10.0));
        std::cout << scale << " ";
        visualization_utils::renderLine(viewer_ptr,
                                        {center.x(), center.y(), center.z()},
                                        {eigen_vectors.col(i).x(), eigen_vectors.col(i).y(), eigen_vectors.col(i).z()},
                                        scale,
                                        {255, 255, 255});
    }

    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}