#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

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

    // // initialize a sample consensus model with plane
    // pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
    //     new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_ptr));

    // // use ransac to fit the model
    // pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    // ransac.setDistanceThreshold(.1);
    // ransac.computeModel();

    // // get inliers from fitting result
    // std::vector<int> inliers;
    // Eigen::VectorXf coefficients;
    // ransac.getInliers(inliers);
    // ransac.getModelCoefficients(coefficients);

    std::vector<int> inliers;
    Eigen::Vector4f coefficients;
    model_fitting::RANSACWithPlane(cloud_ptr, inliers, coefficients, 0.2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::copyPointCloud(*cloud_ptr, inliers, *plane_ptr);
    pcl::PointXYZ center;
    pcl::computeCentroid(*cloud_ptr, center);

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_ptr->setBackgroundColor(0, 0, 0);
    viewer_ptr->addCoordinateSystem(1.0);

    int v1(0);
    viewer_ptr->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    viewer_ptr->setBackgroundColor(0, 0, 0, v1);
    visualization_utils::renderPoindCloud(viewer_ptr, cloud_ptr, "original", v1);
    visualization_utils::renderPoindCloud(viewer_ptr, plane_ptr, "plane", v1, {255, 0, 0});
    visualization_utils::renderLine(viewer_ptr,
                                    {center.x, center.y, center.z},
                                    {coefficients(0), coefficients(1), coefficients(2)},
                                    10,
                                    {0, 0, 255});

    while (!viewer_ptr->wasStopped()) {
        viewer_ptr->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}