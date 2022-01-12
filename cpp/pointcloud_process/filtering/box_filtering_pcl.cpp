#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include "pcl_visiualization_utils.hpp"

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    std::string data_path = "pointcloud_process/sample_data/key_frame_20.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(data_path, *cloud_ptr) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read test data \n");
        return (-1);
    }

    // min/max x/y/z to keep points
    std::pair<float, float> keep_x_range{-30, 30};
    std::pair<float, float> keep_y_range{-15, 15};
    std::pair<float, float> keep_z_range{-2, 2};
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // option 1: use CropBox
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setInputCloud(cloud_ptr);
    box_filter.setMin(Eigen::Vector4f(keep_x_range.first, keep_y_range.first, keep_z_range.first, 1.0));
    box_filter.setMax(Eigen::Vector4f(keep_x_range.second, keep_y_range.second, keep_z_range.second, 1.0));
    box_filter.filter(*temp_cloud_ptr);

    // // min/max x/y/z to remove points
    std::pair<float, float> remove_x_range{-5, 5};
    std::pair<float, float> remove_y_range{-2, 2};
    std::pair<float, float> remove_z_range{-2, 2};
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*temp_cloud_ptr, *filtered_cloud_ptr);

    // option 2: pass through filter
    // pcl::PassThrough<pcl::PointXYZ> pass_filter;
    // bool reverse_limits = true;
    // pass_filter.setInputCloud(filtered_cloud_ptr);
    // pass_filter.setFilterFieldName("x");
    // pass_filter.setFilterLimits(remove_x_range.first, remove_x_range.second);
    // pass_filter.getFilterLimitsNegative(reverse_limits);  // reverse the limits
    // pass_filter.filter(*filtered_cloud_ptr);
    // pass_filter.setFilterFieldName("y");
    // pass_filter.setFilterLimits(remove_y_range.first, remove_y_range.second);
    // pass_filter.getFilterLimitsNegative(reverse_limits);  // reverse the limits
    // pass_filter.filter(*filtered_cloud_ptr);
    // pass_filter.setFilterFieldName("z");
    // pass_filter.setFilterLimits(remove_z_range.first, remove_z_range.second);
    // pass_filter.getFilterLimitsNegative(reverse_limits);  // reverse the limits
    // pass_filter.filter(*filtered_cloud_ptr);

    // option 3: conditional remove filter
    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionOr<pcl::PointXYZ>());
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(
        new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GE, remove_x_range.second)));
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(
        new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LE, remove_x_range.first)));
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GE, remove_y_range.second)));
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LE, remove_y_range.first)));
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GE, remove_z_range.second)));
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LE, remove_z_range.first)));

    pcl::ConditionalRemoval<pcl::PointXYZ> conditional_filter;
    conditional_filter.setInputCloud(temp_cloud_ptr);
    conditional_filter.setCondition(range_condition);
    conditional_filter.filter(*filtered_cloud_ptr);

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