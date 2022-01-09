#include "pcl_visiualization_utils.hpp"

namespace visualization_utils {

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
               const std::vector<float>& center,
               const std::vector<float>& size,
               const std::vector<float>& color,
               bool fill,
               int view_port) {
    float x_range = size[0];
    float y_range = size[1];
    float z_range = size[2];

    float center_x = center[0];
    float center_y = center[1];
    float center_z = center[2];

    float min_x = center_x - x_range / 2.0;
    float max_x = center_x + x_range / 2.0;
    float min_y = center_y - y_range / 2.0;
    float max_y = center_y + y_range / 2.0;
    float min_z = center_z - z_range / 2.0;
    float max_z = center_z + z_range / 2.0;

    // corners of bottom plane
    std::vector<float> pt1{min_x, min_y, min_z};
    std::vector<float> pt2{max_x, min_y, min_z};
    std::vector<float> pt3{max_x, max_y, min_z};
    std::vector<float> pt4{min_x, max_y, min_z};
    // corners of top plane
    std::vector<float> pt5{min_x, min_y, max_z};
    std::vector<float> pt6{max_x, min_y, max_z};
    std::vector<float> pt7{max_x, max_y, max_z};
    std::vector<float> pt8{min_x, max_y, max_z};

    // bottom plane
    renderLine(viewer_ptr, pt1, pt2, color);
    renderLine(viewer_ptr, pt2, pt3, color);
    renderLine(viewer_ptr, pt3, pt4, color);
    renderLine(viewer_ptr, pt4, pt1, color);

    // top plane
    renderLine(viewer_ptr, pt5, pt6, color);
    renderLine(viewer_ptr, pt6, pt7, color);
    renderLine(viewer_ptr, pt7, pt8, color);
    renderLine(viewer_ptr, pt8, pt5, color);

    // connect top and bottom plane
    renderLine(viewer_ptr, pt1, pt5, color);
    renderLine(viewer_ptr, pt2, pt6, color);
    renderLine(viewer_ptr, pt3, pt7, color);
    renderLine(viewer_ptr, pt4, pt8, color);

    if (fill) {
        renderCube(viewer_ptr, center, size, color, 0.3, view_port);
    }
}

void renderLine(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
                const std::vector<float>& start_pt,
                const std::vector<float>& end_pt,
                const std::vector<float>& color,
                int view_port) {
    static int id = 0;

    std::string line_id = "line" + std::to_string(id++);

    viewer_ptr->addLine(pcl::PointXYZ(start_pt[0], start_pt[1], start_pt[2]),
                        pcl::PointXYZ(end_pt[0], end_pt[1], end_pt[2]),
                        color[0],
                        color[1],
                        color[2],
                        line_id,
                        view_port);
}

void renderCube(pcl::visualization::PCLVisualizer::Ptr& viewer_ptr,
                const std::vector<float>& center,
                const std::vector<float>& size,
                const std::vector<float>& color,
                float opacity,
                int view_port) {
    static int id = 0;

    std::string box_id = "box" + std::to_string(id++);

    float x_range = size[0];
    float y_range = size[1];
    float z_range = size[2];

    float center_x = center[0];
    float center_y = center[1];
    float center_z = center[2];

    float min_x = center_x - x_range / 2;
    float max_x = center_x + x_range / 2;
    float min_y = center_y - y_range / 2;
    float max_y = center_y + y_range / 2;
    float min_z = center_z - z_range / 2;
    float max_z = center_z + z_range / 2;

    viewer_ptr->addCube(min_x, max_x, min_y, max_y, min_z, max_z, color[0], color[1], color[2], box_id, view_port);
    viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, box_id, view_port);
}
}  // namespace visualization_utils
