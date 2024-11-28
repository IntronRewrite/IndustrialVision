/*
 * @Author: IntronRewrite weijiehe@sdust.edu.com
 * @Date: 2024-11-26 03:50:58
 * @LastEditors: IntronRewrite weijiehe@sdust.edu.com
 * @LastEditTime: 2024-11-26 08:58:29
 * @FilePath: /plan6/4.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <chrono>
#include <thread>
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/FastGlobalRegistration.h>
#include <open3d/pipelines/registration/Feature.h>
#include <open3d/pipelines/registration/Registration.h>


int main(){
    // Load point clouds
    auto cloud_ref = std::make_shared<open3d::geometry::PointCloud>();
    auto cloud_src = std::make_shared<open3d::geometry::PointCloud>();

    if (open3d::io::ReadPointCloud("/home/lw/IndustrialVision/plan6/8-icp_result_ref.ply", *cloud_ref) &&
        open3d::io::ReadPointCloud("/home/lw/IndustrialVision/plan6/8-icp_result_src.ply", *cloud_src)) {
        std::cout << "Successfully read point clouds." << std::endl;
    } else {
        std::cerr << "Failed to read point clouds." << std::endl;
        return 1;
    }

    // Visualize point clouds
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Point Cloud Visualization", 1600, 900);
    visualizer.AddGeometry(cloud_ref);
    visualizer.AddGeometry(cloud_src);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();

    return 0;

}