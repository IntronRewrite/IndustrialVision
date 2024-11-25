/*
 * @Author: IntronRewrite weijiehe@sdust.edu.com
 * @Date: 2024-11-26 01:58:36
 * @LastEditors: IntronRewrite weijiehe@sdust.edu.com
 * @LastEditTime: 2024-11-26 02:17:10
 * @FilePath: /plan6/3.cpp
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


double ComputeAverageDistance(const std::shared_ptr<open3d::geometry::PointCloud>& pcd) {
    double avg_dist = 0.0;
    open3d::geometry::KDTreeFlann kdtree(*pcd);
    for (size_t i = 0; i < pcd->points_.size(); ++i) {
        std::vector<int> indices(2);
        std::vector<double> distances(2);
        kdtree.SearchKNN(pcd->points_[i], 2, indices, distances);
        avg_dist += std::sqrt(distances[1]);
    }
    avg_dist /= pcd->points_.size();
    return avg_dist;
}

int main(){
    // 是否可视化
    int flag = 1;
    // 计算源点云变换矩阵
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    // 加载点云
    auto start_loading = std::chrono::high_resolution_clock::now();

    auto pcd_src = std::make_shared<open3d::geometry::PointCloud>();
    auto pcd_ref = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud("../src_model.ply", *pcd_src);
    open3d::io::ReadPointCloud("../ref_model.ply", *pcd_ref);
    std::cout << "读取src点云数据成功" << std::endl;
    std::cout << "src点云的点数量: " << pcd_src->points_.size() << std::endl;
    std::cout << "读取ref点云数据成功" << std::endl;
    std::cout << "ref点云的点数量: " << pcd_ref->points_.size() << std::endl;
    // 加载点云时间
    auto end_loading = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_loading = end_loading - start_loading;
    std::cout << "点云加载运行时间: " << elapsed_loading.count() << " 秒" << std::endl;

    

    Eigen::Matrix4d transformation_matrix;
    transformation_matrix << -0.131427, -0.99108, -0.022091, -840.68,
                              0.722204, -0.110989, 0.682717, -31.0836,
                              0.679079, -0.0737735, -0.730349, 3527.89,
                              0, 0, 0, 1;

    pcd_src->Transform(transformation_matrix);
    std::cout << "点云变换完成" << std::endl;

    if (flag) {
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow("PointCloud Visualization", 1600, 900);
        visualizer.AddGeometry(pcd_src);
        visualizer.AddGeometry(pcd_ref);
        visualizer.Run();
        visualizer.DestroyVisualizerWindow();
    }
}