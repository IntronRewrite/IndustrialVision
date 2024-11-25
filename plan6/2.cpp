/*
 * @Author: IntronRewrite weijiehe@sdust.edu.com
 * @Date: 2024-11-25 20:01:17
 * @LastEditors: IntronRewrite weijiehe@sdust.edu.com
 * @LastEditTime: 2024-11-26 03:20:53
 * @FilePath: /plan5/2.cpp
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
    int flag2 = 1;
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


    // 均匀采样
    auto start_sampling = std::chrono::high_resolution_clock::now();
    auto pcd_src_sampled = pcd_src->UniformDownSample(100);
    std::cout << "均匀采样src点云成功，采样后点数量: " << pcd_src_sampled->points_.size() << std::endl;
    // 下采样时间
    auto end_sampling = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_sampling = end_sampling - start_sampling;
    std::cout << "均匀下采样运行时间: " << elapsed_sampling.count() << " 秒" << std::endl;

    // 计算avg_dist
    auto start_compute = std::chrono::high_resolution_clock::now();
    double avg_dist = ComputeAverageDistance(pcd_src_sampled);
    auto end_compute = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_compute = end_compute - start_compute;
    std::cout << "点云的平均距离: " << avg_dist << std::endl;
    std::cout << "计算平均距离运行时间: " << elapsed_compute.count() << " 秒" << std::endl;



    // 聚类以移除离群点
    // 计算时间
    auto start_clustering = std::chrono::high_resolution_clock::now();

    std::vector<int> labels_sampled = pcd_src_sampled->ClusterDBSCAN(avg_dist * 3.7, 1, true);
    int max_label_sampled = *std::max_element(labels_sampled.begin(), labels_sampled.end());
    std::cout << "采样后的点云有 " << max_label_sampled + 1 << " 个聚类" << std::endl;

    // 为每个聚类分配固定颜色
    std::vector<Eigen::Vector3d> colors_sampled(pcd_src_sampled->points_.size(), Eigen::Vector3d(0, 0, 0));
    std::vector<Eigen::Vector3d> cluster_colors_sampled(max_label_sampled + 1);
    std::default_random_engine generator_sampled;
    std::uniform_real_distribution<double> distribution_sampled(0.0, 1.0);
    for (int i = 0; i <= max_label_sampled; ++i) {
        cluster_colors_sampled[i] = Eigen::Vector3d(distribution_sampled(generator_sampled), distribution_sampled(generator_sampled), distribution_sampled(generator_sampled));
    }
    for (size_t i = 0; i < labels_sampled.size(); ++i) {
        if (labels_sampled[i] >= 0) {
            colors_sampled[i] = cluster_colors_sampled[labels_sampled[i]];
        }
    }
    pcd_src_sampled->colors_ = colors_sampled;
    std::cout << "采样后的点云欧式聚类成功" << std::endl;

    auto end_clustering = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_clustering = end_clustering - start_clustering;
    std::cout << "聚类运行时间: " << elapsed_clustering.count() << " 秒" << std::endl;

    // 可视化采样后的聚类结果
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src_sampled}, "采样后的聚类结果", 1600,1600);
        std::cout << "采样后的聚类结果可视化成功" << std::endl;
    }
    if(flag2 == 1){
        open3d::io::WritePointCloud("1-sampled_clusters.ply", *pcd_src_sampled);
        std::cout << "采样后的聚类结果保存成功" << std::endl;
    }
    
    

    // 保留数量最多的一类
    auto start_cluster_selection = std::chrono::high_resolution_clock::now();
    // 找到数量最多的聚类
    std::vector<int> cluster_sizes_sampled(max_label_sampled + 1, 0);
    for (int label : labels_sampled) {
        if (label >= 0) {
            cluster_sizes_sampled[label]++;
        }
    }
    int largest_cluster_label_sampled = std::distance(cluster_sizes_sampled.begin(), std::max_element(cluster_sizes_sampled.begin(), cluster_sizes_sampled.end()));
    std::cout << "数量最多的聚类标签: " << largest_cluster_label_sampled << std::endl;
    // 找到数量最多的聚类的索引
    std::vector<int> largest_cluster_indices_sampled;
    for (size_t i = 0; i < labels_sampled.size(); ++i) {
        if (labels_sampled[i] == largest_cluster_label_sampled) {
            largest_cluster_indices_sampled.push_back(i);
        }
    }
    // 保留数量最多的聚类
    std::vector<size_t> largest_cluster_indices_sampled_size_t(largest_cluster_indices_sampled.begin(), largest_cluster_indices_sampled.end());
    auto largest_cluster_pcd_sampled = pcd_src_sampled->SelectByIndex(largest_cluster_indices_sampled_size_t);
    std::cout << "保留数量最多的聚类成功" << std::endl;

    // 可视化数量最多的聚类
    if (flag == 1) {
        open3d::visualization::DrawGeometries({largest_cluster_pcd_sampled}, "数量最多的聚类", 1600,1600);
        std::cout << "数量最多的聚类可视化成功" << std::endl;
    }

    

    // 选出pcd_src与该类相邻1.5*avg_dist的点
    open3d::geometry::KDTreeFlann kdtree_src(*pcd_src);
    std::vector<size_t> neighbor_indices;
    std::cout<<avg_dist<<std::endl;
    for (const auto& point : largest_cluster_pcd_sampled->points_) {
        std::vector<int> indices;
        std::vector<double> distances;
        kdtree_src.SearchRadius(point,  3*avg_dist, indices, distances);
        neighbor_indices.insert(neighbor_indices.end(), indices.begin(), indices.end());
    }
    // 选出相邻点
    auto neighbor_pcd = pcd_src->SelectByIndex(neighbor_indices);
    pcd_src->points_ = neighbor_pcd->points_;
    pcd_src->colors_ = neighbor_pcd->colors_;
    std::cout << "选出相邻点成功" << std::endl;

    // 计算相邻点时间
    auto end_cluster_selection = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_cluster_selection = end_cluster_selection - start_cluster_selection;
    std::cout << "聚类选择运行时间: " << elapsed_cluster_selection.count() << " 秒" << std::endl;

    // 将pcd_src和pcd_ref分别用不同颜色显示
    pcd_src->PaintUniformColor(Eigen::Vector3d(244.0/255, 171.0/255, 154.0/255)); // 绿色
    pcd_ref->PaintUniformColor(Eigen::Vector3d(182.0/255, 194.0/255, 174.0/255)); // 红色



    // 可视化相邻点
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src}, "相邻点结果", 1600,1600);
    }
    // 保存相邻点结果
    if (flag2 == 1) {
        open3d::io::WritePointCloud("2-neighbor_points.ply", *pcd_src);
        std::cout << "相邻点结果保存成功" << std::endl;
    }


    // 预处理
    // 计算预处理时间
    auto start = std::chrono::high_resolution_clock::now();

    auto src_oriented_bounding_box = pcd_src->GetOrientedBoundingBox();
    auto ref_oriented_bounding_box = pcd_ref->GetOrientedBoundingBox();
    src_oriented_bounding_box.color_ = Eigen::Vector3d(0, 1, 0);
    ref_oriented_bounding_box.color_ = Eigen::Vector3d(1, 0, 0);
    std::cout << "获取src定向边界框成功" << std::endl;
    std::cout << "获取ref定向边界框成功" << std::endl;

    // 可视化定向边界框
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "定向边界框可视化", 1600,1600);
        std::cout << "定向边界框可视化成功" << std::endl;
    }
    // 保存定向边界框可视化结果
    if (flag2 == 1) {
        open3d::io::WritePointCloud("3-oriented_bounding_box_src.ply", *pcd_src);
        open3d::io::WritePointCloud("3-oriented_bounding_box_ref.ply", *pcd_ref);
        std::cout << "定向边界框结果保存成功" << std::endl;
    }

    // 将点云平移到原点
    Eigen::Vector3d src_center = src_oriented_bounding_box.GetCenter();
    Eigen::Vector3d ref_center = ref_oriented_bounding_box.GetCenter();
    transformation.block<3, 1>(0, 3) = -src_center;
    std::cout << "变换矩阵: \n" << transformation << std::endl;
    pcd_src->Transform(transformation);
    src_oriented_bounding_box.Translate(-src_center);
    pcd_ref->Translate(-ref_center);
    ref_oriented_bounding_box.Translate(-ref_center);
    std::cout << "点云和定向边界框已移动到原点" << std::endl;



    // 可视化平移后的点云和定向边界框
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "平移后的点云和定向边界框可视化", 1600,1600);
        std::cout << "平移后的点云和定向边界框可视化成功" << std::endl;
    }
    // 保存平移后的点云和定向边界框可视化结果
    if (flag2 == 1) {
        open3d::io::WritePointCloud("4-translated_src.ply", *pcd_src);
        open3d::io::WritePointCloud("4-translated_ref.ply", *pcd_ref);
        std::cout << "平移后的点云和定向边界框结果保存成功" << std::endl;
    }



    // 旋转src以对齐ref
    Eigen::Matrix3d src_rotation = src_oriented_bounding_box.R_;
    Eigen::Matrix3d ref_rotation = ref_oriented_bounding_box.R_;
    Eigen::Matrix3d rotation = ref_rotation * src_rotation.inverse();
    transformation.block<3, 3>(0, 0) = rotation;
    std::cout << "变换矩阵: \n" << transformation << std::endl;
    pcd_src->Rotate(rotation, Eigen::Vector3d(0, 0, 0));
    src_oriented_bounding_box.Rotate(rotation, Eigen::Vector3d(0, 0, 0));
    std::cout << "src点云和定向边界框已旋转对齐ref" << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "粗配准运行时间: " << elapsed.count() << " 秒" << std::endl;


    

    // 可视化旋转对齐后的点云和定向边界框
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "旋转对齐后的点云和定向边界框可视化", 1600,1600);
    }
    // 保存旋转对齐后的点云和定向边界框可视化结果
    if (flag2 == 1) {
        open3d::io::WritePointCloud("5-rotated_src.ply", *pcd_src);
        open3d::io::WritePointCloud("5-rotated_ref.ply", *pcd_ref);
        std::cout << "旋转对齐后的点云和定向边界框结果保存成功" << std::endl;
    }


    // 计算边界框的长轴
    Eigen::Vector3d src_extent = src_oriented_bounding_box.extent_;
    Eigen::Vector3d ref_extent = ref_oriented_bounding_box.extent_;
    Eigen::Vector3d src_long_axis, ref_long_axis;
    if (src_extent.x() > src_extent.y() && src_extent.x() > src_extent.z()) {
        src_long_axis = src_oriented_bounding_box.R_.col(0);
    } else if (src_extent.y() > src_extent.x() && src_extent.y() > src_extent.z()) {
        src_long_axis = src_oriented_bounding_box.R_.col(1);
    } else {
        src_long_axis = src_oriented_bounding_box.R_.col(2);
    }


    // 计算旋转矩阵，使src长轴旋转180度
    Eigen::Matrix3d rotation_180 = Eigen::AngleAxisd(M_PI, src_long_axis.normalized()).toRotationMatrix();
    transformation.block<3, 3>(0, 0) = rotation_180 * transformation.block<3, 3>(0, 0);
    std::cout << "变换矩阵: \n" << transformation << std::endl;
    pcd_src->Rotate(rotation_180, Eigen::Vector3d(0, 0, 0));
    src_oriented_bounding_box.Rotate(rotation_180, Eigen::Vector3d(0, 0, 0));
    std::cout << "src点云已旋转180度" << std::endl;

    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "旋转对齐后的点云和定向边界框可视化", 1600,1600);
        std::cout << "沿长轴旋转180度后的点云和定向边界框可视化成功" << std::endl;
    }
    // 保存沿长轴旋转180度后的点云和定向边界框可视化结果
    if (flag2 == 1) {
        open3d::io::WritePointCloud("6-rotated_180_long_axis_src.ply", *pcd_src);
        open3d::io::WritePointCloud("6-rotated_180_long_axis_ref.ply", *pcd_ref);
        std::cout << "沿长轴旋转180度后的点云和定向边界框结果保存成功" << std::endl;
    }


    // 计算边界框的短轴
    Eigen::Vector3d src_short_axis, ref_short_axis;
    if (src_extent.x() < src_extent.y() && src_extent.x() < src_extent.z()) {
        src_short_axis = src_oriented_bounding_box.R_.col(0);
    } else if (src_extent.y() < src_extent.x() && src_extent.y() < src_extent.z()) {
        src_short_axis = src_oriented_bounding_box.R_.col(1);
    } else {
        src_short_axis = src_oriented_bounding_box.R_.col(2);
    }


    // 计算旋转矩阵，使src短轴旋转180度
    Eigen::Matrix3d rotation_180_short = Eigen::AngleAxisd(M_PI, src_short_axis.normalized()).toRotationMatrix();
    transformation.block<3, 3>(0, 0) = rotation_180_short * transformation.block<3, 3>(0, 0);
    std::cout << "变换矩阵: \n" << transformation << std::endl;
    pcd_src->Rotate(rotation_180_short, Eigen::Vector3d(0, 0, 0));
    src_oriented_bounding_box.Rotate(rotation_180_short, Eigen::Vector3d(0, 0, 0));
    std::cout << "src点云已沿短轴旋转180度" << std::endl;

    // 可视化沿短轴旋转180度后的点云和定向边界框
    if (flag == 1) {
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "沿短轴旋转180度后的点云和定向边界框可视化", 1600,1600);
        std::cout << "沿短轴旋转180度后的点云和定向边界框可视化成功" << std::endl;
    }
    // 保存沿短轴旋转180度后的点云和定向边界框可视化结果
    if (flag2 == 1) {
        open3d::io::WritePointCloud("7-rotated_180_short_axis_src.ply", *pcd_src);
        open3d::io::WritePointCloud("7-rotated_180_short_axis_ref.ply", *pcd_ref);
        std::cout << "沿短轴旋转180度后的点云和定向边界框结果保存成功" << std::endl;
    }


    // ICP精配准
    auto start_icp = std::chrono::high_resolution_clock::now();

    open3d::pipelines::registration::RegistrationResult icp_result = open3d::pipelines::registration::RegistrationICP(
        *pcd_src, *pcd_ref, 30*avg_dist, Eigen::Matrix4d::Identity(),
        open3d::pipelines::registration::TransformationEstimationPointToPoint());

    auto end_icp = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_icp = end_icp - start_icp;
    std::cout << "ICP精配准运行时间: " << elapsed_icp.count() << " 秒" << std::endl;

    // 更新变换矩阵
    transformation = icp_result.transformation_ * transformation;
    std::cout << "最终的变换矩阵: \n" << transformation << std::endl;
    
    std::cout << "ICP精配准后的变换矩阵: \n" << icp_result.transformation_ << std::endl;

    // 应用变换矩阵到源点云
    pcd_src->Transform(icp_result.transformation_);

    // 将pcd_ref移动回原来的位置,src跟上
    pcd_ref->Translate(ref_center);
    ref_oriented_bounding_box.Translate(ref_center);

    pcd_src->Translate(ref_center);
    src_oriented_bounding_box.Translate(ref_center);

    // 将变换矩阵加上Translate(ref_center)的操作
    transformation.block<3, 1>(0, 3) += ref_center;
    std::cout << "最终变换矩阵: \n" << transformation << std::endl;



    // 可视化ICP精配准结果
    if (flag == 1) {
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref}, "ICP精配准结果", 1600,1600);
        std::cout << "ICP精配准结果可视化成功" << std::endl;
    }
    // 保存ICP精配准结果
    if (flag2 == 1) {
        open3d::io::WritePointCloud("8-icp_result_src.ply", *pcd_src);
        open3d::io::WritePointCloud("8-icp_result_ref.ply", *pcd_ref);
        std::cout << "ICP精配准结果保存成功" << std::endl;
    }

    
    
}