/*聚类+对齐+4pcs
 * @Author: IntronRewrite weijiehe@sdust.edu.com
 * @Date: 2024-11-25 04:22:10
 * @LastEditors: IntronRewrite weijiehe@sdust.edu.com
 * @LastEditTime: 2024-11-25 06:05:39
 * @FilePath: /IndustrialVision/plan3/1.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <chrono>
#include <thread>
#include <open3d/Open3D.h>

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


int main() {

    int flag = 1;



    // 加载点云
    auto start_loading = std::chrono::high_resolution_clock::now();

    auto pcd_src = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud("../src_model.ply", *pcd_src);
    std::cout << "读取src点云数据成功" << std::endl;
    std::cout << "src点云的点数量: " << pcd_src->points_.size() << std::endl;

    auto pcd_ref = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud("../ref_model.ply", *pcd_ref);
    std::cout << "读取ref点云数据成功" << std::endl;
    std::cout << "ref点云的点数量: " << pcd_ref->points_.size() << std::endl;

    auto end_loading = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_loading = end_loading - start_loading;
    std::cout << "点云加载运行时间: " << elapsed_loading.count() << " 秒" << std::endl;



    // 均匀采样，每个保留10000个点
    auto start_sampling = std::chrono::high_resolution_clock::now();

    auto pcd_src_sampled = pcd_src->UniformDownSample(100);
    std::cout << "均匀采样src点云成功，采样后点数量: " << pcd_src_sampled->points_.size() << std::endl;


    auto end_sampling = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_sampling = end_sampling - start_sampling;
    std::cout << "均匀采样运行时间: " << elapsed_sampling.count() << " 秒" << std::endl;


    double avg_dist = ComputeAverageDistance(pcd_src_sampled);
    std::cout << "点云的平均距离: " << avg_dist << std::endl;

    //pcd_src_sampled，pcd_src_sampled做聚类，可视化
    // 移除离群点并进行DBSCAN聚类
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
        open3d::visualization::DrawGeometries({pcd_src_sampled}, "采样后的聚类结果", 1600, 900);
        std::cout << "采样后的聚类结果可视化成功" << std::endl;
    }
    
    

    // 保留数量最多的一类，选出pcd_src与该类相邻1.5*avg_dist的
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

    // 选出pcd_src与该类相邻1.5*avg_dist的点
    open3d::geometry::KDTreeFlann kdtree_src(*pcd_src);
    std::vector<size_t> neighbor_indices;
    for (const auto& point : largest_cluster_pcd_sampled->points_) {
        std::vector<int> indices;
        std::vector<double> distances;
        kdtree_src.SearchRadius(point, 3 * avg_dist, indices, distances);
        neighbor_indices.insert(neighbor_indices.end(), indices.begin(), indices.end());
    }


    // 选出相邻点
    auto neighbor_pcd = pcd_src->SelectByIndex(neighbor_indices);
    std::cout << "选出相邻点成功" << std::endl;

    
    
    // 合并neighbor_pcd和largest_cluster_pcd_sampled
    auto combined_pcd = std::make_shared<open3d::geometry::PointCloud>();
    *combined_pcd += *neighbor_pcd;
    *combined_pcd += *largest_cluster_pcd_sampled;
    pcd_src = combined_pcd;
    std::cout << "合并neighbor_pcd和largest_cluster_pcd_sampled成功" << std::endl;

    auto end_cluster_selection = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_cluster_selection = end_cluster_selection - start_cluster_selection;
    std::cout << "聚类选择运行时间: " << elapsed_cluster_selection.count() << " 秒" << std::endl;

    // 将pcd_src和pcd_ref分别用不同颜色显示
    pcd_src->PaintUniformColor(Eigen::Vector3d(244.0/255, 171.0/255, 154.0/255)); // 绿色
    pcd_ref->PaintUniformColor(Eigen::Vector3d(182.0/255, 194.0/255, 174.0/255)); // 红色

    // 可视化相邻点
    // 将largest_cluster_pcd_sampled添加到可视化中
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src}, "相邻点结果", 1600, 900);
    }
    

    
    // 预处理
    auto start = std::chrono::high_resolution_clock::now();

    auto src_oriented_bounding_box = pcd_src->GetOrientedBoundingBox();
    src_oriented_bounding_box.color_ = Eigen::Vector3d(0, 1, 0);
    std::cout << "获取src定向边界框成功" << std::endl;

    auto ref_oriented_bounding_box = pcd_ref->GetOrientedBoundingBox();
    ref_oriented_bounding_box.color_ = Eigen::Vector3d(1, 0, 0);
    std::cout << "获取ref定向边界框成功" << std::endl;

    

    // 可视化定向边界框
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "定向边界框可视化", 1600, 900);
        std::cout << "定向边界框可视化成功" << std::endl;
    }


    // 将点云平移到原点
    Eigen::Vector3d src_center = src_oriented_bounding_box.GetCenter();
    Eigen::Vector3d ref_center = ref_oriented_bounding_box.GetCenter();
    pcd_src->Translate(-src_center);
    src_oriented_bounding_box.Translate(-src_center);
    pcd_ref->Translate(-ref_center);
    ref_oriented_bounding_box.Translate(-ref_center);
    std::cout << "点云和定向边界框已移动到原点" << std::endl;

    // 可视化平移后的点云和定向边界框
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "平移后的点云和定向边界框可视化", 1600, 900);
        std::cout << "平移后的点云和定向边界框可视化成功" << std::endl;
    }

    // 旋转src以对齐ref
    Eigen::Matrix3d src_rotation = src_oriented_bounding_box.R_;
    Eigen::Matrix3d ref_rotation = ref_oriented_bounding_box.R_;
    
    std::cout << "src_rotation:" << std::endl << src_rotation << std::endl;
    std::cout << "ref_rotation:" << std::endl << ref_rotation << std::endl;

    Eigen::Matrix3d rotation = ref_rotation * src_rotation.inverse();
    pcd_src->Rotate(rotation, Eigen::Vector3d(0, 0, 0));
    src_oriented_bounding_box.Rotate(rotation, Eigen::Vector3d(0, 0, 0));
    std::cout << "src点云和定向边界框已旋转对齐ref" << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "粗配准运行时间: " << elapsed.count() << " 秒" << std::endl;


    

    // 可视化旋转对齐后的点云和定向边界框
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "旋转对齐后的点云和定向边界框可视化", 1600, 900);
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

    if (ref_extent.x() > ref_extent.y() && ref_extent.x() > ref_extent.z()) {
        ref_long_axis = ref_oriented_bounding_box.R_.col(0);
    } else if (ref_extent.y() > ref_extent.x() && ref_extent.y() > ref_extent.z()) {
        ref_long_axis = ref_oriented_bounding_box.R_.col(1);
    } else {
        ref_long_axis = ref_oriented_bounding_box.R_.col(2);
    }

    // 计算旋转矩阵，使src长轴旋转180度
    Eigen::Matrix3d rotation_180 = Eigen::AngleAxisd(M_PI, src_long_axis.normalized()).toRotationMatrix();
    // pcd_src->Rotate(rotation_180, Eigen::Vector3d(0, 0, 0));
    // src_oriented_bounding_box.Rotate(rotation_180, Eigen::Vector3d(0, 0, 0));

    // 将pcd_ref移动回原来的位置
    pcd_ref->Translate(ref_center);
    ref_oriented_bounding_box.Translate(ref_center);

    // 计算源点云总的变换矩阵
    Eigen::Matrix4d total_transformation = Eigen::Matrix4d::Identity();
    total_transformation.block<3, 3>(0, 0) = rotation_180 * rotation;
    total_transformation.block<3, 1>(0, 3) = -src_center;
    total_transformation.block<3, 1>(0, 3) += ref_center;

    std::cout << "源点云总的变换矩阵:" << std::endl << total_transformation << std::endl;

    // 应用总的变换矩阵到源点云
    pcd_src->Transform(total_transformation);

    // 分别应用平移和旋转到定向边界框
    Eigen::Vector3d translation = total_transformation.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation_matrix = total_transformation.block<3, 3>(0, 0);

    src_oriented_bounding_box.Translate(translation);
    src_oriented_bounding_box.Rotate(rotation_matrix, src_oriented_bounding_box.center_);

    std::cout << "src点云和定向边界框已绕长轴旋转180度" << std::endl;
    std::cout << "旋转对齐后的点云和定向边界框可视化成功" << std::endl;
    
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "旋转对齐后的点云和定向边界框可视化", 1600, 900);
    }

    std::cout << "此时pcd_src点的数量: " << pcd_src->points_.size() << std::endl;

    // 对pcd_src进行均匀下采样
    auto pcd_src_downsampled = pcd_src->UniformDownSample(80);
    std::cout << "均匀下采样src点云成功，采样后点数量: " << pcd_src_downsampled->points_.size() << std::endl;

    // 计算平均距离
    avg_dist = ComputeAverageDistance(pcd_src_downsampled);
    std::cout << "点云的平均距离: " << avg_dist << std::endl;


    // 4PCS配准
    auto start_4pcs = std::chrono::high_resolution_clock::now();

    open3d::pipelines::registration::RegistrationResult result_4pcs;
    open3d::pipelines::registration::TransformationEstimationPointToPoint estimation;
    double max_correspondence_distance = avg_dist * 42;
    open3d::pipelines::registration::ICPConvergenceCriteria criteria;
    Eigen::Matrix4d initial_transformation = Eigen::Matrix4d::Identity();

    // result_4pcs = open3d::pipelines::registration::RegistrationICP(*pcd_src, *pcd_ref, max_correspondence_distance, initial_transformation, estimation, criteria);

   
    // 用下采样的点进行4PCS粗配准
    result_4pcs = open3d::pipelines::registration::RegistrationICP(*pcd_src_downsampled, *pcd_ref, max_correspondence_distance, initial_transformation, estimation, criteria);
    std::cout << "4PCS配准结果转换矩阵:" << std::endl << result_4pcs.transformation_ << std::endl;

    auto end_4pcs = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_4pcs = end_4pcs - start_4pcs;
    std::cout << "4PCS配准运行时间: " << elapsed_4pcs.count() << " 秒" << std::endl;

    // 应用4PCS配准结果到原始点云
    pcd_src->Transform(result_4pcs.transformation_);

    // 应用4PCS配准结果
    pcd_src->Transform(result_4pcs.transformation_);

    
    

    // 可视化4PCS配准结果
    if(flag==1){
        open3d::visualization::DrawGeometries({pcd_src, pcd_ref}, "4PCS配准结果", 1600, 900);
        std::cout << "4PCS配准结果可视化成功" << std::endl;

    }
    

    // // 精配准
    // auto start_fine = std::chrono::high_resolution_clock::now();

    // open3d::pipelines::registration::RegistrationResult result_fine;
    // open3d::pipelines::registration::TransformationEstimationPointToPlane estimation_fine;
    // double max_correspondence_distance_fine = avg_dist * 4.2;
    // open3d::pipelines::registration::ICPConvergenceCriteria criteria_fine;

    // result_fine = open3d::pipelines::registration::RegistrationICP(*pcd_src, *pcd_ref, max_correspondence_distance_fine, result_4pcs.transformation_, estimation_fine, criteria_fine);

    // auto end_fine = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed_fine = end_fine - start_fine;
    // std::cout << "精配准运行时间: " << elapsed_fine.count() << " 秒" << std::endl;

    // std::cout << "精配准结果转换矩阵:" << std::endl << result_fine.transformation_ << std::endl;

    // // 应用精配准结果
    // pcd_src->Transform(result_fine.transformation_);

    // // 可视化精配准结果
    // open3d::visualization::DrawGeometries({pcd_src, pcd_ref}, "精配准结果", 1600, 900);
    // std::cout << "精配准结果可视化成功" << std::endl;

    return 0;
}