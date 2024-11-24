/*
 * @Author: IntronRewrite weijiehe@sdust.edu.com
 * @Date: 2024-11-24 23:20:59
 * @LastEditors: IntronRewrite weijiehe@sdust.edu.com
 * @LastEditTime: 2024-11-25 00:13:48
 * @FilePath: /IndustrialVision/plan2/3.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <open3d/Open3D.h>

int main() {
    // 加载点云
    auto pcd_src = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud("../src_model.ply", *pcd_src);
    std::cout << "读取src点云数据成功" << std::endl;
    std::cout << "src点云的点数量: " << pcd_src->points_.size() << std::endl;

    auto pcd_ref = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud("../ref_model.ply", *pcd_ref);
    std::cout << "读取ref点云数据成功" << std::endl;
    std::cout << "src点云的点数量: " << pcd_ref->points_.size() << std::endl;
    

    // 计算平均距离
    double avg_dist = 1.81265;
    std::cout << "点云的平均距离: " << avg_dist << std::endl;

    

    // 移除离群点并进行DBSCAN聚类
    std::vector<int> labels = pcd_src->ClusterDBSCAN(avg_dist * 4.2, 1, true);
    int max_label = *std::max_element(labels.begin(), labels.end());
    std::cout << "点云有 " << max_label + 1 << " 个聚类" << std::endl;


    

    // 为聚类分配颜色
    std::vector<Eigen::Vector3d> colors(pcd_src->points_.size(), Eigen::Vector3d(0, 0, 0));
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            colors[i] = Eigen::Vector3d::Random().cwiseAbs();
        }
    }
    pcd_src->colors_ = colors;
    std::cout << "欧式聚类成功" << std::endl;

    // 可视化聚类结果
    open3d::visualization::DrawGeometries({pcd_src}, "聚类结果", 1600, 900);
    std::cout << "聚类结果可视化成功" << std::endl;

    // 保留最大的聚类
    std::vector<int> largest_cluster_indices;
    int largest_cluster_idx = std::distance(labels.begin(), std::max_element(labels.begin(), labels.end()));
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == largest_cluster_idx) {
            largest_cluster_indices.push_back(i);
        }
    }
    std::vector<size_t> largest_cluster_indices_size_t(largest_cluster_indices.begin(), largest_cluster_indices.end());
    pcd_src = pcd_src->SelectByIndex(largest_cluster_indices_size_t);
    std::cout << "保留点最多的一个聚类成功" << std::endl;

    // 可视化聚类结果
    open3d::visualization::DrawGeometries({pcd_src}, "聚类结果", 1600, 900);
    std::cout << "聚类结果可视化成功" << std::endl;

    // 粗配准
    auto src_oriented_bounding_box = pcd_src->GetOrientedBoundingBox();
    src_oriented_bounding_box.color_ = Eigen::Vector3d(0, 1, 0);
    std::cout << "获取src定向边界框成功" << std::endl;

    auto ref_oriented_bounding_box = pcd_ref->GetOrientedBoundingBox();
    ref_oriented_bounding_box.color_ = Eigen::Vector3d(1, 0, 0);
    std::cout << "获取ref定向边界框成功" << std::endl;

    // 可视化定向边界框
    open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "定向边界框可视化", 1600, 900);
    std::cout << "定向边界框可视化成功" << std::endl;

    // 将pcd_ref颜色设置为单一颜色（例如红色）
    pcd_ref->PaintUniformColor(Eigen::Vector3d(1, 0, 0));

    // 将点云平移到原点
    Eigen::Vector3d src_center = src_oriented_bounding_box.GetCenter();
    Eigen::Vector3d ref_center = ref_oriented_bounding_box.GetCenter();
    pcd_src->Translate(-src_center);
    src_oriented_bounding_box.Translate(-src_center);
    pcd_ref->Translate(-ref_center);
    ref_oriented_bounding_box.Translate(-ref_center);
    std::cout << "点云和定向边界框已移动到原点" << std::endl;

    // 可视化平移后的点云和定向边界框
    open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "平移后的点云和定向边界框可视化", 1600, 900);
    std::cout << "平移后的点云和定向边界框可视化成功" << std::endl;

    // 旋转src以对齐ref
    Eigen::Matrix3d src_rotation = src_oriented_bounding_box.R_;
    Eigen::Matrix3d ref_rotation = ref_oriented_bounding_box.R_;
    Eigen::Matrix3d rotation = ref_rotation * src_rotation.inverse();
    pcd_src->Rotate(rotation, Eigen::Vector3d(0, 0, 0));
    src_oriented_bounding_box.Rotate(rotation, Eigen::Vector3d(0, 0, 0));
    std::cout << "src点云和定向边界框已旋转对齐ref" << std::endl;

    // 可视化旋转对齐后的点云和定向边界框
    open3d::visualization::DrawGeometries({pcd_src, pcd_ref, std::make_shared<open3d::geometry::OrientedBoundingBox>(src_oriented_bounding_box), std::make_shared<open3d::geometry::OrientedBoundingBox>(ref_oriented_bounding_box)}, "旋转对齐后的点云和定向边界框可视化", 1600, 900);
    std::cout << "旋转对齐后的点云和定向边界框可视化成功" << std::endl;

    // // 使用ICP进行精配准
    // double threshold = 1.0;
    // Eigen::Matrix4d trans_init = Eigen::Matrix4d::Identity();
    // auto reg_p2p = open3d::pipelines::registration::RegistrationICP(
    //     *pcd_src, *pcd_ref, threshold, trans_init,
    //     open3d::pipelines::registration::TransformationEstimationPointToPoint()
    // );
    // Eigen::Matrix4d transformation_icp = reg_p2p.transformation_;
    // std::cout << "ICP 变换矩阵:" << std::endl << transformation_icp << std::endl;

    // // 应用变换
    // pcd_src->Transform(transformation_icp);
    // std::cout << "src点云和ref点云精配准成功" << std::endl;

    // // 可视化结果
    // open3d::visualization::DrawGeometries({pcd_src, pcd_ref});
    return 0;
}