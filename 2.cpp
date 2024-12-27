// 用open3d ransac配准 /home/lw/IndustrialVision/ref.stp /home/lw/IndustrialVision/src.ply 
#include <open3d/Open3D.h>
#include <iostream>
#include <Eigen/Dense>

double ComputeAverageNeighborDistance(const open3d::geometry::PointCloud& point_cloud) 
{
    double total_distance = 0.0;
    int num_points = point_cloud.points_.size();
    open3d::geometry::KDTreeFlann kdtree(point_cloud);

    for (int i = 0; i < num_points; ++i) {
        std::vector<int> indices(2);
        std::vector<double> distances(2);
        kdtree.SearchKNN(point_cloud.points_[i], 2, indices, distances);
        total_distance += std::sqrt(distances[1]); // distances[0] is the point itself
    }

    return total_distance / num_points;
}

int main() {
    // 加载参考点云和源点云
    auto ref = std::make_shared<open3d::geometry::PointCloud>();
    auto src = std::make_shared<open3d::geometry::PointCloud>();
    std::cout << "加载参考点云..." << std::endl;
    if (!open3d::io::ReadPointCloud("/home/lw/IndustrialVision/ref_20w.ply", *ref)) {
        std::cerr << "读取参考点云失败。" << std::endl;
        return -1;
    }
    std::cout << "参考点云加载完成。" << std::endl;

    std::cout << "加载源点云..." << std::endl;
    if (!open3d::io::ReadPointCloud("/home/lw/IndustrialVision/src_20w.ply", *src)) {
        std::cerr << "读取源点云失败。" << std::endl;
        return -1;
    }
    std::cout << "源点云加载完成。" << std::endl;

    std::cout << "计算参考点云的平均最近邻距离..." << std::endl;
    double ref_avg_distance = ComputeAverageNeighborDistance(*ref);
    std::cout << "参考点云的平均最近邻距离: " << ref_avg_distance << std::endl;

    std::cout << "计算源点云的平均最近邻距离..." << std::endl;
    double src_avg_distance = ComputeAverageNeighborDistance(*src);
    std::cout << "源点云的平均最近邻距离: " << src_avg_distance << std::endl;

    // // 计算法线
    // std::cout << "估计参考点云的法线..." << std::endl;
    // ref->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(ref_avg_distance*3, 60));
    // std::cout << "参考点云的法线估计完成。" << std::endl;

    // std::cout << "估计源点云的法线..." << std::endl;
    // src->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(src_avg_distance*3, 60));
    // std::cout << "源点云的法线估计完成。" << std::endl;

    // // 计算FPFH特征
    // std::cout << "计算参考点云的FPFH特征..." << std::endl;
    // auto ref_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*ref, open3d::geometry::KDTreeSearchParamHybrid(ref_avg_distance*3, 100));
    // std::cout << "参考点云的FPFH特征计算完成。" << std::endl;

    // std::cout << "计算源点云的FPFH特征..." << std::endl;
    // auto src_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*src, open3d::geometry::KDTreeSearchParamHybrid(src_avg_distance*3, 100));
    // std::cout << "源点云的FPFH特征计算完成。" << std::endl;

    // 设置RANSAC配准参数
    double distance_threshold = ref_avg_distance + src_avg_distance;
    auto checker_edge_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> checkers = {
        std::cref(checker_edge_length),
        std::cref(checker_distance)
    };

    std::cout << "建立对应关系..." << std::endl;
    open3d::pipelines::registration::CorrespondenceSet correspondences;
    for (size_t i = 0; i < src->points_.size(); ++i) {
        correspondences.push_back(Eigen::Vector2i(i, i));
    }

    std::cout << "运行RANSAC配准..." << std::endl;
    auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnCorrespondence(
        *src, *ref, correspondences,
        distance_threshold,
        open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
        3, // ransac_n
        checkers,
        open3d::pipelines::registration::RANSACConvergenceCriteria(100000, 500)
    );

    // 可视化对齐结果
    std::cout << "应用变换到源点云..." << std::endl;
    auto transformed_src = std::make_shared<open3d::geometry::PointCloud>();
    *transformed_src = *src;
    transformed_src->Transform(result.transformation_);
    std::cout << "变换应用完成。" << std::endl;

    // 设置源点云颜色
    for (auto& point : transformed_src->points_) {
        transformed_src->colors_.push_back(Eigen::Vector3d(0, 1, 0)); // 红色
    }

    // 可视化参考点云和对齐后的源点云
    std::cout << "可视化参考点云和对齐后的源点云..." << std::endl;
    open3d::visualization::DrawGeometries({ref, transformed_src}, "Aligned Point Clouds", 800, 600, 50, 50, false);

    return 0;
}
