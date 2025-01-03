// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "open3d/Open3D.h"

using namespace open3d;

// 预处理点云函数，返回原始点云、降采样点云和FPFH特征
std::tuple<std::shared_ptr<geometry::PointCloud>,
           std::shared_ptr<geometry::PointCloud>,
           std::shared_ptr<pipelines::registration::Feature>>
PreprocessPointCloud(const char *file_name, const float voxel_size) {
    // 从文件创建点云
    auto pcd = open3d::io::CreatePointCloudFromFile(file_name);
    // 对点云进行体素降采样
    auto pcd_down = pcd->VoxelDownSample(voxel_size);
    // 估计法线
    pcd_down->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(2 * voxel_size, 30));
    // 计算FPFH特征
    auto pcd_fpfh = pipelines::registration::ComputeFPFHFeature(
            *pcd_down,
            open3d::geometry::KDTreeSearchParamHybrid(5 * voxel_size, 100));
    // 返回原始点云、降采样点云和FPFH特征
    return std::make_tuple(pcd, pcd_down, pcd_fpfh);
}

// 可视化配准结果
void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    // 创建共享指针保存变换后的源点云和目标点云
    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
            new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target_ptr(new geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    // 对源点云进行变换
    source_transformed_ptr->Transform(Transformation);
    // 可视化变换后的源点云和目标点云
    visualization::DrawGeometries({source_transformed_ptr, target_ptr},
                                  "Registration result");
}

// 打印帮助信息
void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > RegistrationRANSAC source_pcd target_pcd"
                     "[--method=feature_matching] "
                     "[--voxel_size=0.05] [--distance_multiplier=1.5]"
                     "[--max_iterations 1000000] [--confidence 0.999]"
                     "[--mutual_filter]");
    // clang-format on
}

int main(int argc, char *argv[]) {
    using namespace open3d;

    // 设置日志级别为Debug
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    // 如果参数不足或存在帮助选项，打印帮助信息并退出
    if (argc < 3 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    // 解析方法选项，默认为"feature_matching"
    std::string method = "";
    const std::string kMethodFeature = "feature_matching";
    const std::string kMethodCorres = "correspondence";
    if (utility::ProgramOptionExists(argc, argv, "--method")) {
        method = utility::GetProgramOptionAsString(argc, argv, "--method");
    } else {
        method = "feature_matching";
    }
    // 如果方法选项不合法，打印错误信息并退出
    if (method != kMethodFeature && method != kMethodCorres) {
        utility::LogInfo(
                "--method must be \'feature_matching\' or "
                "\'correspondence\'.");
        return 1;
    }

    // 解析是否使用互滤选项
    bool mutual_filter = false;
    if (utility::ProgramOptionExists(argc, argv, "--mutual_filter")) {
        mutual_filter = true;
    }
    // 解析体素大小选项，默认为0.05
    float voxel_size =
            utility::GetProgramOptionAsDouble(argc, argv, "--voxel_size", 0.05);
    // 解析距离乘数选项，默认为1.5
    float distance_multiplier = utility::GetProgramOptionAsDouble(
            argc, argv, "--distance_multiplier", 1.5);
    // 计算距离阈值
    float distance_threshold = voxel_size * distance_multiplier;
    // 解析最大迭代次数选项，默认为1000000
    int max_iterations = utility::GetProgramOptionAsInt(
            argc, argv, "--max_iterations", 1000000);
    // 解析置信度选项，默认为0.999
    float confidence = utility::GetProgramOptionAsDouble(argc, argv,
                                                         "--confidence", 0.999);

    // 准备输入点云
    std::shared_ptr<geometry::PointCloud> source, source_down, target,
            target_down;
    std::shared_ptr<pipelines::registration::Feature> source_fpfh, target_fpfh;
    std::tie(source, source_down, source_fpfh) =
            PreprocessPointCloud(argv[1], voxel_size);
    std::tie(target, target_down, target_fpfh) =
            PreprocessPointCloud(argv[2], voxel_size);

    pipelines::registration::RegistrationResult registration_result;

    // 准备检查器
    std::vector<std::reference_wrapper<
            const pipelines::registration::CorrespondenceChecker>>
            correspondence_checker;
    auto correspondence_checker_edge_length =
            pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
                    0.9);
    auto correspondence_checker_distance =
            pipelines::registration::CorrespondenceCheckerBasedOnDistance(
                    distance_threshold);
    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);

    // 根据方法选项进行配准
    if (method == kMethodFeature) {
        registration_result = pipelines::registration::
                RegistrationRANSACBasedOnFeatureMatching(
                        *source_down, *target_down, *source_fpfh, *target_fpfh,
                        mutual_filter, distance_threshold,
                        pipelines::registration::
                                TransformationEstimationPointToPoint(false),
                        3, correspondence_checker,
                        pipelines::registration::RANSACConvergenceCriteria(
                                max_iterations, confidence));
    } else if (method == kMethodCorres) {
        // 手动搜索对应关系
        int nPti = int(source_down->points_.size());
        int nPtj = int(target_down->points_.size());

        geometry::KDTreeFlann feature_tree_i(*source_fpfh);
        geometry::KDTreeFlann feature_tree_j(*target_fpfh);

        pipelines::registration::CorrespondenceSet corres_ji;
        std::vector<int> i_to_j(nPti, -1);

        // 缓存所有对应关系
        for (int j = 0; j < nPtj; j++) {
            std::vector<int> corres_tmp(1);
            std::vector<double> dist_tmp(1);

            feature_tree_i.SearchKNN(Eigen::VectorXd(target_fpfh->data_.col(j)),
                                     1, corres_tmp, dist_tmp);
            int i = corres_tmp[0];
            corres_ji.push_back(Eigen::Vector2i(i, j));
        }

        // 如果使用互滤，进行互滤操作
        if (mutual_filter) {
            pipelines::registration::CorrespondenceSet mutual_corres;
            for (auto &corres : corres_ji) {
                int j = corres(1);
                int j2i = corres(0);

                std::vector<int> corres_tmp(1);
                std::vector<double> dist_tmp(1);
                feature_tree_j.SearchKNN(
                        Eigen::VectorXd(source_fpfh->data_.col(j2i)), 1,
                        corres_tmp, dist_tmp);
                int i2j = corres_tmp[0];
                if (i2j == j) {
                    mutual_corres.push_back(corres);
                }
            }

            utility::LogDebug("{:d} points remain after mutual filter",
                              mutual_corres.size());
            registration_result = pipelines::registration::
                    RegistrationRANSACBasedOnCorrespondence(
                            *source_down, *target_down, mutual_corres,
                            distance_threshold,
                            pipelines::registration::
                                    TransformationEstimationPointToPoint(false),
                            3, correspondence_checker,
                            pipelines::registration::RANSACConvergenceCriteria(
                                    max_iterations, confidence));
        } else {
            utility::LogDebug("{:d} points remain", corres_ji.size());
            registration_result = pipelines::registration::
                    RegistrationRANSACBasedOnCorrespondence(
                            *source_down, *target_down, corres_ji,
                            distance_threshold,
                            pipelines::registration::
                                    TransformationEstimationPointToPoint(false),
                            3, correspondence_checker,
                            pipelines::registration::RANSACConvergenceCriteria(
                                    max_iterations, confidence));
        }
    }

    // 可视化配准结果
    VisualizeRegistration(*source, *target,
                          registration_result.transformation_);

    return 0;
}