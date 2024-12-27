// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include "open3d/Open3D.h"

using namespace open3d;

void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
            new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target_ptr(new geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(Transformation);

    // Set colors for the point clouds
    source_transformed_ptr->PaintUniformColor(Eigen::Vector3d(1, 0, 0)); // Red
    target_ptr->PaintUniformColor(Eigen::Vector3d(0, 1, 0)); // Green

    visualization::DrawGeometries({source_transformed_ptr, target_ptr},
                                  "Registration result");
}

void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > RegistrationColoredICP source_pcd target_pcd [--visualize]");
    // clang-format on
    utility::LogInfo("");
}

int main(int argc, char *argv[]) {
    using namespace open3d;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 3 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    bool visualize = false;
    if (utility::ProgramOptionExists(argc, argv, "--visualize")) {
        visualize = true;
    }

    // Prepare input
    std::shared_ptr<geometry::PointCloud> source =
            open3d::io::CreatePointCloudFromFile(argv[1]);
    std::shared_ptr<geometry::PointCloud> target =
            open3d::io::CreatePointCloudFromFile(argv[2]);
    if (source == nullptr || target == nullptr) {
        utility::LogWarning("Unable to load source or target file.");
        return -1;
    }
    double size = 8.0;
    std::vector<double> voxel_sizes = {size, size / 2, size / 4};
    std::vector<int> iterations = {14, 30, 50};
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    // Read initial transformation matrix from file
    std::ifstream file("/home/lw/IndustrialVision/build/transformation_matrix.txt");
    if (file.is_open()) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                file >> trans(i, j);
            }
        }
        file.close();
    } else {
        utility::LogWarning("Unable to open transformation matrix file.");
        return -1;
    }




    for (int i = 0; i < 3; ++i) {
        float voxel_size = voxel_sizes[i];

        auto source_down = source->VoxelDownSample(voxel_size);
        source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        auto target_down = target->VoxelDownSample(voxel_size);
        target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
                voxel_size * 2.0, 30));

        auto result = pipelines::registration::RegistrationICP(
                *source_down, *target_down, voxel_size * 1.5 , trans,
                pipelines::registration::
                        TransformationEstimationPointToPoint(),
                pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6,
                                                                iterations[i]));
        trans = result.transformation_;

        if (visualize) {
            VisualizeRegistration(*source, *target, trans);
        }
    }

    std::stringstream ss;
    ss << trans;
    utility::LogInfo("Final transformation = \n{}", ss.str());

    return 0;
}