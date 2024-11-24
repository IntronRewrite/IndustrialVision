#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_fpcs.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    /****************FPCS配准********************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>); // 源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>); // 目标点云

    pcl::PLYReader reader;
    if (reader.read("../src_model.ply", *source) == -1) {
        PCL_ERROR("Couldn't read file src_model.ply \n");
        return (-1);
    }
    if (reader.read("../ref_model.ply", *target) == -1) {
        PCL_ERROR("Couldn't read file ref_model.ply \n");
        return (-1);
    }

    if (source->empty() || target->empty()) {
        PCL_ERROR("点云数据为空 \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>); // 目标点云转换后的结果

    // FPCS配准
    pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> registration;
    registration.setInputSource(source); // 源点云
    registration.setInputTarget(target); // 目标点云
    registration.setApproxOverlap(0.5); // 设置源和目标之间的近似重叠度。
    registration.setDelta(0.01); // 设置常数因子delta，用于对内部计算的参数进行加权
    registration.setNumberOfSamples(100); // 设置验证配准效果时要使用的采样点数量
    registration.align(*result);

    if (!registration.hasConverged()) {
        PCL_ERROR("配准未收敛 \n");
        return (-1);
    }

    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("FPCS"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target, 255, 0, 0); // 目标点云
    viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> result_color(result, 0, 255, 0); // 配准结果点云
    viewer->addPointCloud<pcl::PointXYZ>(result, result_color, "result cloud");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(1000000);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}