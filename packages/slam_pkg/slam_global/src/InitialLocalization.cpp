#include "slam_global/InitialLocalization.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>

using slam_global_types::Transform4d;
using slam_global_types::ICPResult;

static pcl::PointCloud<pcl::PointXYZ> voxelDown(
    const pcl::PointCloud<pcl::PointXYZ> &cloud,
    double leaf)
{
    if (leaf <= 0.0) return cloud;

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(static_cast<float>(leaf),
                   static_cast<float>(leaf),
                   static_cast<float>(leaf));
    vg.setInputCloud(cloud.makeShared());

    pcl::PointCloud<pcl::PointXYZ> out;
    vg.filter(out);
    return out;
}

// ICP at given scale
ICPResult InitialLocalization::icpAtScale_(
    const pcl::PointCloud<pcl::PointXYZ> &scan,
    const pcl::PointCloud<pcl::PointXYZ> &submap,
    const Transform4d &init,
    double scale) const
{
    ICPResult res;
    res.T = init;
    res.fitness = 0.0;

    double scan_leaf = scan_voxel_ * scale;
    double map_leaf  = map_voxel_  * scale;

    pcl::PointCloud<pcl::PointXYZ> scan_ds   = voxelDown(scan,   scan_leaf);
    pcl::PointCloud<pcl::PointXYZ> submap_ds = voxelDown(submap, map_leaf);

    if (scan_ds.empty() || submap_ds.empty()) {
        return res; // stay default
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_ds.makeShared());
    icp.setInputTarget(submap_ds.makeShared());

    // Open3D 用 max_corr_dist * scale; 這裡用 setMaxCorrespondenceDistance 類似概念
    icp.setMaxCorrespondenceDistance(1.0 * scale);

    // 初始猜測
    Eigen::Matrix4f init_f = init.cast<float>();
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, init_f);

    Eigen::Matrix4f final_f = icp.getFinalTransformation();
    res.T = final_f.cast<double>();

    // 轉換 PCL fitnessScore() -> fake "fitness" in [0..1], 越大越好
    double score = icp.getFitnessScore();
    res.fitness = 1.0 / (1.0 + score);

    return res;
}

bool InitialLocalization::runICP(
    const Transform4d &initial_guess,
    const pcl::PointCloud<pcl::PointXYZ> &scan,
    const pcl::PointCloud<pcl::PointXYZ> &submap,
    ICPResult &out_result)
{
    ICPResult coarse = icpAtScale_(scan, submap, initial_guess, 5.0);
    ICPResult fine   = icpAtScale_(scan, submap, coarse.T,      1.0);

    out_result = fine;

    // mimic Open3D's "fitness must be >= threshold"
    if (fine.fitness < fitness_thresh_) {
        return false;
    }
    return true;
}
