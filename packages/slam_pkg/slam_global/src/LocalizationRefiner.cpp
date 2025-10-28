#include "slam_global/LocalizationRefiner.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>

using slam_global_types::Transform4d;
using slam_global_types::ICPResult;

static pcl::PointCloud<pcl::PointXYZ> voxelDownLocal(
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

ICPResult LocalizationRefiner::icpAtScale_(
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

    pcl::PointCloud<pcl::PointXYZ> scan_ds   = voxelDownLocal(scan,   scan_leaf);
    pcl::PointCloud<pcl::PointXYZ> submap_ds = voxelDownLocal(submap, map_leaf);

    if (scan_ds.empty() || submap_ds.empty()) {
        return res;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_ds.makeShared());
    icp.setInputTarget(submap_ds.makeShared());
    icp.setMaxCorrespondenceDistance(1.0 * scale);

    Eigen::Matrix4f init_f = init.cast<float>();
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, init_f);

    Eigen::Matrix4f final_f = icp.getFinalTransformation();
    res.T = final_f.cast<double>();

    double score = icp.getFitnessScore();
    res.fitness = 1.0 / (1.0 + score);

    return res;
}

bool LocalizationRefiner::refineICP(
    const Transform4d &last_T_map_to_odom,
    const pcl::PointCloud<pcl::PointXYZ> &scan,
    const pcl::PointCloud<pcl::PointXYZ> &submap,
    ICPResult &out_result)
{
    ICPResult coarse = icpAtScale_(scan, submap, last_T_map_to_odom, 5.0);
    ICPResult fine   = icpAtScale_(scan, submap, coarse.T,           1.0);

    out_result = fine;

    if (fine.fitness < fitness_thresh_) {
        return false;
    }
    return true;
}
