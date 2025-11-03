#include "slam_global/LocalizerICP.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

using slam_global_types::Transform4d;
using slam_global_types::ICPResult;

static pcl::PointCloud<pcl::PointXYZ> voxelDownPCL(
    const pcl::PointCloud<pcl::PointXYZ> &cloud,
    double leaf) {
    if (leaf <= 0.0) {
        return cloud;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(static_cast<float>(leaf),
                   static_cast<float>(leaf),
                   static_cast<float>(leaf));
    vg.setInputCloud(cloud.makeShared());

    pcl::PointCloud<pcl::PointXYZ> out;
    vg.filter(out);
    return out;
}

ICPResult LocalizerICP::icpAtScale_(
    const pcl::PointCloud<pcl::PointXYZ> &scan,
    const pcl::PointCloud<pcl::PointXYZ> &submap,
    const Transform4d &init,
    double scale) const {

    ICPResult res;
    res.T = init;
    res.fitness = 0.0;

    // downsample for this scale
    double scan_leaf = scan_voxel_ * scale;
    double map_leaf  = map_voxel_  * scale;

    pcl::PointCloud<pcl::PointXYZ> scan_ds   = voxelDownPCL(scan,   scan_leaf);
    pcl::PointCloud<pcl::PointXYZ> submap_ds = voxelDownPCL(submap, map_leaf);

    if (scan_ds.empty() || submap_ds.empty()) {
        return res;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_ds.makeShared());
    icp.setInputTarget(submap_ds.makeShared());

    // tuning knobs
    icp.setMaxCorrespondenceDistance(1.0 * scale);
    icp.setMaximumIterations(20);

    // apply initial guess
    Eigen::Matrix4f init_f = init.cast<float>();
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, init_f);

    // get final
    Eigen::Matrix4f final_f = icp.getFinalTransformation();
    res.T = final_f.cast<double>();

    // compute fitness-like score from PCL's MSE
    double score = icp.getFitnessScore(); // MSE (smaller is better)
    res.fitness = 1.0 / (1.0 + score);    // map to (0,1], bigger is better

    return res;
}

bool LocalizerICP::align(
    const Transform4d &initial_guess,
    const pcl::PointCloud<pcl::PointXYZ> &scan,
    const pcl::PointCloud<pcl::PointXYZ> &submap,
    ICPResult &out_result) const {
    
    ICPResult coarse = icpAtScale_(scan, submap, initial_guess, 5.0);
    ICPResult fine   = icpAtScale_(scan, submap, coarse.T,      1.0);

    out_result = fine;

    if (fine.fitness < fitness_thresh_) {
        std::cout << "ICP failed: fitness=" << fine.fitness 
                  << " < thresh=" << fitness_thresh_ << std::endl;
        return false;
    }
    return true;
}
