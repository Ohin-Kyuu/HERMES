#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Types.hpp"  // Transform4d, ICPResult

class LocalizerICP
{
public:
    LocalizerICP(double map_voxel, double scan_voxel, double fitness_thresh)
    : map_voxel_(map_voxel)
    , scan_voxel_(scan_voxel)
    , fitness_thresh_(fitness_thresh)
    {}

    bool align(
        const slam_global_types::Transform4d &initial_guess,
        const pcl::PointCloud<pcl::PointXYZ> &scan,
        const pcl::PointCloud<pcl::PointXYZ> &submap,
        slam_global_types::ICPResult &out_result) const;

private:
    slam_global_types::ICPResult icpAtScale_(
        const pcl::PointCloud<pcl::PointXYZ> &scan,
        const pcl::PointCloud<pcl::PointXYZ> &submap,
        const slam_global_types::Transform4d &init,
        double scale) const;

    double map_voxel_;
    double scan_voxel_;
    double fitness_thresh_;
};
