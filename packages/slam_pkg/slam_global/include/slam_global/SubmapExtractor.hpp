#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include "Types.hpp"  // for slam_global_types::Transform4d

class SubmapExtractor
{
public:
    SubmapExtractor(double fov_rad, double fov_far_dist);

    pcl::PointCloud<pcl::PointXYZ> extract(
        const pcl::PointCloud<pcl::PointXYZ> &global_map,
        const slam_global_types::Transform4d &T_map_to_odom_guess,
        const nav_msgs::msg::Odometry &odom_msg) const;

private:
    double fov_rad_;
    double fov_far_;
};
