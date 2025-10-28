#include "slam_global/SubmapExtractor.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

using slam_global_types::Transform4d;

static Transform4d odomToBase(const nav_msgs::msg::Odometry &odom)
{
    Transform4d T = Transform4d::Identity();
    const auto &p = odom.pose.pose.position;
    const auto &q = odom.pose.pose.orientation;
    Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
    T.block<3,3>(0,0) = Q.toRotationMatrix();
    T.block<3,1>(0,3) = Eigen::Vector3d(p.x, p.y, p.z);
    return T;
}

static Transform4d invertSE3(const Transform4d &T)
{
    Transform4d Ti = Transform4d::Identity();
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    Ti.block<3,3>(0,0) = R.transpose();
    Ti.block<3,1>(0,3) = -R.transpose() * t;
    return Ti;
}

SubmapExtractor::SubmapExtractor(double fov_rad, double fov_far_dist)
: fov_rad_(fov_rad), fov_far_(fov_far_dist)
{}

pcl::PointCloud<pcl::PointXYZ> SubmapExtractor::extract(
    const pcl::PointCloud<pcl::PointXYZ> &global_map,
    const Transform4d &T_map_to_odom_guess,
    const nav_msgs::msg::Odometry &odom_msg) const
{
    // 1. odom -> base_link
    Transform4d T_odom_to_base = odomToBase(odom_msg);

    // 2. map -> base_link (guess * odom->base)
    Transform4d T_map_to_base = T_map_to_odom_guess * T_odom_to_base;

    // 3. base_link -> map
    Transform4d T_base_to_map = invertSE3(T_map_to_base);

    pcl::PointCloud<pcl::PointXYZ> submap;
    submap.reserve(global_map.size());

    for (const auto &pt_map : global_map.points)
    {
        Eigen::Vector4d p_map(
            static_cast<double>(pt_map.x),
            static_cast<double>(pt_map.y),
            static_cast<double>(pt_map.z),
            1.0);

        Eigen::Vector4d p_base = T_base_to_map * p_map;
        double x = p_base.x();
        double y = p_base.y();
        // double z = p_base.z();

        double dist_xy = std::sqrt(x*x + y*y);
        if (dist_xy > fov_far_) continue;

        double ang = std::atan2(y, x);
        if (std::fabs(ang) > (fov_rad_ * 0.5)) continue;

        submap.points.emplace_back(pt_map);
    }

    submap.width  = submap.points.size();
    submap.height = 1;
    submap.is_dense = true;

    return submap;
}
