#include "slam_global/SensorManager.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void SensorManager::updateScanMsg(const sensor_msgs::msg::PointCloud2 &msg) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);

    cur_scan_.clear();
    cur_scan_.reserve(pcl_cloud.size());

    for (const auto &p : pcl_cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.x == 0.f && p.y == 0.f && p.z == 0.f) continue;
        cur_scan_.points.emplace_back(pcl::PointXYZ(p.x, p.y, p.z));
    }

    cur_scan_.width  = cur_scan_.points.size();
    cur_scan_.height = 1;
    cur_scan_.is_dense = true;

    have_scan_ = true;
}

void SensorManager::updateOdomMsg(const nav_msgs::msg::Odometry &msg) {
    cur_odom_ = msg;
    have_odom_ = true;
}
