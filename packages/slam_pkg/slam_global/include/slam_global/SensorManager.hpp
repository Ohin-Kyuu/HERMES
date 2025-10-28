#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SensorManager {
public:
    SensorManager() = default;

    void updateScanMsg(const sensor_msgs::msg::PointCloud2 &msg);
    void updateOdomMsg(const nav_msgs::msg::Odometry &msg);

    bool haveScan() const { return have_scan_; }
    bool haveOdom() const { return have_odom_; }

    const pcl::PointCloud<pcl::PointXYZ> &getScan() const { return cur_scan_; }
    const nav_msgs::msg::Odometry &getOdom() const { return cur_odom_; }

private:
    pcl::PointCloud<pcl::PointXYZ> cur_scan_;
    nav_msgs::msg::Odometry        cur_odom_;

    bool have_scan_ = false;
    bool have_odom_ = false;
};
