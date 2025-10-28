#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MapLoader {
public:
    explicit MapLoader(double voxel_size = 0.4);

    void loadMapMsg(const sensor_msgs::msg::PointCloud2 &msg);  // set global_map_
    bool isReady() const { return ready_; }

    const pcl::PointCloud<pcl::PointXYZ> &getMap() const { return global_map_; }

private:
    pcl::PointCloud<pcl::PointXYZ> global_map_;
    double voxel_size_;
    bool ready_ = false;
};