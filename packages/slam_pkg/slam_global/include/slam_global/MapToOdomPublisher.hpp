#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "Types.hpp"

class MapToOdomPublisher {
public:
    explicit MapToOdomPublisher(rclcpp::Node &node);

    void publishOdometry(
        const slam_global_types::Transform4d &T_map_to_odom,
        const nav_msgs::msg::Odometry &odom_time_source);

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;
};
