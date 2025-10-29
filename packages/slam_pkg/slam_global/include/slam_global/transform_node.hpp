#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class TransformNode : public rclcpp::Node
{
public:
    TransformNode();

private:
    // callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapToOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // periodic publisher (like your while True loop at 50 Hz)
    void timerCallback();

    // helper: Pose+Orient -> 4x4 matrix
    static Eigen::Matrix4d odomToMat4(const nav_msgs::msg::Odometry &odom_msg);

    // state
    nav_msgs::msg::Odometry cur_odom_to_baselink_;
    nav_msgs::msg::Odometry cur_map_to_odom_;
    bool have_odom_ = false;
    bool have_map_to_odom_ = false;

    // pubs
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_localization_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // timer (50 Hz => 20 ms)
    rclcpp::TimerBase::SharedPtr timer_;
};
