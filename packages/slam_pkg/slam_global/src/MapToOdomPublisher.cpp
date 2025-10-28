#include "slam_global/MapToOdomPublisher.hpp"

using slam_global_types::Transform4d;

MapToOdomPublisher::MapToOdomPublisher(rclcpp::Node &node)
{
    pub_map_to_odom_ = node.create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 1);
}

void MapToOdomPublisher::publishOdometry(
    const Transform4d &T_map_to_odom,
    const nav_msgs::msg::Odometry &odom_time_source)
{
    nav_msgs::msg::Odometry msg;
    msg.header = odom_time_source.header;
    msg.header.frame_id = "map";

    Eigen::Vector3d t = T_map_to_odom.block<3,1>(0,3);
    Eigen::Matrix3d R = T_map_to_odom.block<3,3>(0,0);
    Eigen::Quaterniond q(R);

    msg.pose.pose.position.x = t.x();
    msg.pose.pose.position.y = t.y();
    msg.pose.pose.position.z = t.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    pub_map_to_odom_->publish(msg);
}
