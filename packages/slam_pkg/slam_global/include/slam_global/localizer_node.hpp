#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Types.hpp"
#include "MapLoader.hpp"
#include "SensorManager.hpp"
#include "SubmapExtractor.hpp"
#include "LocalizerICP.hpp"
#include "MapToOdomPublisher.hpp"

class GlobalLocalizationNode : public rclcpp::Node
{
public:
    GlobalLocalizationNode();

private:
    // --- ROS Callbacks ---
    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // --- internal logic ---
    void timerCallback();
    void tryStartTimer();

    bool canInitialize() const;
    bool canRefine() const;

    slam_global_types::Transform4d poseWithCovToMat4(
        const geometry_msgs::msg::PoseWithCovarianceStamped &msg) const;

    // --- State ---
    bool initialized_ = false;

    slam_global_types::Transform4d T_map_to_odom_;  // map->odom transform we maintain

    MapLoader           map_manager_;
    SensorManager       sensor_manager_;
    SubmapExtractor     submap_extractor_;
    LocalizerICP icp_localizer_;
    MapToOdomPublisher  map_to_odom_pub_;

    // --- ROS interface ---
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initpose_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cur_scan_viz_;

    rclcpp::TimerBase::SharedPtr timer_;
};
