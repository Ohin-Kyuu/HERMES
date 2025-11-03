#include "slam_global/transform_node.hpp"

using std::placeholders::_1;

TransformNode::TransformNode()
: rclcpp::Node("transform_fusion") {
    RCLCPP_INFO(this->get_logger(), "Transform Fusion Node Inited...");

    // publisher: /localization
    pub_localization_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/localization", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // subscribers
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry",
        qos,
        std::bind(&TransformNode::odomCallback, this, _1));

    this->create_subscription<nav_msgs::msg::Odometry>(
        "/map_to_odom",
        qos,
        std::bind(&TransformNode::mapToOdomCallback, this, _1));

    // timer at 50 Hz (20 ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TransformNode::timerCallback, this));
}

void TransformNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    cur_odom_to_baselink_ = *msg;
    have_odom_ = true;
}

void TransformNode::mapToOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    cur_map_to_odom_ = *msg;
    have_map_to_odom_ = true;
}

Eigen::Matrix4d TransformNode::odomToMat4(
    const nav_msgs::msg::Odometry &odom_msg) {
    const auto &p = odom_msg.pose.pose.position;
    const auto &q = odom_msg.pose.pose.orientation;

    Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
    Eigen::Matrix3d R = Q.toRotationMatrix();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Eigen::Vector3d(p.x, p.y, p.z);

    return T;
}

void TransformNode::timerCallback() {
    //  T_map_to_odom
    Eigen::Matrix4d T_map_to_odom = Eigen::Matrix4d::Identity();
    if (have_map_to_odom_) {
        T_map_to_odom = odomToMat4(cur_map_to_odom_);
    }

    //  TF: map -> camera_init
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id  = "camera_init";  
        Eigen::Vector3d t = T_map_to_odom.block<3,1>(0,3);
        Eigen::Matrix3d R = T_map_to_odom.block<3,3>(0,0);
        Eigen::Quaterniond q(R);

        tf_msg.transform.translation.x = t.x();
        tf_msg.transform.translation.y = t.y();
        tf_msg.transform.translation.z = t.z();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    if (!have_odom_) {
        return;
    }

    // T_odom_to_base_link 
    Eigen::Matrix4d T_odom_to_base_link = odomToMat4(cur_odom_to_baselink_);

    // T_map_to_base_link = T_map_to_odom * T_odom_to_base_link
    Eigen::Matrix4d T_map_to_base_link = T_map_to_odom * T_odom_to_base_link;

    Eigen::Vector3d xyz = T_map_to_base_link.block<3,1>(0,3);
    Eigen::Matrix3d R_mb = T_map_to_base_link.block<3,3>(0,0);
    Eigen::Quaterniond q_mb(R_mb);

    nav_msgs::msg::Odometry localization;

    // pose (map frame)
    localization.pose.pose.position.x = xyz.x();
    localization.pose.pose.position.y = xyz.y();
    localization.pose.pose.position.z = xyz.z();
    localization.pose.pose.orientation.x = q_mb.x();
    localization.pose.pose.orientation.y = q_mb.y();
    localization.pose.pose.orientation.z = q_mb.z();
    localization.pose.pose.orientation.w = q_mb.w();

    // twist
    localization.twist = cur_odom_to_baselink_.twist;

    // header
    localization.header.stamp = cur_odom_to_baselink_.header.stamp;
    localization.header.frame_id = "map";
    localization.child_frame_id = "body";

    pub_localization_->publish(localization);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TransformNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}