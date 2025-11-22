#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class OdomToVisionPoseBridge : public rclcpp::Node
{
public:
  OdomToVisionPoseBridge()
  : Node("odom_to_vision_pose_bridge")
  {
    std::string input_topic = "/localization";
    std::string output_topic = "/mavros/vision_pose/pose";

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      50,
      std::bind(&OdomToVisionPoseBridge::odomCallback, this, std::placeholders::_1)
    );

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      output_topic,
      50
    );

    RCLCPP_INFO(this->get_logger(), "OdomToVisionPoseBridge started.");
    RCLCPP_INFO(this->get_logger(), " Subscribing: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), " Publishing: %s", output_topic.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "camera_init";
    pose_msg.pose = msg->pose.pose;
    pose_msg.header.stamp = this->now();
    pose_pub_->publish(pose_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToVisionPoseBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
