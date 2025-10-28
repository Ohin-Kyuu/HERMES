#include "slam_global/localizer_node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using slam_global_types::Transform4d;
using slam_global_types::ICPResult;

// helper: PCL -> PointCloud2 (with stride decimation)
static sensor_msgs::msg::PointCloud2 pclToROSMsg(
    const pcl::PointCloud<pcl::PointXYZ> &pc,
    const std::string &frame_id,
    rclcpp::Time stamp,
    int stride = 1)
{
    pcl::PointCloud<pcl::PointXYZ> decimated;
    decimated.reserve(pc.size() / (stride > 0 ? stride : 1));
    for (size_t i = 0; i < pc.size(); i += ((stride>0)?stride:1)) {
        decimated.points.push_back(pc.points[i]);
    }
    decimated.width  = decimated.points.size();
    decimated.height = 1;
    decimated.is_dense = true;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(decimated, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    return msg;
}

GlobalLocalizationNode::GlobalLocalizationNode()
: rclcpp::Node("global_localization_node"),
  map_manager_(0.4 /*voxel*/),
  sensor_manager_(),
  submap_extractor_(6.28 /*fov_rad*/, 30.0 /*fov_far*/),
  init_localizer_(0.4 /*map_voxel*/, 0.1 /*scan_voxel*/, 0.95 /*fitness_thresh*/),
  refiner_(0.4 /*map_voxel*/, 0.1 /*scan_voxel*/, 0.95 /*fitness_thresh*/),
  map_to_odom_pub_(*this)
{
    // 初始 T_map_to_odom = Identity
    T_map_to_odom_.setIdentity();

    // debug publishes
    pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
    pub_cur_scan_viz_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 1);

    // subs
    sub_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/map", 10,
        std::bind(&GlobalLocalizationNode::mapCallback, this, std::placeholders::_1));

    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 10,
        std::bind(&GlobalLocalizationNode::scanCallback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 50,
        std::bind(&GlobalLocalizationNode::odomCallback, this, std::placeholders::_1));

    sub_initpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&GlobalLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GlobalLocalizationNode started.");
}

void GlobalLocalizationNode::mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!map_manager_.isReady()) {
        map_manager_.loadMapMsg(*msg);
        RCLCPP_INFO(this->get_logger(), "Loaded global map (%zu pts)",
                    map_manager_.getMap().points.size());
    }
}

void GlobalLocalizationNode::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    sensor_manager_.updateScanMsg(*msg);

    // publish debug current scan as-is (camera_init frame)
    auto scan_msg = pclToROSMsg(
        sensor_manager_.getScan(),
        /*frame_id*/ "camera_init",
        this->now(),
        /*stride*/5);
    pub_cur_scan_viz_->publish(scan_msg);
}

void GlobalLocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    sensor_manager_.updateOdomMsg(*msg);
}

bool GlobalLocalizationNode::canInitialize() const
{
    return map_manager_.isReady()
        && sensor_manager_.haveScan()
        && sensor_manager_.haveOdom()
        && !initialized_;
}

bool GlobalLocalizationNode::canRefine() const
{
    return map_manager_.isReady()
        && sensor_manager_.haveScan()
        && sensor_manager_.haveOdom()
        && initialized_;
}

Transform4d GlobalLocalizationNode::poseWithCovToMat4(
    const geometry_msgs::msg::PoseWithCovarianceStamped &msg) const
{
    Transform4d T = Transform4d::Identity();
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
    T.block<3,3>(0,0) = Q.toRotationMatrix();
    T.block<3,1>(0,3) = Eigen::Vector3d(p.x, p.y, p.z);
    return T;
}

void GlobalLocalizationNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!canInitialize()) {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot initialize yet (need map+scan+odom and not yet initialized)");
        return;
    }

    Transform4d T_init_guess = poseWithCovToMat4(*msg);

    // 1. extract submap around this guess
    pcl::PointCloud<pcl::PointXYZ> submap_cloud = submap_extractor_.extract(
        map_manager_.getMap(),
        T_init_guess,
        sensor_manager_.getOdom());

    // publish submap for RViz
    auto submap_msg = pclToROSMsg(
        submap_cloud,
        /*frame_id*/ "map",
        this->now(),
        /*stride*/10);
    pub_submap_->publish(submap_msg);

    // 2. ICP initial alignment
    ICPResult result;
    bool ok = init_localizer_.runICP(
        T_init_guess,
        sensor_manager_.getScan(),
        submap_cloud,
        result);

    if (!ok) {
        RCLCPP_WARN(this->get_logger(), "Initial ICP failed (fitness too low)");
        // ...可加 debug log result.fitness
        return;
    }

    // 3. accept
    T_map_to_odom_ = result.T;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Initialized! fitness=%.3f", result.fitness);

    // 4. publish /map_to_odom
    map_to_odom_pub_.publishOdometry(T_map_to_odom_, sensor_manager_.getOdom());

    // 5. start periodic refine
    tryStartTimer();
}

void GlobalLocalizationNode::tryStartTimer()
{
    if (timer_) return;
    if (!initialized_) return;
    if (!sensor_manager_.haveScan() || !sensor_manager_.haveOdom() || !map_manager_.isReady()) return;

    // run refine at ~0.5 Hz (2 seconds)
    auto period = std::chrono::milliseconds(2000);
    timer_ = this->create_wall_timer(
        period,
        std::bind(&GlobalLocalizationNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Refinement timer started.");
}

void GlobalLocalizationNode::timerCallback()
{
    if (!canRefine()) {
        return;
    }

    // 1. submap based on last T_map_to_odom_
    pcl::PointCloud<pcl::PointXYZ> submap_cloud = submap_extractor_.extract(
        map_manager_.getMap(),
        T_map_to_odom_,
        sensor_manager_.getOdom());

    // publish submap debug
    auto submap_msg = pclToROSMsg(
        submap_cloud,
        "map",
        this->now(),
        10);
    pub_submap_->publish(submap_msg);

    // 2. refine ICP
    ICPResult result;
    bool ok = refiner_.refineICP(
        T_map_to_odom_,
        sensor_manager_.getScan(),
        submap_cloud,
        result);

    if (!ok) {
        RCLCPP_WARN(this->get_logger(), "Refine ICP rejected (fitness too low)");
        return;
    }

    // 3. accept new transform
    T_map_to_odom_ = result.T;
    RCLCPP_INFO(this->get_logger(), "Refined ICP fitness=%.3f", result.fitness);

    // 4. republish /map_to_odom
    map_to_odom_pub_.publishOdometry(T_map_to_odom_, sensor_manager_.getOdom());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GlobalLocalizationNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}