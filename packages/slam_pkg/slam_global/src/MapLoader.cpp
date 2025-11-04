#include "slam_global/MapLoader.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

MapLoader::MapLoader(double voxel_size)
: voxel_size_(voxel_size)
{}

static pcl::PointCloud<pcl::PointXYZ> voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZ> &in,
    double leaf) {
    if (leaf <= 0.0) {
        return in; // no downsample
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(static_cast<float>(leaf),
                   static_cast<float>(leaf),
                   static_cast<float>(leaf));
    vg.setInputCloud(in.makeShared());

    pcl::PointCloud<pcl::PointXYZ> out;
    vg.filter(out);
    return out;
}

void MapLoader::loadMapMsg(const sensor_msgs::msg::PointCloud2 &msg) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ> cleaned;
    cleaned.reserve(pcl_cloud.size());
    for (const auto &p : pcl_cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.x == 0.f && p.y == 0.f && p.z == 0.f) continue;
        cleaned.points.emplace_back(pcl::PointXYZ(p.x, p.y, p.z));
    }
    cleaned.width  = cleaned.points.size();
    cleaned.height = 1;
    cleaned.is_dense = true;

    global_map_ = voxelDownsample(cleaned, voxel_size_);

    ready_ = true;
}
