#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace slam_global_types
{
    using Transform4d = Eigen::Matrix<double, 4, 4>;

    struct ICPResult {
        Transform4d T; 
        double fitness;
    };
}
