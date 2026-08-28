/*
工具函数的实现，与ROS插件无关
*/
#pragma once

#include <vector>
#include <Eigen/Dense>
#include <set>
#include <cmath>
#include <algorithm>

namespace global_planner
{

    // =====================================================
    // 计算路径上所有线段向量的夹角 (弧度)
    // =====================================================
    std::vector<double> compute_angles(const std::vector<Eigen::Vector2d>& points);

    // =====================================================
    // 路径简化：保留拐角
    // =====================================================
    std::vector<Eigen::Vector2d> simplify_path(
        const std::vector<Eigen::Vector2d>& points,
        double corner_deg = 30.0,
        int corner_dilate = 1
    );

} // namespace global_planner
