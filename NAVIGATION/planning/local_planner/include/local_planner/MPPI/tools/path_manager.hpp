/*
*   Path-Manager
*   usage in mppi_planner.cpp:
*       path_manager_->clear()
*       path_manager_->setPath(*msg) (in path_callback)
*       path_manager_->isGoalReached(robot_pose) (in timer_callback)
*       auto local_path = path_manager_->buildLocalPath(robot_pose) (in timer_callback)
*/
#ifndef LOCAL_PLANNER__MPPI__TOOLS__PATH_MANAGER_HPP_
#define LOCAL_PLANNER__MPPI__TOOLS__PATH_MANAGER_HPP_

#include "local_planner/MPPI/core/types.hpp"

#include <nav_msgs/msg/path.hpp>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cstddef>
#include <mutex>
#include <string>

namespace local_planner::mppi_core
{

    struct PathManagerSettings
    {
        double local_path_horizon{3.0};             // 局部规划视野(单位m)
        double path_resolution{0.10};
        double goal_tolerance{0.15};

        std::size_t nearest_search_window{100};
    };

    struct LocalPathResult
    {
        ReferencePath path;
        std::string frame_id;
        std::size_t progress_index;
        bool goal_reached;
    };

    class PathManager
    {
        public:
            explicit PathManager(const PathManagerSettings & settings);

            bool setPath(const nav_msgs::msg::Path & msg);          // 对输入路径重采样存入 global_path_，重置 progress_index_

            // 更新进度并构建局部路径，同时判断是否到达终点
            LocalPathResult updateAndBuildLocalPath(const Pose2D & robot_pose);

            std::string frameId() const;

            void clear();

        private:
            // 严格按照 path_resolution 重采样
            nav_msgs::msg::Path resamplePath(const nav_msgs::msg::Path & input) const;  

            // 输入当前位置、progress_index_、progress_index_ + nearest_search_window（当前处理的路段的起始终止索引）
            // 返回距离当前位置最近的路点的索引
            std::size_t findClosestIndex(
                const Pose2D & robot_pose,
                std::size_t begin,
                std::size_t end) const;

            static float poseYaw(const geometry_msgs::msg::Pose & pose);        // get yaw from orientation

        private:
            PathManagerSettings settings_;

            nav_msgs::msg::Path global_path_;
            std::size_t progress_index_{0};

            mutable std::mutex mutex_;
    };

}  // namespace local_planner::mppi_core

#endif