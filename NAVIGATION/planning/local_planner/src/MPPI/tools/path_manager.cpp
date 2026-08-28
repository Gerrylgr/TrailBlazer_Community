#include "local_planner/MPPI/tools/path_manager.hpp"

namespace local_planner::mppi_core
{
    PathManager::PathManager(const PathManagerSettings & settings)
    {
        settings_ = settings;
    }

    bool PathManager::setPath(const nav_msgs::msg::Path & msg)
    {
        if(msg.poses.size() < 2)
            return false;

        nav_msgs::msg::Path resampled = resamplePath(msg);

        if (resampled.poses.size() < 2)
            return false;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            global_path_ = std::move(resampled);
            progress_index_ = 0;
        }

        return true;
    }

    LocalPathResult PathManager::updateAndBuildLocalPath(const Pose2D & robot_pose)
    {
        LocalPathResult result;
        // 只需要加一次锁，保证整个过程状态一致
        std::lock_guard<std::mutex> lock(mutex_);

        result.frame_id = global_path_.header.frame_id;

        result.progress_index = progress_index_;
        result.goal_reached = false;

        if (global_path_.poses.size() < 2)
        {
            return result;
        }

        // // ----------------- 判断是否到达终点 -----------------
        // const auto &goal = global_path_.poses.back();
        // const double distance_to_goal = std::hypot(
        //     (goal.pose.position.x - static_cast<double>(robot_pose.x)),
        //     (goal.pose.position.y - static_cast<double>(robot_pose.y)));
        
        // result.goal_reached = (distance_to_goal <= settings_.goal_tolerance);

        // // ----------------- 构建局部路径 -----------------
        const std::size_t last_index = global_path_.poses.size() - 1;

        const std::size_t search_begin = std::min(progress_index_, last_index);
        const std::size_t search_end =
            search_begin +
            std::min(
                settings_.nearest_search_window,
                last_index - search_begin);
        const std::size_t closest = findClosestIndex(robot_pose, search_begin, search_end);

        progress_index_ = std::max(progress_index_, closest);
        result.progress_index = progress_index_;

        const auto & goal = global_path_.poses.back().pose.position;

        const double distance_to_goal = std::hypot(
            goal.x - static_cast<double>(robot_pose.x),
            goal.y - static_cast<double>(robot_pose.y));

        const bool progress_near_end = progress_index_ >= last_index - 1;

        // 到达终点需要同时：距离足够近、progress_index_ >= last_index - 1
        result.goal_reached = progress_near_end && distance_to_goal <= settings_.goal_tolerance;

        if (result.goal_reached)
        {
            return result;
        }

        double accumulated_distance = 0.0;
        const std::size_t local_begin = std::min(progress_index_, last_index - 1);
        for (std::size_t i = local_begin; i < global_path_.poses.size(); ++i)
        {
            const auto & pose = global_path_.poses[i].pose;
            result.path.poses.push_back(Pose2D{
                static_cast<float>(pose.position.x),
                static_cast<float>(pose.position.y),
                poseYaw(pose)
            });

            if (i + 1 >= global_path_.poses.size())
            {
                break;
            }

            const auto & next = global_path_.poses[i + 1].pose;
            accumulated_distance += std::hypot(
                next.position.x - pose.position.x,
                next.position.y - pose.position.y);

            if (accumulated_distance >= settings_.local_path_horizon)
            {
                // 把达到 horizon 的那个点也加入
                result.path.poses.push_back(Pose2D{
                    static_cast<float>(next.position.x),
                    static_cast<float>(next.position.y),
                    poseYaw(next)
                });
                break;
            }
        }

        return result;
    }

    nav_msgs::msg::Path PathManager::resamplePath(const nav_msgs::msg::Path & input) const
    {
        constexpr double kEps = 1e-8;

        nav_msgs::msg::Path output;
        output.header = input.header;

        const double resolution = settings_.path_resolution;

        if (input.poses.empty() || !std::isfinite(resolution) || resolution <= 0.0)
        {
            return output;
        }

        std::vector<geometry_msgs::msg::PoseStamped> cleaned;
        cleaned.reserve(input.poses.size());

        for (const auto & pose : input.poses)
        {
            const auto & p = pose.pose.position;

            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            {
                return output;
            }

            if (cleaned.empty())
            {
                cleaned.push_back(pose);
                continue;
            }

            const auto & last = cleaned.back().pose.position;
            const double distance = std::hypot(p.x - last.x, p.y - last.y);

            if (distance > kEps)                // 不推入重复点
            {
                cleaned.push_back(pose);
            }
            else
            {
                // 同位置时保留较新的姿态，尤其是最终目标朝向
                cleaned.back() = pose;
            }
        }

        if (cleaned.empty())
        {
            return output;
        }

        if (cleaned.size() == 1)
        {
            auto pose = cleaned.front();
            pose.header = output.header;
            output.poses.push_back(std::move(pose));
            return output;
        }

        std::vector<double> cumulative(cleaned.size(), 0.0);        // 存储累积长度

        for (std::size_t i = 1; i < cleaned.size(); ++i)
        {
            const auto & previous = cleaned[i - 1].pose.position;
            const auto & current = cleaned[i].pose.position;

            cumulative[i] =
                cumulative[i - 1] +
                std::hypot(
                    current.x - previous.x,
                    current.y - previous.y);
        }

        const double total_length = cumulative.back();

        if (total_length <= kEps)
        {
            auto pose = cleaned.back();
            pose.header = output.header;
            output.poses.push_back(std::move(pose));
            return output;
        }

        auto first = cleaned.front();
        first.header = output.header;
        output.poses.push_back(std::move(first));

        std::size_t segment = 0;

        for (std::size_t sample_index = 1; ; ++sample_index)
        {
            const double target = static_cast<double>(sample_index) * resolution;

            if (target >= total_length - kEps)
            {
                break;
            }

            // cumulative[segment + 1] < target 代表这一段的弧长小于 resolution，需要 ++segment 
            while (segment + 1 < cleaned.size() - 1 && cumulative[segment + 1] < target)
            {
                ++segment;
            }

            const double segment_start = cumulative[segment];
            const double segment_length = cumulative[segment + 1] - segment_start;

            if (segment_length <= kEps)
            {
                continue;
            }

            const double ratio = (target - segment_start) / segment_length;

            const auto & start = cleaned[segment].pose;
            const auto & end = cleaned[segment + 1].pose;

            geometry_msgs::msg::PoseStamped sample;
            sample.header = output.header;

            sample.pose.position.x = start.position.x + ratio * (end.position.x - start.position.x);
            sample.pose.position.y = start.position.y + ratio * (end.position.y - start.position.y);
            sample.pose.position.z = start.position.z + ratio * (end.position.z - start.position.z);

            const double start_yaw = tf2::getYaw(start.orientation);
            const double end_yaw = tf2::getYaw(end.orientation);
            const double yaw_delta = std::atan2(std::sin(end_yaw - start_yaw), std::cos(end_yaw - start_yaw));

            const double yaw = std::atan2(
                std::sin(start_yaw + ratio * yaw_delta), 
                std::cos(start_yaw + ratio * yaw_delta));

            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, yaw);
            sample.pose.orientation = tf2::toMsg(quaternion);

            output.poses.push_back(std::move(sample));
        }

        auto final_pose = cleaned.back();
        final_pose.header = output.header;
        output.poses.push_back(std::move(final_pose));

        return output;
    }

    std::size_t PathManager::findClosestIndex(
                const Pose2D & robot_pose,
                std::size_t begin,
                std::size_t end) const
    {
        const size_t path_size = global_path_.poses.size();
        if (path_size == 0) return 0;

        const size_t begin_idx = std::min(begin, path_size - 1);
        const size_t end_idx = std::min(end, path_size - 1);

        const double x = robot_pose.x;
        const double y = robot_pose.y;
        // const double yaw = robot_pose.yaw;

        size_t best_idx = std::min(begin_idx, end_idx);
        double best_dist_sq = std::numeric_limits<double>::max();

        for (size_t idx = begin_idx; idx <= end_idx; ++idx) 
        {
            const auto& pt = global_path_.poses[idx].pose.position;
            const double dx = static_cast<double>(pt.x) - x;
            const double dy = static_cast<double>(pt.y) - y;
            const double dist_sq = dx * dx + dy * dy;

            // 直接选择最近点
            if (dist_sq < best_dist_sq)
            {
                best_dist_sq = dist_sq;
                best_idx = idx;
            }
        }
        return best_idx;
    }

    float PathManager::poseYaw(const geometry_msgs::msg::Pose & pose)
    {
        return static_cast<float>(tf2::getYaw(pose.orientation));
    }

    std::string PathManager::frameId() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return global_path_.header.frame_id;
    }

    void PathManager::clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        global_path_ = nav_msgs::msg::Path();
        progress_index_ = 0;
    }
}