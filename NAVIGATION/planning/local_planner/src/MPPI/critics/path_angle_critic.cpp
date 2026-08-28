#include "local_planner/MPPI/critics/path_angle_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace local_planner::mppi_core
{

    PathAngleCritic::PathAngleCritic(const PathAngleCriticSettings & settings) : settings_(settings)
    {
        if (settings_.cost_weight < 0.0f)
        {
            throw std::invalid_argument(
                "PathAngleCritic: cost_weight must be non-negative.");
        }

        if (settings_.trajectory_step == 0)
        {
            settings_.trajectory_step = 1;
        }

        if (settings_.cost_power == 0)
        {
            settings_.cost_power = 1;
        }
    }

    std::size_t PathAngleCritic::findClosestPathIndex(const TensorPath & path, float x, float y) const
    {
        std::size_t best_index = 0;

        float best_distance_sq = std::numeric_limits<float>::infinity();

        for (std::size_t i = 0; i < path.x.size(); ++i)
        {
            const float dx = x - path.x(i);
            const float dy = y - path.y(i);

            const float distance_sq = dx * dx + dy * dy;

            if (distance_sq < best_distance_sq)
            {
                best_distance_sq = distance_sq;
                best_index = i;
            }
        }

        return best_index;
    }

    // 计算 index 点前方路径的朝向
    float PathAngleCritic::calculatePathTangentYaw(const TensorPath & path, std::size_t index) const
    {
        const std::size_t path_size = path.x.size();

        if (path_size < 2)
        {
            return path_size == 1 ? path.yaw(0) : 0.0f;
        }

        std::size_t begin_index;
        std::size_t end_index;

        if (index + 1 < path_size)
        {
            begin_index = index;
            end_index = index + 1;
        }
        else
        {
            begin_index = index - 1;
            end_index = index;
        }

        const float dx = path.x(end_index) - path.x(begin_index);
        const float dy = path.y(end_index) - path.y(begin_index);

        if (std::hypot(dx, dy) < 1.0e-6f)
        {
            return path.yaw(index);
        }

        return std::atan2(dy, dx);
    }

    void PathAngleCritic::score(CriticData & data) const
    {
        if (!settings_.enabled)
        {
            return;
        }

        const std::size_t batch_size = data.trajectories.x.shape()[0];
        const std::size_t time_steps = data.trajectories.x.shape()[1];

        const std::size_t path_size = data.path.x.size();

        if (path_size < 2 || batch_size == 0 || time_steps == 0)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("path_angle_critic"),
                "path_size/batch_size/time_steps invalid, costs will be infinite!!!"
            );
            for (std::size_t batch = 0; batch < data.costs.size(); ++batch)
            {
                data.costs(batch) += settings_.invalid_cost;
            }

            return;
        }

        // 遍历所有路径
        for (std::size_t batch = 0; batch < batch_size; ++batch)
        {
            float accumulated_angle_cost = 0.0f;
            std::size_t sample_count = 0;

            // 遍历所有时间步
            for (std::size_t t = 0; t < time_steps; t += settings_.trajectory_step)
            {
                const float trajectory_x = data.trajectories.x(batch, t);
                const float trajectory_y = data.trajectories.y(batch, t);
                const float trajectory_yaw = data.trajectories.yaw(batch, t);       // （运动学）推算出的的车头朝向

                if (!std::isfinite(trajectory_x) || !std::isfinite(trajectory_y) || !std::isfinite(trajectory_yaw))
                {
                    accumulated_angle_cost = settings_.invalid_cost;
                    sample_count = 1;
                    break;
                }

                const std::size_t closest_index = findClosestPathIndex(data.path, trajectory_x, trajectory_y);
                const float target_yaw = calculatePathTangentYaw(data.path, closest_index);         // 目标 path 对应点处要求的车头朝向

                const float yaw_error = std::atan2(std::sin(trajectory_yaw - target_yaw), std::cos(trajectory_yaw - target_yaw));

                // 0°  -> 0
                // 90° -> 1
                // 180°-> 2
                const float angle_cost = 1.0f - std::cos(yaw_error);

                accumulated_angle_cost += angle_cost;

                ++sample_count;
            }

            if (sample_count == 0)
            {
                continue;
            }

            const float mean_angle_cost = accumulated_angle_cost / static_cast<float>(sample_count);

            const float weighted_cost = settings_.cost_weight * mean_angle_cost;

            const float final_cost = settings_.cost_power > 1 ? std::pow(weighted_cost, static_cast<float>(settings_.cost_power)) : weighted_cost;

            data.costs(batch) += final_cost;
        }
    }

}  // namespace local_planner::mppi_core