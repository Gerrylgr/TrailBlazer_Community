/*
*   目标路径距离代价
*   计算各个预测路径与目标路径的平均距离作为代价,从而让预测路径更倾向于靠近目标路径
*/
#include "local_planner/MPPI/critics/path_align_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace local_planner::mppi_core
{

PathAlignCritic::PathAlignCritic(const PathAlignCriticSettings & settings) : settings_(settings)
{
    if (settings_.cost_weight < 0.0f)
    {
        throw std::invalid_argument("PathAlignCritic: cost_weight must be non-negative.");
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

void PathAlignCritic::score(CriticData & data) const
{
    if (!settings_.enabled)
    {
        return;
    }

    const std::size_t batch_size = data.trajectories.x.shape()[0];              // 预测的路径数目
    const std::size_t time_steps = data.trajectories.x.shape()[1];              // 时间步数目
    const std::size_t path_size = data.path.x.size();                   // 要追踪的路径的大小

    if (path_size == 0)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("path_align_critic"),
            "Tensor path is empty, costs will be infinite!!!"
        );
        for (std::size_t batch = 0; batch < data.costs.size(); ++batch)
        {
            data.costs(batch) += settings_.invalid_cost;
        }

        return;
    }

    // 遍历所有路线
    for (std::size_t batch = 0; batch < batch_size; ++batch)
    {
        float accumulated_distance = 0.0f;              // 当前路线与目标路径整体的差距
        std::size_t sample_count = 0;               // 有效路径点数目

        // 遍历所有时间步
        for (std::size_t t = 0; t < time_steps; t += settings_.trajectory_step)
        {
            const float trajectory_x =data.trajectories.x(batch, t);
            const float trajectory_y = data.trajectories.y(batch, t);

            if (!std::isfinite(trajectory_x) || !std::isfinite(trajectory_y))
            {
                accumulated_distance = settings_.invalid_cost;
                sample_count = 1;
                break;
            }

            float minimum_distance_sq = std::numeric_limits<float>::infinity();

            // 遍历目标路径
            for (std::size_t path_index = 0; path_index < path_size; ++path_index)
            {
                const float dx = trajectory_x - data.path.x(path_index);
                const float dy = trajectory_y - data.path.y(path_index);

                minimum_distance_sq = std::min(minimum_distance_sq, dx * dx + dy * dy);             // 找到当前点与目标路径上的匹配点(就是最近的)
            }

            accumulated_distance += std::sqrt(minimum_distance_sq);

            ++sample_count;
        }

        if (sample_count == 0)
        {
            continue;
        }

        const float mean_distance = accumulated_distance / static_cast<float>(sample_count);                // 当前路线与目标路线的平均距离
        const float weighted_cost = settings_.cost_weight * mean_distance;

        const float final_cost = settings_.cost_power > 1 ? std::pow(weighted_cost, static_cast<float>(settings_.cost_power)) : weighted_cost;
        data.costs(batch) += final_cost;                // 算作当前这个预测路线的代价
    }
}

}  // namespace local_planner::mppi_core