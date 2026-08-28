/*
*   轨迹追踪惩罚计算
*   优化目标：让预测轨迹的终点 尽可能接近路径上的前瞻点
*/
#include "local_planner/MPPI/critics/path_follow_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace local_planner::mppi_core
{

    PathFollowCritic::PathFollowCritic(const PathFollowCriticSettings & settings) : settings_(settings)
    {
        if (settings_.cost_weight < 0.0f)
        {
            throw std::invalid_argument(
                "PathFollowCritic: cost_weight must be non-negative.");
        }

        if (settings_.max_endpoint_path_distance < 0.0f)
        {
            throw std::invalid_argument(
                "PathFollowCritic: max_endpoint_path_distance must be non-negative.");
        }

        if (settings_.cost_power == 0)
        {
            settings_.cost_power = 1;
        }
    }

    std::optional<std::size_t> PathFollowCritic::findFurthestReachedIndex(const CriticData & data) const
    {
        const std::size_t path_size = data.path.x.size();
        const std::size_t batch_size = data.trajectories.x.shape()[0];
        const std::size_t time_steps = data.trajectories.x.shape()[1];

        if (path_size == 0 || batch_size == 0 || time_steps == 0)
        {
            return std::nullopt;
        }

        const std::size_t final_step = time_steps - 1;
        const float max_distance_sq =
            settings_.max_endpoint_path_distance * settings_.max_endpoint_path_distance;

        std::optional<std::size_t> accepted_furthest;
        std::optional<std::size_t> fallback_furthest;

        const auto update_furthest =
            [](std::optional<std::size_t> & value, std::size_t index)
            {
                value = value ? std::max(*value, index) : index;
            };

        for (std::size_t batch = 0; batch < batch_size; ++batch)
        {
            // 要评估的候选轨迹点
            const float x = data.trajectories.x(batch, final_step);
            const float y = data.trajectories.y(batch, final_step);

            if (!std::isfinite(x) || !std::isfinite(y))
            {
                continue;
            }

            float best_distance_sq = std::numeric_limits<float>::infinity();
            std::size_t closest_index = 0;

            // 遍历目标路径的所有点，找到距离轨迹点最近的那个的索引
            for (std::size_t i = 0; i < path_size; ++i)
            {
                const float dx = x - data.path.x(i);
                const float dy = y - data.path.y(i);
                const float distance_sq = dx * dx + dy * dy;

                if (distance_sq < best_distance_sq)
                {
                    best_distance_sq = distance_sq;
                    closest_index = i;
                }
            }

            update_furthest(fallback_furthest, closest_index);      // 始终更新 fallback_furthest

            if (best_distance_sq <= max_distance_sq)        // 距离小与 max_distance_sq 才更新 accepted_furthest
            {
                update_furthest(accepted_furthest, closest_index);
            }
        }

        // 如果存在至少一条轨迹是成功贴合路径的（accepted_furthest 有值），则返回这些有效轨迹中走到的最远索引
        // 如果所有轨迹都跑偏了（accepted_furthest 为空），则返回 fallback_furthest（即几何上最远能触达的路径索引）
        return accepted_furthest ? accepted_furthest : fallback_furthest;
    }

    /*
    *   findTargetIndex 找到目标点索引，之后找到目标点坐标
    *   计算所有路径最后一个点坐标与目标点距离，乘上权重得到这条路径的代价
    */
    void PathFollowCritic::score(CriticData & data) const
    {
        if (!settings_.enabled)
        {
            return;
        }

        const std::size_t path_size = data.path.x.size();
        const std::size_t batch_size = data.trajectories.x.shape()[0];          // 轨迹数目
        const std::size_t time_steps = data.trajectories.x.shape()[1];          // 时间步数目

        if (data.path.y.size() != path_size ||
            data.trajectories.y.shape() != data.trajectories.x.shape() ||
            data.costs.size() != batch_size)
        {
            throw std::runtime_error(
                "PathFollowCritic: inconsistent tensor dimensions.");
        }

        const auto furthest = findFurthestReachedIndex(data);           // 找到最远点索引

        if (!furthest)
        {
            for (std::size_t batch = 0; batch < data.costs.size(); ++batch)
            {
                data.costs(batch) += settings_.invalid_cost;
            }
            return;
        }

        const std::size_t last_path_index = path_size - 1;

        // 要追踪的 target_index
        const std::size_t target_index =
            *furthest +
            std::min(
                settings_.offset_from_furthest,
                last_path_index - *furthest);

        const float target_x = data.path.x(target_index);
        const float target_y = data.path.y(target_index);
        const std::size_t final_step = time_steps - 1;

        for (std::size_t batch = 0; batch < batch_size; ++batch)
        {
            // 预测路径的点
            const float trajectory_x = data.trajectories.x(batch, final_step);
            const float trajectory_y = data.trajectories.y(batch, final_step);

            if (!std::isfinite(trajectory_x) || !std::isfinite(trajectory_y))
            {
                data.costs(batch) += settings_.invalid_cost;

                continue;
            }

            const float distance = std::hypot(trajectory_x - target_x, trajectory_y - target_y);
            const float weighted_cost = settings_.cost_weight * distance;

            const float final_cost =
                settings_.cost_power > 1 ?
                std::pow(
                    weighted_cost,
                    static_cast<float>(settings_.cost_power)) : weighted_cost;

            data.costs(batch) += final_cost;
        }
    }

}  // namespace local_planner::mppi_core