#include "local_planner/MPPI/critics/esdf_footprint_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace local_planner::mppi_core
{

    EsdfFootprintCritic::EsdfFootprintCritic(const EsdfFootprintCriticSettings & settings) : settings_(settings)
    {
        if (settings_.safe_distance < settings_.collision_distance)
        {
            throw std::invalid_argument(
                "EsdfFootprintCritic: safe_distance must be "
                "greater than collision_distance.");
        }

        if (settings_.trajectory_step == 0)
        {
            settings_.trajectory_step = 1;
        }

        if (settings_.footprint_samples.empty())
        {
            throw std::invalid_argument("EsdfFootprintCritic: footprint samples are empty.");
        }
    }

    void EsdfFootprintCritic::score(CriticData & data) const
    {
        if (!settings_.enabled)
        {
            return;
        }

        // 没有获取到 ESDF 数据全部按照碰撞处理
        if (!data.esdf)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("esdf_footprint_critic"),
                "No ESDF data yet, costs will be infinite!!!"
            );
            for (std::size_t batch = 0; batch < data.costs.size(); ++batch)
            {
                data.costs(batch) += settings_.collision_cost;
            }

            return;
        }

        const std::size_t batch_size = data.trajectories.x.shape()[0];
        const std::size_t time_steps = data.trajectories.x.shape()[1];

        // 遍历所有预测的路径
        for (std::size_t batch = 0; batch < batch_size; ++batch)
        {
            float trajectory_cost = 0.0f;
            bool collision = false;

            // 遍历所有的时间步
            for (std::size_t t = 0; t < time_steps; t += settings_.trajectory_step)
            {
                const float robot_x = data.trajectories.x(batch, t);
                const float robot_y = data.trajectories.y(batch, t);
                const float robot_yaw = data.trajectories.yaw(batch, t);

                const float cos_yaw = std::cos(robot_yaw);
                const float sin_yaw = std::sin(robot_yaw);

                float minimum_distance = std::numeric_limits<float>::infinity();

                // 遍历车体轮廓做碰撞检测
                for (const auto & sample : settings_.footprint_samples)
                {
                    const float world_x = robot_x + cos_yaw * sample.x - sin_yaw * sample.y;
                    const float world_y = robot_y + sin_yaw * sample.x + cos_yaw * sample.y;

                    float distance = 0.0;
                    if (!data.esdf->distance_bilinear(world_x, world_y, distance) || !std::isfinite(distance))
                    {
                        collision = true;
                        break;
                    }

                    minimum_distance = std::min(minimum_distance, distance);
                }

                if (collision || minimum_distance <= settings_.collision_distance)
                {
                    collision = true;
                    break;
                }

                if (minimum_distance < settings_.safe_distance)
                {
                    const float error = settings_.safe_distance - minimum_distance;
                    trajectory_cost += settings_.repulsion_weight * error * error;
                }
            }

            if (collision)
            {
                data.costs(batch) += settings_.collision_cost;
            }
            else
            {
                data.costs(batch) += trajectory_cost;
            }
        }
    }

}  // namespace local_planner::mppi_core