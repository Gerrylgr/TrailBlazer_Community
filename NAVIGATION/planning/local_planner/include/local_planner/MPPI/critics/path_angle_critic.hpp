#ifndef LOCAL_PLANNER__MPPI__CRITICS__PATH_ANGLE_CRITIC_HPP_
#define LOCAL_PLANNER__MPPI__CRITICS__PATH_ANGLE_CRITIC_HPP_

#include "local_planner/MPPI/core/critic_function.hpp"

#include <cstddef>

namespace local_planner::mppi_core
{

    struct PathAngleCriticSettings
    {
        bool enabled{true};

        float cost_weight{8.0f};
        unsigned int cost_power{1};

        // 每隔多少个预测时间步检查一次
        std::size_t trajectory_step{2};

        float invalid_cost{1.0e6f};
    };

    class PathAngleCritic final : public CriticFunction
    {
    public:
        explicit PathAngleCritic(const PathAngleCriticSettings & settings);

        void score(CriticData & data) const override;

    private:
        std::size_t findClosestPathIndex(const TensorPath & path, float x, float y) const;

        float calculatePathTangentYaw(const TensorPath & path, std::size_t index) const;

    private:
        PathAngleCriticSettings settings_;
    };

}  // namespace local_planner::mppi_core

#endif