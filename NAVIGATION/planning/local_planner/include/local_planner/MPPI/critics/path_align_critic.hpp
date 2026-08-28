#ifndef LOCAL_PLANNER__MPPI__CRITICS__PATH_ALIGN_CRITIC_HPP_
#define LOCAL_PLANNER__MPPI__CRITICS__PATH_ALIGN_CRITIC_HPP_

#include "local_planner/MPPI/core/critic_function.hpp"

#include <cstddef>

namespace local_planner::mppi_core
{

    struct PathAlignCriticSettings
    {
        bool enabled{true};

        float cost_weight{2.0f};
        unsigned int cost_power{1};

        std::size_t trajectory_step{2};             // 计算平均距离时一次越过的点数

        float invalid_cost{1.0e6f};
    };

    class PathAlignCritic final : public CriticFunction
    {
    public:
        explicit PathAlignCritic(const PathAlignCriticSettings & settings);

        void score(CriticData & data) const override;

    private:
        PathAlignCriticSettings settings_;
    };

}  // namespace local_planner::mppi_core

#endif