#ifndef LOCAL_PLANNER__MPPI__CRITICS__PATH_FOLLOW_CRITIC_HPP_
#define LOCAL_PLANNER__MPPI__CRITICS__PATH_FOLLOW_CRITIC_HPP_

#include "local_planner/MPPI/core/critic_function.hpp"

#include <cstddef>
#include <optional>

namespace local_planner::mppi_core
{

    struct PathFollowCriticSettings
    {
        bool enabled{true};

        float cost_weight{5.0f};
        unsigned int cost_power{1};

        // 严格重采样后，7 个点约等于 0.7m
        std::size_t offset_from_furthest{7};

        // （预测的路径点）距离目标路径点 max_endpoint_path_distance 以内算做到达
        float max_endpoint_path_distance{0.75f};

        float invalid_cost{1.0e6f};
    };

    class PathFollowCritic final : public CriticFunction
    {
    public:
        explicit PathFollowCritic(const PathFollowCriticSettings & settings);

        void score(CriticData & data) const override;

    private:
        std::optional<std::size_t> findFurthestReachedIndex(const CriticData & data) const;

        PathFollowCriticSettings settings_;
    };

}  // namespace local_planner::mppi_core

#endif