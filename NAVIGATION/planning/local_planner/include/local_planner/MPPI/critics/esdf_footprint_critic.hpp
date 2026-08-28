#ifndef LOCAL_PLANNER__MPPI__CRITICS__ESDF_FOOTPRINT_CRITIC_HPP_
#define LOCAL_PLANNER__MPPI__CRITICS__ESDF_FOOTPRINT_CRITIC_HPP_

#include "local_planner/MPPI/core/critic_function.hpp"

#include <cstddef>
#include <vector>

namespace local_planner::mppi_core
{

    struct EsdfFootprintCriticSettings
    {
        bool enabled{true};

        float safe_distance{0.50f};                 // 产生代价的安全距离
        float collision_distance{0.03f};            // 认为发生碰撞的距离

        float repulsion_weight{20.0f};
        float collision_cost{1.0e6f};

        std::size_t trajectory_step{2};

        std::vector<Point2D> footprint_samples;             // 车体轮廓 footprint 外参
    };

    class EsdfFootprintCritic final
        : public CriticFunction
    {
    public:
        explicit EsdfFootprintCritic(const EsdfFootprintCriticSettings & settings);

        void score(CriticData & data) const override;

    private:
        EsdfFootprintCriticSettings settings_;
    };

}  // namespace local_planner::mppi_core

#endif