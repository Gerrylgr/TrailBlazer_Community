#ifndef LOCAL_PLANNER__MPPI__CORE__CRITIC_DATA_HPP_
#define LOCAL_PLANNER__MPPI__CORE__CRITIC_DATA_HPP_

#include "local_planner/MPPI/core/types.hpp"
#include "field_map_builder/utility/esdf_query.hpp"

#include <memory>

namespace local_planner::mppi_core
{

    struct CriticData
    {
        const State & state;
        const Trajectories & trajectories;          // 一大堆预测的候选轨迹
        const TensorPath & path;

        xt::xtensor<float, 1> & costs;              // 每一条候选轨迹的总代价值

        float model_dt;                 // 时间步长

        const std::shared_ptr<const field_map_builder::EsdfMapSnapshot> esdf;
    };

}  // namespace local_planner::mppi_core

#endif