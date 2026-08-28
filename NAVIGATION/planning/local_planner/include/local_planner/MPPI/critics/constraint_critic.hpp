#ifndef LOCAL_PLANNER__MPPI__CRITICS__CONSTRAINT_CRITIC_HPP_
#define LOCAL_PLANNER__MPPI__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "local_planner/MPPI/core/critic_function.hpp"

#include <cstddef>

namespace local_planner::mppi_core
{

  // 速度超限代价计算的参数
  struct ConstraintCriticSettings
  {
    bool enabled{true};     // 是否启用

    // 整体代价权重
    float cost_weight{4.0f};

    // 最终代价幂次，第一版建议设为 1
    unsigned int cost_power{1};           // 对代价取 cost_power 次幂，放大高违反量轨迹的代价

    // 差速底盘线速度约束
    float vx_min{0.0f};                 // 最小线速度为0代表会惩罚倒车
    float vx_max{0.6f};

    // 差速底盘角速度绝对值上限
    float wz_max{0.8f};

    // 线速度/角速度代价权重
    float linear_violation_weight{1.0f};
    float angular_violation_weight{1.0f};

    // 速度出现 NaN/Inf 时直接施加大代价
    float invalid_state_cost{1.0e6f};
  };

  class ConstraintCritic final : public CriticFunction
  {
  public:
    explicit ConstraintCritic(const ConstraintCriticSettings & settings);

    void score(CriticData & data) const override;

    const ConstraintCriticSettings & settings() const
    {
      return settings_;
    }

  private:
    ConstraintCriticSettings settings_;
  };

}  // namespace local_planner::mppi_core

#endif