/*
* 速度超限惩罚计算
*/
#include "local_planner/MPPI/critics/constraint_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace local_planner::mppi_core
{

ConstraintCritic::ConstraintCritic(const ConstraintCriticSettings & settings): settings_(settings)
{
  if (settings_.vx_min > settings_.vx_max)
  {
    throw std::invalid_argument("ConstraintCritic: vx_min must not be greater than vx_max.");
  }

  if (settings_.wz_max < 0.0f)
  {
    throw std::invalid_argument("ConstraintCritic: wz_max must be non-negative.");
  }

  if (settings_.cost_weight < 0.0f)
  {
    throw std::invalid_argument("ConstraintCritic: cost_weight must be non-negative.");
  }

  if (settings_.linear_violation_weight < 0.0f || settings_.angular_violation_weight < 0.0f)
  {
    throw std::invalid_argument("ConstraintCritic: violation weights must be non-negative.");
  }

  // power=0 会让所有非零代价变成1，不符合这里的用途
  if (settings_.cost_power == 0)
  {
    settings_.cost_power = 1;
  }
}

/*
* 通过 CriticData 中的 state.wz/state.wz 对速度超限的轨迹作出惩罚
*/
void ConstraintCritic::score(CriticData & data) const
{
  if (!settings_.enabled)
  {
    return;
  }

  const std::size_t batch_size = data.state.vx.shape()[0];
  const std::size_t time_steps = data.state.vx.shape()[1];

  // 检查张量尺寸是否一致
  if (data.state.wz.shape()[0] != batch_size || data.state.wz.shape()[1] != time_steps)
  {
    throw std::runtime_error("ConstraintCritic: state.vx and state.wz shapes do not match.");
  }

  if (data.costs.size() != batch_size)
  {
    throw std::runtime_error("ConstraintCritic: costs size does not match batch size.");
  }

  const float dt = std::max(data.model_dt, 0.0f);

  // 遍历所有轨迹
  for (std::size_t batch = 0; batch < batch_size; ++batch)
  {
    float accumulated_violation = 0.0f;
    bool invalid_state = false;

    // 遍历所有时间步
    for (std::size_t t = 0; t < time_steps; ++t)
    {
      const float vx = data.state.vx(batch, t);
      const float wz = data.state.wz(batch, t);

      if (!std::isfinite(vx) || !std::isfinite(wz))
      {
        invalid_state = true;
        break;
      }

      // vx > vx_max 时的超限量
      const float vx_above = std::max(vx - settings_.vx_max, 0.0f);
      // vx < vx_min 时的超限量
      const float vx_below = std::max(settings_.vx_min - vx, 0.0f);
      // |wz| > wz_max 时的超限量
      const float wz_above = std::max(std::abs(wz) - settings_.wz_max, 0.0f);

      const float linear_violation = vx_above + vx_below;
      const float angular_violation = wz_above;

      accumulated_violation +=
          (
              settings_.linear_violation_weight * linear_violation +
              settings_.angular_violation_weight * angular_violation
          ) * dt;
    }

    if (invalid_state)
    {
      data.costs(batch) += settings_.invalid_state_cost;
      continue;
    }

    const float weighted_cost = settings_.cost_weight * accumulated_violation;        // 这条轨迹的整体代价

    float final_cost = weighted_cost;

    if (settings_.cost_power > 1)
    {
        final_cost = std::pow(weighted_cost, static_cast<float>(settings_.cost_power));       // 对代价取 cost_power 次幂，放大高违反量轨迹的代价
    }

    data.costs(batch) += final_cost;
  }
}

}  // namespace local_planner::mppi_core