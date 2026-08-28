#ifndef LOCAL_PLANNER__MPPI__CRITICS__PATH_TRACKING_CRITIC_HPP_
#define LOCAL_PLANNER__MPPI__CRITICS__PATH_TRACKING_CRITIC_HPP_

#include "local_planner/MPPI/core/critic_function.hpp"

#include <cstddef>

namespace local_planner::mppi_core
{

struct PathTrackingCriticSettings
{
    bool enabled{true};

    float cost_weight{1.0f};
    float lateral_error_weight{18.0f};
    float heading_error_weight{6.0f};
    float speed_error_weight{16.0f};

    std::size_t trajectory_step{1};
    float time_discount{0.98f};

    float max_linear_speed{0.80f};
    float min_curve_speed{0.06f};

    float curvature_gain{1.20f};
    float lateral_error_speed_gain{2.50f};
    float heading_error_speed_gain{1.00f};

    // ---- 横向阻尼 ----
    // lateral_velocity_weight 惩罚的是“实际横向速度 - 期望收敛横向速度”，而不是阻止机器人产生一切横向速度。
    float lateral_velocity_weight{4.5f};            // 侧向速度惩罚
    float yaw_rate_error_weight{2.0f};              // 角速度偏离惩罚
    float yaw_rate_gate_distance{0.20f};

    // ---- 收敛航向 ----
    bool use_convergence_heading{true};
    float convergence_gain{1.0f};
    float softening_speed{0.20f};
    float max_convergence_angle{0.45f};

    // 参考角速度 = vx * curvature - yaw_rate_heading_gain * heading_tracking_error。
    // 第一项是路径曲率前馈，第二项使纠偏角速度在接近目标航向时平滑回落。
    float yaw_rate_heading_gain{1.5f};

    std::size_t curvature_lookahead{3};

    // 每个预测点只在上一次投影线段之后的有限窗口内搜索，避免在交叉路径上
    // 跳到很远的后续分支，同时降低 PathTrackingCritic 的计算量。
    std::size_t projection_search_window{12};

    float invalid_cost{1.0e6f};
};

class PathTrackingCritic final : public CriticFunction
{
public:
    explicit PathTrackingCritic(const PathTrackingCriticSettings & settings);

    void score(CriticData & data) const override;

private:
    struct Projection
    {
        std::size_t segment{0};             // 哪一段折线
        float distance_squared{0.0f};       // 点距离投影折线的最短距离
        float tangent_yaw{0.0f};            // 投影折线的朝向
        float signed_lateral{0.0f};         // 路径前进方向左侧为正
    };

    // 把轨迹点投影到一段折线上，返回投影信息
    Projection findForwardProjection(
        const TensorPath & path,
        float x,
        float y,
        std::size_t first_segment) const;

    // 根据 curvature_lookahead 估算折线所在位置的曲率
    float estimateCurvature(const TensorPath & path, std::size_t segment) const;
    static float segmentYaw(const TensorPath & path, std::size_t segment);
    static float normalizedAngle(float angle);

    PathTrackingCriticSettings settings_;
};

}  // namespace local_planner::mppi_core

#endif  // LOCAL_PLANNER__MPPI__CRITICS__PATH_TRACKING_CRITIC_HPP_
