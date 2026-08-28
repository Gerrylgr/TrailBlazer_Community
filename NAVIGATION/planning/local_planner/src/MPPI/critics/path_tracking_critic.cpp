#include "local_planner/MPPI/critics/path_tracking_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace local_planner::mppi_core
{

PathTrackingCritic::PathTrackingCritic(const PathTrackingCriticSettings & settings)
    : settings_(settings)
{
    if (settings_.cost_weight < 0.0f ||
        settings_.lateral_error_weight < 0.0f ||
        settings_.heading_error_weight < 0.0f ||
        settings_.speed_error_weight < 0.0f)
    {
        throw std::invalid_argument("PathTrackingCritic: weights must be non-negative.");
    }

    if (settings_.trajectory_step == 0)
    {
        settings_.trajectory_step = 1;
    }

    if (settings_.time_discount <= 0.0f || settings_.time_discount > 1.0f)
    {
        throw std::invalid_argument("PathTrackingCritic: time_discount must be in (0, 1].");
    }

    if (settings_.max_linear_speed <= 0.0f ||
        settings_.min_curve_speed < 0.0f ||
        settings_.min_curve_speed > settings_.max_linear_speed)
    {
        throw std::invalid_argument("PathTrackingCritic: invalid speed limits.");
    }

    if (settings_.lateral_velocity_weight < 0.0f ||
        settings_.yaw_rate_error_weight < 0.0f ||
        settings_.convergence_gain < 0.0f ||
        settings_.yaw_rate_heading_gain < 0.0f)
    {
        throw std::invalid_argument("PathTrackingCritic: damping weights must be non-negative.");
    }

    if (settings_.yaw_rate_gate_distance <= 0.0f ||
        settings_.softening_speed <= 0.0f ||
        settings_.max_convergence_angle <= 0.0f)
    {
        throw std::invalid_argument(
            "PathTrackingCritic: gate distance, softening speed and maximum "
            "convergence angle must be positive.");
    }

    if (settings_.curvature_lookahead == 0)
    {
        settings_.curvature_lookahead = 1;
    }

    if (settings_.projection_search_window == 0)
    {
        settings_.projection_search_window = 1;
    }
}

float PathTrackingCritic::normalizedAngle(float angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

float PathTrackingCritic::segmentYaw(const TensorPath & path, std::size_t segment)
{
    const std::size_t last_segment = path.x.size() - 2;
    const std::size_t index = std::min(segment, last_segment);
    const float dx = path.x(index + 1) - path.x(index);
    const float dy = path.y(index + 1) - path.y(index);

    if (std::hypot(dx, dy) <= 1.0e-6f)
    {
        return path.yaw(index);
    }

    return std::atan2(dy, dx);
}

PathTrackingCritic::Projection PathTrackingCritic::findForwardProjection(
    const TensorPath & path,
    float x,
    float y,
    std::size_t first_segment) const
{
    const std::size_t last_segment = path.x.size() - 2;
    first_segment = std::min(first_segment, last_segment);

    Projection best;
    best.segment = first_segment;
    best.distance_squared = std::numeric_limits<float>::infinity();

    const std::size_t search_end = first_segment + std::min(
        settings_.projection_search_window,
        last_segment - first_segment);

    // 距离“折线段”而不是离散路径点；严格重采样后也不会出现 0.1 m 的量化跳变。
    for (std::size_t segment = first_segment; segment <= search_end; ++segment)
    {
        const float start_x = path.x(segment);
        const float start_y = path.y(segment);
        const float dx = path.x(segment + 1) - start_x;
        const float dy = path.y(segment + 1) - start_y;
        const float length_squared = dx * dx + dy * dy;

        if (length_squared <= 1.0e-12f)
        {
            continue;
        }

        const float ratio = std::clamp(
            ((x - start_x) * dx + (y - start_y) * dy) / length_squared,
            0.0f,
            1.0f);
        const float projection_x = start_x + ratio * dx;
        const float projection_y = start_y + ratio * dy;
        const float error_x = x - projection_x;
        const float error_y = y - projection_y;
        const float distance_squared = error_x * error_x + error_y * error_y;

        if (distance_squared < best.distance_squared)
        {
            const float inv_length = 1.0f / std::sqrt(length_squared);
            const float tangent_x = dx * inv_length;
            const float tangent_y = dy * inv_length;

            best.segment = segment;
            best.distance_squared = distance_squared;
            best.tangent_yaw = std::atan2(dy, dx);

            // 左法向量为 (-tangent_y, tangent_x)。与投影误差做点积，
            // 正值表示轨迹点位于路径前进方向左侧。
            best.signed_lateral =
                -error_x * tangent_y + error_y * tangent_x;
        }
    }

    return best;
}

float PathTrackingCritic::estimateCurvature(
    const TensorPath & path,
    std::size_t segment) const
{
    const std::size_t last_segment = path.x.size() - 2;
    const std::size_t first =
        segment > settings_.curvature_lookahead ?
        segment - settings_.curvature_lookahead : 0;
    const std::size_t last = std::min(
        last_segment,
        segment + settings_.curvature_lookahead);

    float arc_length = 0.0f;
    for (std::size_t i = first; i <= last; ++i)
    {
        arc_length += std::hypot(
            path.x(i + 1) - path.x(i),
            path.y(i + 1) - path.y(i));
    }

    if (arc_length <= 1.0e-4f)
    {
        return 0.0f;
    }

    // 返回带符号的角度
    const float yaw_change = normalizedAngle(
        segmentYaw(path, last) - segmentYaw(path, first));
    // 正 = 左转
    return yaw_change / arc_length;                 // 曲率 = 朝向角变化 / 弧长
}

void PathTrackingCritic::score(CriticData & data) const
{
    if (!settings_.enabled)
    {
        return;
    }

    const std::size_t batch_size = data.trajectories.x.shape()[0];
    const std::size_t time_steps = data.trajectories.x.shape()[1];
    const std::size_t path_size = data.path.x.size();

    if (path_size < 2 ||
        data.path.y.size() != path_size ||
        data.trajectories.y.shape() != data.trajectories.x.shape() ||
        data.trajectories.yaw.shape() != data.trajectories.x.shape() ||
        data.state.vx.shape() != data.trajectories.x.shape() ||
        data.state.wz.shape() != data.trajectories.x.shape() ||
        data.costs.size() != batch_size)
    {
        throw std::runtime_error("PathTrackingCritic: inconsistent tensor dimensions.");
    }

    // 曲率只由参考路径决定，同一轮 score 中无需为每个 batch/time 重复计算。
    const std::size_t segment_count = path_size - 1;
    std::vector<float> path_curvature(segment_count, 0.0f);
    for (std::size_t segment = 0; segment < segment_count; ++segment)
    {
        path_curvature[segment] = estimateCurvature(data.path, segment);
    }

    for (std::size_t batch = 0; batch < batch_size; ++batch)
    {
        float accumulated_cost = 0.0f;
        float accumulated_weight = 0.0f;
        std::size_t first_segment = 0;

        for (std::size_t t = 0; t < time_steps; t += settings_.trajectory_step)
        {
            const float x = data.trajectories.x(batch, t);
            const float y = data.trajectories.y(batch, t);
            const float yaw = data.trajectories.yaw(batch, t);
            const float vx = data.state.vx(batch, t);
            const float wz = data.state.wz(batch, t);

            if (!std::isfinite(x) || !std::isfinite(y) ||
                !std::isfinite(yaw) || !std::isfinite(vx) ||
                !std::isfinite(wz))
            {
                accumulated_cost = settings_.invalid_cost;
                accumulated_weight = 1.0f;
                break;
            }

            // 找到该路径点所在折线
            const Projection projection =
                findForwardProjection(data.path, x, y, first_segment);

            if (!std::isfinite(projection.distance_squared) ||
                !std::isfinite(projection.tangent_yaw) ||
                !std::isfinite(projection.signed_lateral))
            {
                accumulated_cost = settings_.invalid_cost;
                accumulated_weight = 1.0f;
                break;
            }

            first_segment = projection.segment;

            const float lateral_error = std::sqrt(projection.distance_squared);         // 横向误差
            const float signed_lateral = projection.signed_lateral;             // 带符号横向误差
            const float heading_error = normalizedAngle(yaw - projection.tangent_yaw);      // 朝向误差

            const float curvature = path_curvature[projection.segment];         // 折线所在曲率
            const float curvature_mag = std::abs(curvature);

            // 弯道或横向/航向误差较大时主动降低参考速度。MPPI 因此会在靠近
            // 路径之前收油，而不是以恒定最大速度穿过路径后再反向纠偏。
           
            // 这里曲率减益系数计算见 image/cur_err_tuning.png。
            const float curve_speed = settings_.max_linear_speed /
                        (1.0f + settings_.curvature_gain * curvature_mag);
            
            // 横向误差越大、朝向误差越大，速度越小
            // 其中横偏 0.1 m 对分母贡献 2.5×0.1=0.252.5×0.1=0.25，航向差 0.25 rad（约14°）贡献 1.0×0.25=0.251.0×0.25=0.25
            // 横向误差、航向角误差减益系数调参见 image/error_param_tuning1.png
            const float error_speed = settings_.max_linear_speed /
                (1.0f +
                 settings_.lateral_error_speed_gain * lateral_error +
                 settings_.heading_error_speed_gain * std::abs(heading_error));

            // 防御性策略：选两个速度中最小的；以及最终的速度限幅
            const float reference_speed = std::clamp(
                std::min(curve_speed, error_speed),
                settings_.min_curve_speed,
                settings_.max_linear_speed);
            
            // 只处罚超过参考速度的部分；低于参考速度仍由 PathFollow 推动前进。
            const float speed_excess = std::max(vx - reference_speed, 0.0f);

            // ---------- 收敛航向目标 ----------
            // 路径左侧 signed_lateral > 0，此时 desired_heading_error < 0，
            // 机器人会朝右侧收敛；靠近路径后收敛角连续回到 0。
            float convergence_angle = 0.0f;
            if (settings_.use_convergence_heading)
            {
                // 核心思想：横向误差只能通过产生横向速度来抵消，而横向速度只能通过车头偏转产生（差速机器人）
                // 此处就在计算要抵消当前的横向误差需要多少的偏转角
                /*
                推导：
                    期望的横向速度正比于横向误差：v_lat_desired = −k_e · e_y_signed
                    车体的实际横向速度由航向偏差产生：v_lat_actual ≈ vx · sin(e_ψ) ≈ vx · e_ψ   （小角度）
                    令二者相等：
                        vx · e_ψ_desired ≈ −k_e · e_y_signed
                        e_ψ_desired ≈ −k_e · e_y_signed / vx
                */
                /*
                最精巧的地方在于消除了之前的冲突：
                    之前机器人有横向误差、平行于路径行走时，这个 heading-error 项的误差为0，而横向误差项却有代价，也就是两种代价冲突了
                    现在化横向误差为角速度，使得二者都有代价、都向着路径靠近
                */
               // 调参见 image/convergence_angle.png
                convergence_angle = std::atan2(
                    settings_.convergence_gain * signed_lateral,
                    std::abs(vx) + settings_.softening_speed);

                convergence_angle = std::clamp(
                    convergence_angle,
                    -settings_.max_convergence_angle,
                    settings_.max_convergence_angle);
            }

            // 若机器人在路径左侧，则 convergence_angle>0，此时需要机器人右转才行
            // 而 heading_error>0 代表机器人朝向左边，因此 convergence_angle>0 时需要 heading_erro<0，反之亦然
            const float desired_heading_error = -convergence_angle;
            const float heading_tracking_error = normalizedAngle(heading_error - desired_heading_error);

            // 不处罚“为了回到路径而必需的横向速度”，只处罚相对于期望收敛运动的横向速度残差。
            const float actual_lateral_velocity = vx * std::sin(heading_error);
            const float desired_lateral_velocity = vx * std::sin(desired_heading_error);
            const float lateral_velocity_error = actual_lateral_velocity - desired_lateral_velocity;

            // ---------- 角速度跟踪 + 近路径门控 ----------
            // vx * curvature 是曲率前馈；航向反馈项允许机器人执行必要纠偏，
            // 并在 heading_tracking_error 接近 0 时让参考角速度平滑回落。
            const float reference_wz =
                vx * curvature - settings_.yaw_rate_heading_gain * heading_tracking_error;
            const float yaw_rate_error = wz - reference_wz;                 // 多余的角速度

            /*
            |  横向误差 |    门控 |
            | ----: | ----: |
            |   0 m | 1.000 |
            | 0.1 m | 0.779 |
            | 0.2 m | 0.368 |
            | 0.3 m | 0.105 |
            | 0.4 m | 0.018 |
            */
            const float normalized_lateral = lateral_error / settings_.yaw_rate_gate_distance;
            const float near_path_gate = std::exp(-normalized_lateral * normalized_lateral);

            // 单个轨迹采样点的代价
            // 误差等价关系见 image/error_explain.png
            // 调参的计算见 image/error_param_tuning.png
            /*
            lateral_velocity_weight 的调参：
                0.1 m 横向误差代价：18×0.1^2=0.18
                若 heading_error_weight=8，10°航向代价：8(1−cos10∘)=0.12
                剩余约 0.06 分配给横向速度项：lateral_velocity_weight=(0.06)/(0.8*sin10)^2=3.1

            yaw_rate_error_weight 的调参：
                近路径且门控约为 1 时代价（当角速度误差为 0.3 rad/s）：2*0.3^2=0.18(恰好与 0.1 m 横向误差代价相同)
            */
            const float sample_cost =
                settings_.lateral_error_weight * projection.distance_squared +
                settings_.heading_error_weight * (1.0f - std::cos(heading_tracking_error)) +
                settings_.speed_error_weight * speed_excess * speed_excess +
                settings_.lateral_velocity_weight * lateral_velocity_error * lateral_velocity_error +
                // 门控会适时地惩罚多余的角速度（越近惩罚越大）
                near_path_gate * settings_.yaw_rate_error_weight * yaw_rate_error * yaw_rate_error;

            // 乘上时间权重，越向后权重越小（0.98^t）
            const float time_weight = std::pow(settings_.time_discount, static_cast<float>(t));

            accumulated_cost += time_weight * sample_cost;
            accumulated_weight += time_weight;
        }

        if (accumulated_weight > 0.0f)
        {
            data.costs(batch) += settings_.cost_weight *
                accumulated_cost / accumulated_weight;
        }
    }
}

}  // namespace local_planner::mppi_core
