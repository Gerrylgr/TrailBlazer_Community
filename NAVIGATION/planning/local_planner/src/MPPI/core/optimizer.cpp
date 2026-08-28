/*
*   pipeline:
*       evalControl:
*           prepare:
*               接收 input 的 robot_pose、robot_speed、esdf_snapshot、转换 reference_path 为 tensor_path
*           根据 iteration 数目 optimizeOnce：
*               generateNoisedTrajectories()
*               构造 CriticData，score 方法计算所有 critic 的代价
*               updateControlSequence()
*
*           command_vx =control_sequence_.vx(0);
*           command_wz = control_sequence_.wz(0);
*           buildOptimizedTrajectory()
*           validateTrajectory()
*           shiftControlSequence()       
*/
#include "local_planner/MPPI/core/optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace local_planner::mppi_core
{

Optimizer::Optimizer(const OptimizerSettings & settings, std::unique_ptr<CriticManager> critic_manager)
            : settings_(settings), critic_manager_(std::move(critic_manager))
{
    if (!critic_manager_)
    {
        throw std::invalid_argument("Optimizer: critic_manager must not be null.");
    }

    if (settings_.batch_size == 0 || settings_.time_steps == 0 || settings_.iteration_count == 0)
    {
        throw std::invalid_argument("Optimizer: dimensions and iteration_count must be positive.");
    }

    if (settings_.model_dt <= 0.0f || settings_.temperature <= 0.0f)
    {
        throw std::invalid_argument("Optimizer: model_dt and temperature must be positive.");
    }

    if (settings_.min_temperature <= 0.0f ||
        settings_.max_temperature < settings_.min_temperature ||
        settings_.target_ess_ratio <= 0.0f ||
        settings_.target_ess_ratio > 1.0f)
    {
        throw std::invalid_argument("Optimizer: invalid adaptive-temperature settings.");
    }

    noise_generator_.initialize(settings_);
    reset();
}

void Optimizer::reset()
{
    state_.reset(settings_.batch_size, settings_.time_steps);

    control_sequence_.reset(settings_.time_steps);

    trajectories_.reset(settings_.batch_size, settings_.time_steps);

    costs_ = xt::zeros<float>({settings_.batch_size});

    path_.reset(0);
    current_esdf_.reset();

    noise_generator_.reset();
}

TensorPath Optimizer::toTensor(const ReferencePath & path) const
{
    TensorPath result;
    result.reset(path.size());

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        result.x(i) = path.poses[i].x;
        result.y(i) = path.poses[i].y;
        result.yaw(i) = path.poses[i].yaw;
    }

    return result;
}

void Optimizer::prepare(const OptimizerInput & input)
{
    state_.pose = input.robot_pose;
    state_.speed = input.robot_speed;

    path_ = toTensor(input.reference_path);

    current_esdf_ = input.esdf;
}

OptimizerResult Optimizer::evalControl(const OptimizerInput & input)
{
    OptimizerResult result;

    if (input.reference_path.size() < 2)
    {
        return result;
    }

    prepare(input);

    ++log_cycle_;        
    log_iteration_ = 0;  

    for (std::size_t iteration = 0; iteration < settings_.iteration_count; ++iteration)
    {
        optimizeOnce();
    }

    if (control_sequence_.vx.size() == 0 || control_sequence_.wz.size() == 0)
    {
        return result;
    }

    // 真正可执行的速度
    const float command_vx = advanceLinearVelocity(state_.speed.vx, control_sequence_.vx(0));
    const float command_wz = advanceAngularVelocity(state_.speed.wz,control_sequence_.wz(0));

    // // 计算这一次理论上真正能达到的速度
    // const float min_dvx = settings_.ax_min *settings_.model_dt;
    // const float max_dvx = settings_.ax_max * settings_.model_dt;
    // const float max_dwz = settings_.az_max * settings_.model_dt;

    // const float command_vx = state_.speed.vx + std::clamp(control_sequence_.vx(0) - state_.speed.vx, min_dvx, max_dvx);
    // const float command_wz = state_.speed.wz + std::clamp(control_sequence_.wz(0) - state_.speed.wz, -max_dwz, max_dwz);

    if (!std::isfinite(command_vx) || !std::isfinite(command_wz))
    {
        reset();
        return result;
    }

    // 单次的控制指令
    result.command.vx = command_vx;
    result.command.vy = 0.0f;
    result.command.wz = command_wz;

    result.minimum_cost = costs_.size() > 0 ? *std::min_element(costs_.begin(), costs_.end()) : 0.0f;

    buildOptimizedTrajectory(result);

    if (!input.esdf)
    {
        result.valid = false;
        return result;
    }

    result.valid = validateTrajectory(result.optimized_trajectory, *input.esdf);

    if (result.valid)
    {
        if (settings_.shift_control_sequence)
        {
            shiftControlSequence();
        }
    }
    else                    // 无效轨迹不再 warm start
    {
        reset();
    }

    return result;
}

/*
*   generateNoisedTrajectories
*   计算所有 critic 的代价
*   updateControlSequence
*/
void Optimizer::optimizeOnce()
{
    // 每轮优化必须重新清空 Critic 总代价
    costs_.fill(0.0f);

    generateNoisedTrajectories();

    // 构造 critic data
    CriticData data{
        state_,
        trajectories_,
        path_,
        costs_,
        settings_.model_dt,
        current_esdf_
    };

    critic_manager_->score(data);               // 计算所有 critic 的代价

    if (!updateControlSequence())
    {
        throw std::runtime_error("Optimizer: failed to update control sequence.");
    }

    ++log_iteration_; 
}

void Optimizer::generateNoisedTrajectories()
{
    noise_generator_.generate();

    noise_generator_.apply(state_, control_sequence_);

    // 注入几条确定性候选控制序列
    if (settings_.batch_size >= 4)
    {
        const float rotate_wz = 0.75f * settings_.wz_max;
        const float forward_vx = std::min(0.25f, settings_.vx_max);

        for (std::size_t t = 0; t < settings_.time_steps; ++t)
        {
            // 序列0：保持当前名义控制序列
            state_.cvx(0, t) = control_sequence_.vx(t);
            state_.cwz(0, t) = control_sequence_.wz(t);

            // 序列1：原地持续左转
            state_.cvx(1, t) = 0.0f;
            state_.cwz(1, t) = rotate_wz;

            // 序列2：原地持续右转
            state_.cvx(2, t) = 0.0f;
            state_.cwz(2, t) = -rotate_wz;

            // 3：匀速直行
            state_.cvx(3, t) = forward_vx;
            state_.cwz(3, t) = 0.0f;
        }
    }

    // 所有候选最后统一限幅
    state_.cvx = xt::clip(state_.cvx, settings_.vx_min, settings_.vx_max);
    state_.cwz = xt::clip(state_.cwz, -settings_.wz_max, settings_.wz_max);

    integrateCandidateTrajectories();
}

void Optimizer::integrateCandidateTrajectories()
{
    // 遍历每条轨迹
    for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
    {
        float x = state_.pose.x;
        float y = state_.pose.y;
        float yaw = state_.pose.yaw;

        // 预测速度初始化为当前速度
        float predicted_vx = state_.speed.vx;
        float predicted_wz = state_.speed.wz;

        // 遍历所有时间步，
        for (std::size_t t = 0; t < settings_.time_steps; ++t)
        {
            // 预测速度更新（更新幅度不能超过机器人极限）
            predicted_vx = advanceLinearVelocity(predicted_vx, state_.cvx(batch, t));
            predicted_wz = advanceAngularVelocity(predicted_wz, state_.cwz(batch, t));

            state_.vx(batch, t) = predicted_vx;
            state_.wz(batch, t) = predicted_wz;

            Pose2D predicted_pose{x, y, yaw};
            propagatePose(predicted_pose, predicted_vx, predicted_wz);
            x = predicted_pose.x;
            y = predicted_pose.y;
            yaw = predicted_pose.yaw;

            trajectories_.x(batch, t) = x;
            trajectories_.y(batch, t) = y;
            trajectories_.yaw(batch, t) = yaw;
        }
    }
}

bool Optimizer::updateControlSequence()
{
    // 噪声方差
    const float vx_variance = settings_.vx_std * settings_.vx_std;
    const float wz_variance = settings_.wz_std *settings_.wz_std;

    if (vx_variance <= 0.0f || wz_variance <= 0.0f)
    {
        return false;
    }

    // 遍历所有路径，加入 MPPI 控制扰动代价
    for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
    {
        float control_cost = 0.0f;

        for (std::size_t t = 0; t < settings_.time_steps; ++t)
        {
            const float vx_noise = state_.cvx(batch, t) - control_sequence_.vx(t);
            const float wz_noise = state_.cwz(batch, t) - control_sequence_.wz(t);

            control_cost +=
                settings_.gamma *
                (
                    control_sequence_.vx(t) * vx_noise / vx_variance +
                    control_sequence_.wz(t) * wz_noise / wz_variance
                );
        }

        costs_(batch) += control_cost;
    }

    float minimum_cost = std::numeric_limits<float>::infinity();

     for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
    {
        // 只有有限的代价才参与 minimum_cost 的计算
        if (std::isfinite(costs_(batch)))
        {
            minimum_cost = std::min(minimum_cost, costs_(batch));
        }
    }

    // 如果所有的候选轨迹代价都是无效的（minimum_cost 仍然是无穷大），则本轮优化失败
    if (!std::isfinite(minimum_cost))
    {
        return false;
    }

    // 理想温度
    const float effective_temperature = selectTemperature(minimum_cost);

    xt::xtensor<float, 1> weights = xt::zeros<float>({settings_.batch_size});       // 不同序列的代价

    float weight_sum = 0.0f;

    for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
    {
        // 如果候选轨迹代价是 NaN 或 Inf，直接将其权重设为 0 并跳过
        if (!std::isfinite(costs_(batch)))
        {
            weights(batch) = 0.0f;
            continue;
        }

        const float normalized_cost = costs_(batch) - minimum_cost;             // 代价归一化（最好的代价为0）

        /*
        * 温度低：好的轨迹权重极大，差的轨迹权重接近 0 -> 结果趋近于“最优轨迹控制”。
        * 温度高：大家权重都差不多 -> 结果趋近于“所有轨迹平均值”。
        */
        const float weight = std::exp(-normalized_cost / effective_temperature);                // 权重

        weights(batch) = weight;
        weight_sum += weight;
    }

    if (!std::isfinite(weight_sum) || weight_sum <= 1.0e-12f)       // 所有有效轨迹的权重之和太小，说明没有可用候选，优化失败
    {
        return false;
    }

    float weight_square_sum = 0.0f;
    for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
    {
        weight_square_sum += weights(batch) * weights(batch);
    }
    const float effective_sample_size =                         // ESS
        weight_square_sum > 1.0e-20f ?
        weight_sum * weight_sum / weight_square_sum : 0.0f;

    logWeights(
        minimum_cost,
        weights,
        weight_sum,
        effective_temperature,
        effective_sample_size);

    for (std::size_t t = 0; t < settings_.time_steps; ++t)
    {
        float weighted_vx = 0.0f;
        float weighted_wz = 0.0f;

        // 遍历所有候选路径点，根据权重加权得到控制序列
        for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
        {
            const float normalized_weight = weights(batch) / weight_sum;                // 归一化的权重

            weighted_vx += normalized_weight * state_.cvx(batch, t);
            weighted_wz += normalized_weight * state_.cwz(batch, t);
        }

        control_sequence_.vx(t) = weighted_vx;
        control_sequence_.wz(t) = weighted_wz;
    }

    applyControlConstraints();
    return true;
}

float Optimizer::effectiveSampleSize(float minimum_cost, float temperature) const
{
    float weight_sum = 0.0f;
    float weight_square_sum = 0.0f;

    for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
    {
        if (!std::isfinite(costs_(batch)))
        {
            continue;
        }

        const float weight = std::exp(-(costs_(batch) - minimum_cost) / temperature);
        weight_sum += weight;                         
        weight_square_sum += weight * weight;          
    }

    // 计算重要性采样 ESS（有效投票数量）
    return weight_square_sum > 1.0e-20f ? weight_sum * weight_sum / weight_square_sum : 0.0f;
}

float Optimizer::selectTemperature(float minimum_cost) const
{
    if (!settings_.adaptive_temperature)
    {
        return settings_.temperature;
    }

    std::size_t valid_count = 0;
    for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
    {
        valid_count += std::isfinite(costs_(batch)) ? 1U : 0U;
    }

    if (valid_count == 0)
    {
        return settings_.temperature;
    }

    const float target_ess = std::clamp(
        settings_.target_ess_ratio * static_cast<float>(valid_count),
        1.0f,
        static_cast<float>(valid_count));

    float lower = settings_.min_temperature;
    float upper = settings_.max_temperature;

    // 温度越高、权重越平均
    if (effectiveSampleSize(minimum_cost, lower) >= target_ess)
    {
        // 最低的温度（集权最严重时）有效投票数量都超过了目标值
        return lower;
    }

    if (effectiveSampleSize(minimum_cost, upper) <= target_ess)
    {
        // 最高温度（权重最分散时）有效投票数量都小于目标值
        return upper;
    }

    // ESS 随温度单调增大，此处二分法求解目标 ESS
    // 此处24轮恰好能够榨干浮点数精度，具体解释见 image/ess_explain.png
    for (int iteration = 0; iteration < 24; ++iteration)
    {
        const float middle = 0.5f * (lower + upper);
        if (effectiveSampleSize(minimum_cost, middle) < target_ess)
        {
            lower = middle;
        }
        else
        {
            upper = middle;
        }
    }

    return upper;
}

void Optimizer::applyControlConstraints()
{
    for (std::size_t t = 0; t < settings_.time_steps; ++t)
    {
        control_sequence_.vx(t) = std::clamp(control_sequence_.vx(t), settings_.vx_min, settings_.vx_max);
        control_sequence_.wz(t) = std::clamp(control_sequence_.wz(t), -settings_.wz_max, settings_.wz_max);
    }
}

void Optimizer::shiftControlSequence()
{
    if (settings_.time_steps < 2)
    {
        return;
    }

    for (std::size_t t = 0; t + 1 < settings_.time_steps; ++t)
    {
        control_sequence_.vx(t) = control_sequence_.vx(t + 1);
        control_sequence_.wz(t) = control_sequence_.wz(t + 1);
    }

    control_sequence_.vx(settings_.time_steps - 1) = control_sequence_.vx(settings_.time_steps - 2);
    control_sequence_.wz(settings_.time_steps - 1) = control_sequence_.wz(settings_.time_steps - 2);
}

void Optimizer::buildOptimizedTrajectory(OptimizerResult & result) const
{
    result.optimized_trajectory.clear();
    result.optimized_trajectory.reserve(settings_.time_steps + 1);

    float x = state_.pose.x;
    float y = state_.pose.y;
    float yaw = state_.pose.yaw;

    float predicted_vx = state_.speed.vx;
    float predicted_wz = state_.speed.wz;

    // 把预测使用的起始状态也发布出去。RViz 中轨迹的第一个点应与
    // 经过坐标变换和延迟补偿后的机器人状态重合，便于排查 frame/timestamp 问题。
    result.optimized_trajectory.push_back(Pose2D{x, y, yaw});

    for (std::size_t t = 0; t < settings_.time_steps; ++t)
    {
        predicted_vx = advanceLinearVelocity(predicted_vx, control_sequence_.vx(t));
        predicted_wz = advanceAngularVelocity(predicted_wz, control_sequence_.wz(t));

        Pose2D predicted_pose{x, y, yaw};
        propagatePose(predicted_pose, predicted_vx, predicted_wz);
        x = predicted_pose.x;
        y = predicted_pose.y;
        yaw = predicted_pose.yaw;

        result.optimized_trajectory.push_back(Pose2D{x, y, yaw});
    }
}

bool Optimizer::validateTrajectory(const std::vector<Pose2D> & trajectory, const field_map_builder::EsdfMapSnapshot & esdf) const
{
    bool valid = true;

    // 第 0 个点是仅用于可视化/对齐检查的当前状态，从第一个未来点开始验证。
    for(size_t i = trajectory.size() > 1 ? 1 : 0; i < trajectory.size(); i++)
    {
        const float robot_x = trajectory[i].x;
        const float robot_y = trajectory[i].y;
        const float robot_yaw = trajectory[i].yaw;

        const float cos_yaw = std::cos(robot_yaw);
        const float sin_yaw = std::sin(robot_yaw);

        float minimum_distance = std::numeric_limits<float>::infinity();

        for (const auto & sample : settings_.footprint_samples)
        {
            const float world_x = robot_x + cos_yaw * sample.x - sin_yaw * sample.y;
            const float world_y = robot_y + sin_yaw * sample.x + cos_yaw * sample.y;

            float distance = 0.0;
            if(!esdf.distance_bilinear(world_x, world_y, distance))
            {
                return false;
            }

            if (!std::isfinite(distance))
            {
                valid = false;
                break;
            }

            minimum_distance = std::min(minimum_distance, distance);
        }

        if (!valid || minimum_distance <= settings_.collision_distance)
        {
            valid = false;
            break;
        }
    }

    return valid;
}

float Optimizer::advanceLinearVelocity(float current, float target) const
{
    target = std::clamp(target, settings_.vx_min, settings_.vx_max);

    return current + std::clamp(
        target - current,
        settings_.ax_min * settings_.model_dt,
        settings_.ax_max * settings_.model_dt);
}

float Optimizer::advanceAngularVelocity(float current, float target) const
{
    target = std::clamp(target, -settings_.wz_max, settings_.wz_max);

    const float max_delta = settings_.az_max * settings_.model_dt;

    return current + std::clamp(
        target - current,
        -max_delta,
        max_delta);
}

/*
原版：
    x += vx * cos(yaw) * dt;      // 用"本步开始时"的朝向走完整步
    y += vx * sin(yaw) * dt;
    yaw += wz * dt;               // 转向放到最后
*/
void Optimizer::propagatePose(Pose2D & pose, float vx, float wz) const
{
    // 中点积分比“先平移、后更新 yaw”的显式 Euler 在弯道上偏差更小。
    const float yaw_delta = wz * settings_.model_dt;
    const float midpoint_yaw = pose.yaw + 0.5f * yaw_delta;         // 取半程朝向

    pose.x += vx * std::cos(midpoint_yaw) * settings_.model_dt;         // 用半程朝向走完整步
    pose.y += vx * std::sin(midpoint_yaw) * settings_.model_dt;
    pose.yaw = std::atan2(
        std::sin(pose.yaw + yaw_delta),
        std::cos(pose.yaw + yaw_delta));
}

void Optimizer::enableWeightLogging()
{
    if (!settings_.weight_logging_enabled)
    {
        return;
    }

    weight_log_file.open(settings_.logging_file_path, std::ios::out | std::ios::trunc);
    settings_.weight_logging_enabled = weight_log_file.is_open();
    if (settings_.weight_logging_enabled)
    {
        // 把当时的参数写进文件头，方便回溯
        weight_log_file << "# temperature=" << settings_.temperature
                         << " batch_size=" << settings_.batch_size
                         << " iteration_count=" << settings_.iteration_count << "\n";
        weight_log_file <<
            "cycle,iteration,batch,cost,normalized_cost,weight,normalized_weight,"
            "effective_temperature,ess\n";
    }
}

void Optimizer::logWeights(float minimum_cost,
                           const xt::xtensor<float, 1> & weights,
                           float weight_sum,
                           float effective_temperature,
                           float effective_sample_size)
{
    if (!settings_.weight_logging_enabled || !weight_log_file.is_open())
    {
        return;
    }
    // 只记录每个 cycle 最后一次迭代，避免调试 I/O 反过来拖慢控制周期。
    if (log_iteration_ + 1 < settings_.iteration_count)
    {
        return;
    }

    for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
    {
        const float cost = costs_(batch);
        weight_log_file
            << log_cycle_ << ','
            << log_iteration_ << ','
            << batch << ','
            << cost << ','
            << (cost - minimum_cost) << ','                // normalized_cost
            << weights(batch) << ','
            << (weight_sum > 0.0f ? weights(batch) / weight_sum : 0.0f) << ','
            << effective_temperature << ','
            << effective_sample_size
            << '\n';
    }
}

}  // namespace local_planner::mppi_core
