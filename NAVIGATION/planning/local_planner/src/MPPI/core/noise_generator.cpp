#include "local_planner/MPPI/core/noise_generator.hpp"

#include <stdexcept>

#include <xtensor/xrandom.hpp>
#include <xtensor/xview.hpp>

namespace local_planner::mppi_core
{

    void NoiseGenerator::initialize(const OptimizerSettings & settings)
    {
        settings_ = settings;

        if (settings_.batch_size == 0 || settings_.time_steps == 0)
        {
            throw std::invalid_argument("NoiseGenerator: batch_size and time_steps must be positive.");
        }

        if (settings_.vx_std <= 0.0f || settings_.wz_std <= 0.0f)
        {
            throw std::invalid_argument("NoiseGenerator: noise standard deviations must be positive.");
        }

        initialized_ = true;
        reset();
    }

    void NoiseGenerator::reset()
    {
        if (!initialized_)
        {
            return;
        }

        noise_vx_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
        noise_wz_ = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    }

    void NoiseGenerator::generate()
    {
        if (!initialized_)
        {
            throw std::runtime_error("NoiseGenerator has not been initialized.");
        }

        // noise_vx_(i, j) 被加到第 i 条轨迹的第 j 个时间步的线速度上
        noise_vx_ = xt::random::randn<float>(
            {
                settings_.batch_size,
                settings_.time_steps
            },
            0.0f,                   // 噪声均值
            settings_.vx_std);

        // noise_wz_(i, j) 被加到角速度上
        noise_wz_ = xt::random::randn<float>(
            {
                settings_.batch_size,
                settings_.time_steps
            },
            0.0f,
            settings_.wz_std);

        // 使用经过一阶低通滤波后的噪声
        const float vx_alpha = 0.80f;
        const float wz_alpha = 0.90f;

        const float vx_scale = std::sqrt(1.0f - vx_alpha * vx_alpha);       // 0.6
        const float wz_scale = std::sqrt(1.0f - wz_alpha * wz_alpha);       // 0.44

        for (std::size_t batch = 0; batch < settings_.batch_size; ++batch)
        {
            for (std::size_t t = 1; t < settings_.time_steps; ++t)
            {
                noise_vx_(batch, t) = vx_alpha * noise_vx_(batch, t - 1) + vx_scale * noise_vx_(batch, t);
                noise_wz_(batch, t) = wz_alpha * noise_wz_(batch, t - 1) + wz_scale * noise_wz_(batch, t);
            }
        }

        // 成对的正/负扰动可显著减小有限 batch 下的采样均值偏置。
        // 这样在降低 temperature、提高纠偏力度时，不会因为某一轮随机样本
        // 恰好偏左或偏右而频繁翻转控制方向。
        const std::size_t pair_count = settings_.batch_size / 2;
        for (std::size_t batch = 0; batch < pair_count; ++batch)
        {
            const std::size_t paired_batch = batch + pair_count;
            for (std::size_t t = 0; t < settings_.time_steps; ++t)
            {
                noise_vx_(paired_batch, t) = -noise_vx_(batch, t);
                noise_wz_(paired_batch, t) = -noise_wz_(batch, t);
            }
        }
    }

    void NoiseGenerator::apply(State & state, const ControlSequence & control_sequence) const
    {
        if (!initialized_)
        {
            throw std::runtime_error("NoiseGenerator has not been initialized.");
        }

        /*
        *   // 形状 (1, 50) + 形状 (500, 50)
        *   state.cvx = view(...) + noise_vx_;
        *   广播机制：左边 (1, 50) 会自动“复制” 500 次，变成 (500, 50)，与右边 (500, 50) 的噪声矩阵相加得到500份不同的轨迹
        */
        state.cvx =
            xt::view(
                control_sequence.vx,
                xt::newaxis(),              // 表示插入一个新维度      
                xt::all())                  // 取该维度的所有元素
                + noise_vx_;

        state.cwz =
            xt::view(
                control_sequence.wz,
                xt::newaxis(),
                xt::all()) 
                + noise_wz_;

        // 差速底盘没有横向速度
        state.cvy.fill(0.0f);
    }

}  // namespace local_planner::mppi_core
