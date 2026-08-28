#ifndef LOCAL_PLANNER__MPPI__CORE__NOISE_GENERATOR_HPP_
#define LOCAL_PLANNER__MPPI__CORE__NOISE_GENERATOR_HPP_

#include "local_planner/MPPI/core/types.hpp"
#include "local_planner/MPPI/core/optimizer_settings.hpp"

#include <xtensor/xtensor.hpp>

namespace local_planner::mppi_core
{

    class NoiseGenerator
    {
    public:
        NoiseGenerator() = default;

        void initialize(const OptimizerSettings & settings);

        void reset();           // reset noise_vx_/noise_wz_

        // 生成随机噪声
        // 并不用初始的高斯白噪声，而是又加了一层一阶低通滤波，是为了让生成的噪声也是连续、可执行的
        void generate();        

        // 将噪声（noise_vx_/noise_wz_）应用到上一个周期的最优序列上（control_sequence.vx/wz），
        // 得到一大堆候选序列（state.cvx/cwz/cvy）
        // 输入的 control_sequence 是(1, 50)形状的，加上(500, 50)形状的 noise_vx_/noise_wz_ 得到 候选序列
        void apply(State & state, const ControlSequence & control_sequence) const;

    private:
        OptimizerSettings settings_;

        xt::xtensor<float, 2> noise_vx_;
        xt::xtensor<float, 2> noise_wz_;

        bool initialized_{false};
    };

}  // namespace local_planner::mppi_core

#endif