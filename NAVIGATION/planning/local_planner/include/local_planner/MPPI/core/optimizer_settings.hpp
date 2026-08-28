#ifndef LOCAL_PLANNER__MPPI__CORE__OPTIMIZER_SETTINGS_HPP_
#define LOCAL_PLANNER__MPPI__CORE__OPTIMIZER_SETTINGS_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "local_planner/MPPI/core/types.hpp"

namespace local_planner::mppi_core
{
    struct OptimizerSettings
    {
        float model_dt{0.05f};                  // 时间间隔

        std::size_t time_steps{50};                 // 每条候选轨迹的预测时间步数
        std::size_t batch_size{500};                // 每个控制周期随机生成的候选轨迹数量
        std::size_t iteration_count{1};

        /*
        * 温度低：好的轨迹权重极大，差的轨迹权重接近 0 -> 结果趋近于“最优轨迹控制”。
        * 温度高：大家权重都差不多 -> 结果趋近于“所有轨迹平均值”。
        */
        float temperature{0.7f};                // 关闭自适应温度时使用的固定温度
        bool adaptive_temperature{true};        // 根据本轮 cost 分布自动调温，避免权重塌缩或近似均匀
        float target_ess_ratio{0.25f};          // 目标有效样本数 / 有效候选数
        float min_temperature{0.20f};
        float max_temperature{2.00f};
        float gamma{0.015f};            // 控制平滑性的权重参数（运动出现高频抖动，可尝试增大；若响应迟钝，可适当减小）

        float vx_max{0.6f};
        float vx_min{0.0f};
        float wz_max{0.8f};

        // 加速度限制
        float ax_min{-0.5f};
        float ax_max{0.5f};
        float az_max{1.5f};

        // 控制噪声的标准差，用于生成随机扰动
        float vx_std{0.15f};
        float wz_std{0.30f};

        bool shift_control_sequence{true};                  // 是否将上一周期最优控制序列向左平移一个时间步，作为本周期的初始均值控制序列

        // std::size_t retry_attempt_limit{1};                 // 当所有采样轨迹都无效（如碰撞）时，尝试重新生成轨迹的次数

        std::vector<Point2D> footprint_samples;             // 车体轮廓 footprint 外参
        double collision_distance{0.03};                      // 认为碰撞的距离

        // logging
        bool weight_logging_enabled{false};
        std::string logging_file_path;
    };

}  // namespace local_planner::mppi_core

#endif
