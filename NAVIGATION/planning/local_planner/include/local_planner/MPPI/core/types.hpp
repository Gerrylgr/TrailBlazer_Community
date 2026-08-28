#ifndef LOCAL_PLANNER__MPPI__CORE__TYPES_HPP_
#define LOCAL_PLANNER__MPPI__CORE__TYPES_HPP_

#include <memory>
#include <vector>

#include <xtensor/xtensor.hpp>

namespace local_planner::mppi_core
{
    struct Point2D
    {
        float x{0.0f};
        float y{0.0f};
    };

    // 二维位姿
    struct Pose2D
    {
        float x{0.0f};
        float y{0.0f};
        float yaw{0.0f};
    };

    // 二维运动速度
    struct Twist2D
    {
        float vx{0.0f};
        float vy{0.0f};
        float wz{0.0f};
    };

    // 控制指令
    struct ControlCommand
    {
        float vx{0.0f};
        float vy{0.0f};
        float wz{0.0f};
    };

    // （从 global_path 直接截取的）参考路径
    struct ReferencePath
    {
        std::vector<Pose2D> poses;

        bool empty() const
        {
            return poses.empty();
        }

        std::size_t size() const
        {
            return poses.size();
        }

        void clear()
        {
            poses.clear();
        }
    };

    // 目标路径（需要追随的，代价就按这个为标准来计算）
    struct TensorPath
    {
        xt::xtensor<float, 1> x;
        xt::xtensor<float, 1> y;
        xt::xtensor<float, 1> yaw;

        void reset(std::size_t size)
        {
            x = xt::zeros<float>({size});
            y = xt::zeros<float>({size});
            yaw = xt::zeros<float>({size});
        }
    };

    // 存储机器人当前位姿速度，以及候选控制指令/预测速度
    struct State
    {
        // 第一个维度：第n条轨迹
        // 第二个维度：第m个时间步
        // 实际预测速度（通过运动学模型预测出来）
        xt::xtensor<float, 2> vx;           // 例如 vx(50, 10) 表示：第 50 条候选轨迹，在未来第 10 个时间步时的线速度
        xt::xtensor<float, 2> vy;
        xt::xtensor<float, 2> wz;

        // 加入随机噪声后的候选控制指令
        xt::xtensor<float, 2> cvx;          // 例如 cvx(50, 10) 代表：在第 50 条候选轨迹中，未来第 10 个时间步时的线速度控制量（输入指令）
        xt::xtensor<float, 2> cvy;
        xt::xtensor<float, 2> cwz;

        Pose2D pose;            // 当前位姿
        Twist2D speed;          // 当前速度

        void reset(std::size_t batch_size, std::size_t time_steps)
        {
            vx = xt::zeros<float>({batch_size, time_steps});
            vy = xt::zeros<float>({batch_size, time_steps});
            wz = xt::zeros<float>({batch_size, time_steps});

            cvx = xt::zeros<float>({batch_size, time_steps});
            cvy = xt::zeros<float>({batch_size, time_steps});
            cwz = xt::zeros<float>({batch_size, time_steps});
        }
    };

    // 优化后的最优控制序列（形状：(time_steps)）
    struct ControlSequence
    {
        xt::xtensor<float, 1> vx;
        xt::xtensor<float, 1> vy;
        xt::xtensor<float, 1> wz;

        void reset(std::size_t time_steps)
        {
            vx = xt::zeros<float>({time_steps});
            vy = xt::zeros<float>({time_steps});
            wz = xt::zeros<float>({time_steps});
        }
    };

    // 预测轨迹集合（通过运动学模型预测出来）
    struct Trajectories
    {
        xt::xtensor<float, 2> x;
        xt::xtensor<float, 2> y;
        xt::xtensor<float, 2> yaw;

        void reset(std::size_t batch_size, std::size_t time_steps)
        {
            x = xt::zeros<float>({batch_size, time_steps});
            y = xt::zeros<float>({batch_size, time_steps});
            yaw = xt::zeros<float>({batch_size, time_steps});
        }
    };

    // 优化器结果
    struct OptimizerResult
    {
        bool valid{false};
        ControlCommand command;             // 单次的控制指令
        std::vector<Pose2D> optimized_trajectory;
        float minimum_cost{0.0f};
    };

}  // namespace local_planner::mppi_core

#endif