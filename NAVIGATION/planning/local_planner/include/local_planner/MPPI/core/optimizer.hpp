#ifndef LOCAL_PLANNER__MPPI__CORE__OPTIMIZER_HPP_
#define LOCAL_PLANNER__MPPI__CORE__OPTIMIZER_HPP_

#include "local_planner/MPPI/core/types.hpp"
#include "local_planner/MPPI/core/optimizer_settings.hpp"
#include "local_planner/MPPI/core/noise_generator.hpp"
#include "local_planner/MPPI/core/critic_manager.hpp"
#include "field_map_builder/utility/esdf_query.hpp"

#include <memory>
#include <fstream>   
#include <string>    

namespace local_planner::mppi_core
{

    struct OptimizerInput
    {
        Pose2D robot_pose;
        Twist2D robot_speed;

        ReferencePath reference_path;

        std::shared_ptr<const field_map_builder::EsdfMapSnapshot> esdf;
    };

    class Optimizer
    {
    public:
        Optimizer(const OptimizerSettings & settings, std::unique_ptr<CriticManager> critic_manager);

        void reset();

        OptimizerResult evalControl(const OptimizerInput & input);          // 主入口

        void enableWeightLogging();

    private:
        /*
        * 赋值 pose/speed/current_esdf
        * 将 reference_path 转为 tensor_path
        */
        void prepare(const OptimizerInput & input);

        void optimizeOnce();

        /*
        * 生成噪声、应用到之前的控制序列得到 state、再进行预测
        * 同时手动注入四条候选轨迹序列，保证候选中有：
        *   一条持续左转轨迹
        *   一条持续右转轨迹
        *   一条直行轨迹
        *   一条名义控制轨迹
        */
        void generateNoisedTrajectories();

        // 在随机生成的一大堆带有噪声的控制序列的基础上，结合运动学模型，
        // 预测每条轨迹未来的速度/位置（用于后续 critic 代价计算）
        void integrateCandidateTrajectories();

        /*
        *   计算所有候选路线的 MPPI 控制扰动代价（限制控制量的大小）
        *   （找到最小的代价来）对代价归一化，并应用上 temperature 得到权重，
        *   根据权重对不同路线加权得到控制序列
        *   对结果限幅
        */
        bool updateControlSequence();

        // 输入 minimum_cost、temperature，根据 costs_ 计算出重要性采样 ESS（有效投票数量）
        float effectiveSampleSize(float minimum_cost, float temperature) const;
        // 根据 target_ess_ratio 求解目标 temperature 大小
        float selectTemperature(float minimum_cost) const;

        // 对控制序列的结果限幅
        void applyControlConstraints();

        // 时间序列平移（热启动）
        void shiftControlSequence();

        // 根据本次计算出的 control_sequence_ 与运动学模型构造出本次的 optimized_trajectory
        void buildOptimizedTrajectory(OptimizerResult & result) const;

        // 对加权平均后的最终路线做碰撞检测(返回是否合法)，发生碰撞就会返回 false
        bool validateTrajectory(const std::vector<Pose2D> & trajectory, const field_map_builder::EsdfMapSnapshot & esdf) const;

        TensorPath toTensor(const ReferencePath & path) const;          // 将 ReferencePath 路径转为 TensorPath 路径

        // 输入 current、target，根据限幅计算出真正的可执行速度
        float advanceLinearVelocity(float current, float target) const;
        float advanceAngularVelocity(float current, float target) const;

        // 更新位姿
        void propagatePose(Pose2D & pose, float vx, float wz) const;

        // logging
        void logWeights(float minimum_cost,
                    const xt::xtensor<float, 1> & weights,
                    float weight_sum,
                    float effective_temperature,
                    float effective_sample_size);

    private:
        OptimizerSettings settings_;

        std::unique_ptr<CriticManager> critic_manager_;

        NoiseGenerator noise_generator_;

        State state_;               // 当前机器人状态
        ControlSequence control_sequence_;              // 最优控制序列
        Trajectories trajectories_;                 // 预测的轨迹
        TensorPath path_;

        xt::xtensor<float, 1> costs_;               // 各个路径的总代价

        std::shared_ptr<const field_map_builder::EsdfMapSnapshot> current_esdf_;


        std::size_t log_cycle_{0};      // 第几次 evalControl
        std::size_t log_iteration_{0};  // 本次 evalControl 内第几次迭代
        std::ofstream weight_log_file;
    };

}  // namespace local_planner::mppi_core

#endif
