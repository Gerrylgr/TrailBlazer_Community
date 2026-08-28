#pragma once

#include <memory>
#include <Eigen/Core>

#include "field_map_builder/utility/esdf_query.hpp"

#include "trailblazer_map_interfaces/msg/esdf_map.hpp"

#include "lbfgs.hpp" 

namespace path_optimizer
{
    class LbfgsBackend;  // 前向声明，OptimizationContext 里要存它的指针

    // 配置结构体：initialize() 之后永不修改
    struct LbfgsBackendConfig
    {
        bool if_reference_cost{true};            
        bool if_curvature_cost{true};  

        double lambda_smooth{1.0};
        double lambda_distance{1.0};
        double obstacle_dist{0.8};
        double lambda_reference{1.0};
        double reference_tolerance{0.2};            // 超出原始路径这个距离才会产生代价
        double lambda_curvature{1.0};
        double max_curvature{1.0};   // 1/m
        double tangent_epsilon{1e-4};

        int max_iterations{100};
        double g_epsilon{1e-4};
        int fixed_boundary_control_points{3};
        int optimization_samples_per_span{5};

        int memory_size{8};
        int past{5};
        double delta{1.0e-4};
        int max_linesearch{20};
    };

    // 单次优化的输入
    struct OptimizeRequest
    {
        Eigen::MatrixXd initial_ctrl_pts;   // 初始控制点（2 x N，本函数不修改它）

        int degree{3};
        bool use_lbfgs{true};

        std::shared_ptr<const field_map_builder::EsdfMapSnapshot> esdf;

        const std::atomic_bool * cancel_requested{nullptr};
    };

    // 优化结果
    struct OptimizeResult
    {
        bool success{false};

        bool used_fallback{false};          // true 表示求解器未正常收敛，采用历史最好有效解

        std::string failure_reason;         // 失败原因

        Eigen::MatrixXd ctrl_pts;     // success 或 used_fallback 时有效
    };

    // 单次求解的全部"运行时状态"
    // 每次 optimize() 在栈上创建一份，lbfgs 回调通过 void* 拿到它
    struct OptimizationContext
    {
        const LbfgsBackend * backend{nullptr};   // 静态回调里用它调回后端 const 函数

        LbfgsBackendConfig config;               // 本次求解用的配置快照（拷贝）

        int degree{3};
        bool use_lbfgs{true};

        // 可优化控制点范围 [start_idx, end_idx)
        int start_idx{0};
        int end_idx{0};
        int variable_num{0};                     // 2 * (end_idx - start_idx)

        Eigen::MatrixXd initial_ctrl_pts;        // 本轮输入（重试时从这里重新开始）
        Eigen::MatrixXd reference_ctrl_pts;      // 参考路径控制点（= 初始控制点）
        Eigen::MatrixXd working_ctrl_pts;        // 当前迭代中的控制点

        Eigen::MatrixXd best_finite_ctrl_pts;    // 见过的最小 cost 的解
        double best_finite_cost{std::numeric_limits<double>::infinity()};
        double initial_cost{std::numeric_limits<double>::infinity()};

        std::shared_ptr<const field_map_builder::EsdfMapSnapshot> esdf;

        const std::atomic_bool * cancel_requested{nullptr};

        // 当前这一次目标函数评估是否有曲线采样点越出 ESDF（L-BFGS 线搜索中的临时状态，每次评估都会重新清零，不能作为永久失败）
        bool trial_out_of_bounds{false};

        // 真正不可恢复的错误，例如不支持的 B-spline degree
        bool invalid_evaluation{false};
        std::string failure_reason;

        bool cancelled{false};                  // 是否主动取消
    };

    class LbfgsBackend
    {
        public:
            LbfgsBackend() = default;
            ~LbfgsBackend() = default;

            void initialize(const LbfgsBackendConfig & config);

            // void updateEsdfMapFromMsg(const trailblazer_map_interfaces::msg::EsdfMap::SharedPtr msg);

            OptimizeResult optimize(const OptimizeRequest & request) const;

        private:
            // 单次 LBFGS 求解；结果写进 context.working_ctrl_pts
            int runLbfgsOnce(
                const Eigen::MatrixXd & start_ctrl_pts,
                OptimizationContext & context,
                int max_iterations,
                double & final_cost) const;

            // 评估：一维变量 -> (cost, 梯度)，并更新 context 中的 best 记录
            double evaluate(
                OptimizationContext & context,
                const Eigen::VectorXd & x,
                Eigen::VectorXd & g) const;

            // ---------------- 代价项（都多了一个 context 参数） ----------------
            double computeCostAndGradient(
                OptimizationContext & context,
                const Eigen::MatrixXd & ctrl_pts,
                Eigen::MatrixXd & grad,
                bool print_breakdown = false,
                const std::string & phase = "") const; 

            double computeSmoothnessCost(
                const Eigen::MatrixXd & ctrl_pts,
                Eigen::MatrixXd & grad) const;

            double computeDistanceCost(
                OptimizationContext & context,
                const Eigen::MatrixXd & ctrl_pts,
                Eigen::MatrixXd & grad) const;

            double computeReferenceCost(
                const OptimizationContext & context,
                const Eigen::MatrixXd & ctrl_pts,
                Eigen::MatrixXd & grad) const;

            double computeCurvatureCost(
                const OptimizationContext & context,
                const Eigen::MatrixXd & ctrl_pts,
                Eigen::MatrixXd & grad) const;

            // ---------------- 工具 ----------------
            Eigen::VectorXd flattenInnerControlPoints(
                const Eigen::MatrixXd & ctrl_pts,
                int start_idx, int end_idx) const;

            void assignInnerControlPoints(
                const Eigen::VectorXd & x,
                int start_idx, int end_idx,
                Eigen::MatrixXd & ctrl_pts) const;

            // ---------------- lbfgs 回调 ----------------
            static double costFunction(
                void * instance,
                const Eigen::VectorXd & x,
                Eigen::VectorXd & g);

            static int monitorProgress(
                void * instance,
                const Eigen::VectorXd & x,
                const Eigen::VectorXd & g,
                const double fx,
                const double step,
                const int k,
                const int ls);

        private:
            LbfgsBackendConfig config_;
    };

}   // namespace path_optimizer