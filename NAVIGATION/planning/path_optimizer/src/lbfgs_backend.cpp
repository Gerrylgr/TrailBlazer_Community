/*
调用 LBFGS-Lite 实现 LBFGS 后端优化
*/

#include "path_optimizer/lbfgs_backend.hpp"

#include <cmath>
#include <iostream>

namespace
{
    constexpr double kOutOfBoundsTrialCost = 1.0e12;            // 越界试探代价

    struct CubicBasis       // 三次基函数
    {
        Eigen::Vector4d position;               // 位置基函数
        Eigen::Vector4d first_derivative;       // 一阶导数基函数（代表速度），把 4 个控制点乘以这 4 个权重，就能得到曲线在当前参数 s 处的 切线方向/速度 P′(s)
        Eigen::Vector4d second_derivative;      // 二阶导数基函数（代表加速度），把 4 个控制点乘以这 4 个权重，就能得到曲线在当前参数 s 处的 加速度/曲率变化 P′′(s)
    };

    // 三次 B 样条基函数实现（s 不同，四个点的权重也不同）
    CubicBasis evaluateCubicBasis(const double s)
    {
        const double s2 = s * s;
        const double s3 = s2 * s;
        const double one_minus_s = 1.0 - s;

        CubicBasis basis;

        basis.position <<
            one_minus_s * one_minus_s * one_minus_s / 6.0,
            (3.0 * s3 - 6.0 * s2 + 4.0) / 6.0,
            (-3.0 * s3 + 3.0 * s2 + 3.0 * s + 1.0) / 6.0,
            s3 / 6.0;

        basis.first_derivative <<
            -0.5 * one_minus_s * one_minus_s,
            1.5 * s2 - 2.0 * s,
            -1.5 * s2 + s + 0.5,
            0.5 * s2;

        basis.second_derivative <<
            one_minus_s,
            3.0 * s - 2.0,
            1.0 - 3.0 * s,
            s;

        return basis;
    }

}  // namespace

namespace path_optimizer
{

void LbfgsBackend::initialize(const LbfgsBackendConfig & config)
{
    config_ = config;
}

int LbfgsBackend::runLbfgsOnce(
    const Eigen::MatrixXd & start_ctrl_pts,
    OptimizationContext & context,
    int max_iterations,
    double & final_cost) const
{
    context.working_ctrl_pts = start_ctrl_pts;

    Eigen::VectorXd x = flattenInnerControlPoints(
        context.working_ctrl_pts, context.start_idx, context.end_idx);

    lbfgs::lbfgs_parameter_t params;
    params.mem_size = context.config.memory_size;   
    params.max_iterations = max_iterations;
    params.g_epsilon = context.config.g_epsilon;
    params.past = context.config.past;
    params.delta = context.config.delta;
    params.max_linesearch = context.config.max_linesearch;

    const int ret = lbfgs::lbfgs_optimize(
        x, final_cost,
        LbfgsBackend::costFunction, nullptr,
        LbfgsBackend::monitorProgress,
        &context,     // 最关键的：原来传 this，现在传本次求解的 context 
        params);

    assignInnerControlPoints(x, context.start_idx, context.end_idx, context.working_ctrl_pts);

    return ret;
}

// 评估当前的迭代结果；输入 优化后的（一维）坐标，计算当前的 cost 和梯度
double LbfgsBackend::evaluate(
    OptimizationContext & context,
    const Eigen::VectorXd & x,
    Eigen::VectorXd & g) const
{
    // 已判定硬失败
    if (context.invalid_evaluation) 
    {
        g.setZero(context.variable_num);
        return std::numeric_limits<double>::infinity();         // 返回无穷大代价
    }

    // 一维 -> 二维，写进 context.working_ctrl_pts
    assignInnerControlPoints(
        x, context.start_idx, context.end_idx, context.working_ctrl_pts);

    Eigen::MatrixXd grad_mat = Eigen::MatrixXd::Zero(
        context.working_ctrl_pts.rows(), context.working_ctrl_pts.cols());

    const double cost =
        computeCostAndGradient(context, context.working_ctrl_pts, grad_mat);

    // 二维梯度 -> 一维
    g.resize(context.variable_num);
    int k = 0;
    for (int i = context.start_idx; i < context.end_idx; ++i) {
        g(k++) = grad_mat(0, i);
        g(k++) = grad_mat(1, i);
    }

    // 记录本次求解见过的最好结果
    if (!context.invalid_evaluation && !context.trial_out_of_bounds && std::isfinite(cost) && cost < context.best_finite_cost)
    {
        context.best_finite_cost = cost;
        context.best_finite_ctrl_pts = context.working_ctrl_pts;
    }
    return cost;
}

double LbfgsBackend::costFunction(
    void * instance,
    const Eigen::VectorXd & x,
    Eigen::VectorXd & g)                // g 是需要求解的梯度
{
    auto * context = static_cast<OptimizationContext *>(instance);          // 用到了 LbfgsBackend* 指针
    return context->backend->evaluate(*context, x, g);
}

int LbfgsBackend::monitorProgress(
    void * instance,
    const Eigen::VectorXd & x,
    const Eigen::VectorXd & g,
    const double fx,
    const double step,
    const int k,
    const int ls)
{
    (void)x; (void)step; (void)ls;

    (void)g; (void)fx; (void)k;

    auto * context = static_cast<OptimizationContext *>(instance);

    // 硬失败（采样点出地图等）：返回非 0，lbfgs 会立刻终止本次求解
    if (context->invalid_evaluation) 
    {
        return 1;
    }
    // 外部请求取消：同样返回非 0 终止
    if (context->cancel_requested != nullptr &&
        context->cancel_requested->load(std::memory_order_relaxed)) 
    {
        context->cancelled = true;
        context->failure_reason = "Cancelled by caller.";
        return 1;
    }

    // std::cout << "[LbfgsBackend] iter = " << k
    //           << ", cost = " << fx
    //           << ", grad_inf_norm = " << g.cwiseAbs().maxCoeff()
    //           << std::endl;
    return 0;
}

// 核心 optimize 函数，调用 LBFGS-Lite 求解库
/*
┌─────────────────────────────────────────────────────────┐
│ LbfgsBackend::optimize(request)          [const 成员函数]│
└─────────────────────────────────────────────────────────┘
   │
   ├─► computeCostAndGradient(context, 初始点, init_grad, print="Initial")
   │       ├─► computeSmoothnessCost()
   │       ├─► computeDistanceCost()      ──► esdf->distance_and_gradient()
   │       ├─► computeReferenceCost()
   │       └─► computeCurvatureCost()
   │
   ├─► runLbfgsOnce(start_ctrl_pts, context, max_iter, final_cost)
   │       │
   │       ├─► flattenInnerControlPoints()      // 二维 → 一维 x
   │       │
   │       ├─► lbfgs::lbfgs_optimize(x, final_cost,
   │       │              costFunction,  nullptr,
   │       │              monitorProgress, &context, params)
   │       │       │
   │       │       │   【库内部反复回调】
   │       │       ├─► LbfgsBackend::costFunction(&context, x, g)   ×N 次
   │       │       │       └─► evaluate(context, x, g)
   │       │       │               ├─► assignInnerControlPoints()   // 一维 → 二维
   │       │       │               ├─► computeCostAndGradient(...)  // 同上四项
   │       │       │               │       └─► (四个代价子函数 + ESDF 查询)
   │       │       │               └─► (更新 best_finite_cost / best_finite_ctrl_pts)
   │       │       │
   │       │       └─► LbfgsBackend::monitorProgress(&context,...)  ×每轮 1 次
   │       │               └─► (检查 invalid_evaluation / cancel_requested)
   │       │
   │       └─► assignInnerControlPoints()       // 最终 x 写回 working_ctrl_pts
   │
   ├─► computeCostAndGradient(context, 最优解, ..., print="Final")  // 仅打印明细
   │
   └─► 返回 OptimizeResult { success, used_fallback, ctrl_pts, failure_reason }

*/
OptimizeResult LbfgsBackend::optimize(const OptimizeRequest & request) const
{
    OptimizeResult result;

    // ---------- 输入检查（对应原来的前几个 if） ----------
    if (request.initial_ctrl_pts.cols() < request.degree + 1) 
    {
        result.failure_reason = "Too few control points for the requested degree.";
        return result;
    }
    if (!request.esdf)
    {
        result.failure_reason = "Null ESDF snapshot.";
        return result;
    }

    // ★★★ 构造本次求解的 context（栈上局部变量，函数返回即销毁） ★★★
    OptimizationContext context;

    context.backend = this;
    context.config = config_;                       // 配置快照
    context.degree = request.degree;
    context.use_lbfgs = request.use_lbfgs;
    context.esdf = request.esdf;
    context.cancel_requested = request.cancel_requested;

    context.initial_ctrl_pts = request.initial_ctrl_pts;
    context.reference_ctrl_pts = request.initial_ctrl_pts;   // 参考点 = 初始点
    context.working_ctrl_pts = request.initial_ctrl_pts;

    const int control_point_num = static_cast<int>(request.initial_ctrl_pts.cols());
    context.start_idx = context.config.fixed_boundary_control_points;
    context.end_idx = control_point_num - context.config.fixed_boundary_control_points;
    context.variable_num = 2 * (context.end_idx - context.start_idx);

    if (context.end_idx <= context.start_idx) 
    {
        result.failure_reason = "No inner control points to optimize.";
        return result;
    }

    // ---------- 简单梯度下降分支----------
    if (!context.use_lbfgs) 
    {
        const double step = 0.01;
        Eigen::MatrixXd ctrl = context.initial_ctrl_pts;   

        for (int iter = 0; iter < context.config.max_iterations; ++iter) 
        {
            Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(ctrl.rows(), ctrl.cols());
            computeCostAndGradient(context, ctrl, grad);

            if (context.invalid_evaluation) 
            {            
                result.failure_reason = context.failure_reason;
                return result;
            }

            if (context.trial_out_of_bounds)
            {
                result.failure_reason =
                    "Gradient-descent candidate moved outside the valid ESDF map.";
                return result;
            }

            if (context.cancel_requested &&
                context.cancel_requested->load(std::memory_order_acquire))
            {
                result.failure_reason = "Cancelled by caller.";
                return result;
            }

            double grad_norm = 0.0;
            for (int i = context.start_idx; i < context.end_idx; ++i) 
            {
                ctrl.col(i) -= step * grad.col(i);
                grad_norm += grad.col(i).squaredNorm();
            }
            if (std::sqrt(grad_norm) < context.config.g_epsilon) 
            {
                break;
            }
        }
        result.success = true;
        result.ctrl_pts = ctrl;
        return result;
    }

    // ---------- LBFGS 分支 ----------
    Eigen::MatrixXd init_grad = Eigen::MatrixXd::Zero(
        context.initial_ctrl_pts.rows(), context.initial_ctrl_pts.cols());
    const double init_cost = computeCostAndGradient(context, context.initial_ctrl_pts, init_grad, true, "Initial");

    if (context.invalid_evaluation) 
    {
        result.failure_reason = context.failure_reason;
        return result;
    }
    if (context.trial_out_of_bounds)
    {
        result.failure_reason =
            "Initial B-spline contains samples outside the valid ESDF map.";
        return result;
    }

    if (!std::isfinite(init_cost))
    {
        result.failure_reason =
            "Initial B-spline cost is NaN or Inf.";
        return result;
    }

    context.initial_cost = init_cost;
    context.best_finite_cost = init_cost;
    context.best_finite_ctrl_pts = context.initial_ctrl_pts;

    // ---------- 只执行一次 LBFGS ----------
    double final_cost = std::numeric_limits<double>::infinity();

    const int ret = runLbfgsOnce(
        context.initial_ctrl_pts,
        context,
        context.config.max_iterations,
        final_cost);

    const std::string solver_message = lbfgs::lbfgs_strerror(ret);

    std::cout
        << "[LbfgsBackend] LBFGS ret = " << ret
        << " (" << solver_message << ")"
        << ", final_cost = " << final_cost
        << ", best_cost = " << context.best_finite_cost
        << std::endl;

    // 调用者主动取消。
    if (context.cancelled)
    {
        result.failure_reason = context.failure_reason;
        return result;
    }

    // 真正不可恢复的目标函数错误。
    if (context.invalid_evaluation)
    {
        result.failure_reason = context.failure_reason;
        return result;
    }

    // 正常收敛，或者满足 past/delta 停止条件
    if (ret == lbfgs::LBFGS_CONVERGENCE || ret == lbfgs::LBFGS_STOP)
    {
        result.success = true;
        result.ctrl_pts = context.working_ctrl_pts;

        {
            Eigen::MatrixXd final_grad = Eigen::MatrixXd::Zero(context.working_ctrl_pts.rows(), context.working_ctrl_pts.cols());
            computeCostAndGradient(context, context.working_ctrl_pts, final_grad, true, "Final");
        }

        return result;
    }

    const bool has_finite_best = 
        std::isfinite(context.best_finite_cost) && 
        context.best_finite_ctrl_pts.size() != 0;

    if (!has_finite_best)
    {
        result.failure_reason =
            "LBFGS stopped: " + solver_message +
            " No finite fallback candidate exists.";
        return result;
    }

    // 即使出现 -1009 等状态，只要历史最好解比初始解改善至少 1%，就直接接受历史最好解，不再用相同配置重新求解一次。
    if (context.best_finite_cost < context.initial_cost * 0.99)
    {
        std::cout
            << "[LbfgsBackend] LBFGS did not converge normally, "
            << "but the best valid candidate improves the initial path."
            << " init_cost = " << context.initial_cost
            << ", best_cost = " << context.best_finite_cost
            << std::endl;

        result.success = true;
        result.used_fallback = true;
        result.failure_reason =
            "LBFGS stopped: " + solver_message;
        result.ctrl_pts = context.best_finite_ctrl_pts;

        {
            Eigen::MatrixXd final_grad = Eigen::MatrixXd::Zero(context.best_finite_ctrl_pts.rows(), context.best_finite_ctrl_pts.cols());
            computeCostAndGradient(context, context.best_finite_ctrl_pts, final_grad, true, "Final");
        }

        return result;
    }

    // 没有产生明显改善，发布初始解/历史最好解，并明确报告原因。
    result.success = false;
    result.used_fallback = true;
    result.failure_reason =
        "LBFGS stopped: " + solver_message +
        " No meaningful improvement over the initial path.";
    result.ctrl_pts = context.best_finite_ctrl_pts;

    {
        Eigen::MatrixXd final_grad = Eigen::MatrixXd::Zero(context.best_finite_ctrl_pts.rows(), context.best_finite_ctrl_pts.cols());
        computeCostAndGradient(context, context.best_finite_ctrl_pts, final_grad, true, "Final");
    }

    return result;
}

// 计算总代价和总梯度
double LbfgsBackend::computeCostAndGradient(
    OptimizationContext & context,
    const Eigen::MatrixXd & ctrl_pts,
    Eigen::MatrixXd & grad,
    bool print_breakdown,
    const std::string & phase) const
{
    // 每一次函数评估都重新清零
    context.trial_out_of_bounds = false;

    grad.setZero();

    Eigen::MatrixXd grad_smooth = Eigen::MatrixXd::Zero(ctrl_pts.rows(), ctrl_pts.cols());
    Eigen::MatrixXd grad_dist = Eigen::MatrixXd::Zero(ctrl_pts.rows(), ctrl_pts.cols());
    Eigen::MatrixXd grad_refe = Eigen::MatrixXd::Zero(ctrl_pts.rows(), ctrl_pts.cols());
    Eigen::MatrixXd grad_cur = Eigen::MatrixXd::Zero(ctrl_pts.rows(), ctrl_pts.cols());

    const double smooth_cost = computeSmoothnessCost(ctrl_pts, grad_smooth);
    const double dist_cost = computeDistanceCost(context, ctrl_pts, grad_dist);

    // 真正不可恢复的错误
    if (context.invalid_evaluation)
    {
        grad.setZero();
        return std::numeric_limits<double>::infinity();
    }

    // 当前线搜索试探步越出地图
    // 返回一个大而有限的值，让线搜索缩短步长后重新尝试
    if (context.trial_out_of_bounds)
    {
        grad.setZero();
        return kOutOfBoundsTrialCost;
    }

    double reference_cost = 0.0;
    double curvature_cost = 0.0;

    if (context.config.if_reference_cost)
    {
        reference_cost = computeReferenceCost(context, ctrl_pts, grad_refe);
    }

    if (context.config.if_curvature_cost)
    {
        curvature_cost = computeCurvatureCost(context, ctrl_pts, grad_cur);
    }

    const LbfgsBackendConfig & cfg = context.config;

    grad =
        cfg.lambda_smooth * grad_smooth +
        cfg.lambda_distance * grad_dist +
        cfg.lambda_reference * grad_refe +
        cfg.lambda_curvature * grad_cur;

    // ---------------- 打印代价明细 ----------------
    if (print_breakdown) 
    {
        double w_smooth = smooth_cost * cfg.lambda_smooth;
        double w_dist   = dist_cost * cfg.lambda_distance;
        double w_refe   = reference_cost * cfg.lambda_reference;
        double w_cur    = curvature_cost * cfg.lambda_curvature;

        // 计算内部控制点 [start_idx, end_idx) 梯度的无穷范数
        auto get_inf_norm = [&](const Eigen::MatrixXd& g) -> double 
        {
            if (context.start_idx >= context.end_idx) return 0.0;
            return g.block(0, context.start_idx, g.rows(), context.end_idx - context.start_idx).cwiseAbs().maxCoeff();
        };

        double g_smooth_inf = get_inf_norm(cfg.lambda_smooth * grad_smooth);
        double g_dist_inf   = get_inf_norm(cfg.lambda_distance * grad_dist);
        double g_refe_inf   = get_inf_norm(cfg.lambda_reference * grad_refe);
        double g_cur_inf    = get_inf_norm(cfg.lambda_curvature * grad_cur);

        std::cout << "[" << phase << " Cost Breakdown]\n";
        std::cout << "smooth   raw: " << smooth_cost   << " weighted: " << w_smooth << " grad_inf: " << g_smooth_inf << "\n";
        std::cout << "distance raw: " << dist_cost     << " weighted: " << w_dist   << " grad_inf: " << g_dist_inf   << "\n";
        std::cout << "reference raw: " << reference_cost << " weighted: " << w_refe   << " grad_inf: " << g_refe_inf   << "\n";
        std::cout << "curvature raw: " << curvature_cost << " weighted: " << w_cur    << " grad_inf: " << g_cur_inf    << "\n";
    }
    // ----------------------------------------------------

    return
        cfg.lambda_smooth * smooth_cost +
        cfg.lambda_distance * dist_cost +
        cfg.lambda_reference * reference_cost +
        cfg.lambda_curvature * curvature_cost;
}

// 计算平滑项的代价以及梯度
double LbfgsBackend::computeSmoothnessCost(
    const Eigen::MatrixXd & ctrl_pts,
    Eigen::MatrixXd & grad) const
{
    double cost = 0.0;
    // 虽然 P1,P2 是固定控制点，但这些项仍然会对可优化的 P3,P4 产生梯度，因此从 0 开始
    // 同时为防止越界要小于 ctrl_pts.cols() - 2
    for (int i = 0; i < ctrl_pts.cols() - 2; ++i)
    {
        // 二阶差分计算加速度
        Eigen::Vector2d acc =
            ctrl_pts.col(i + 2) - 2.0 * ctrl_pts.col(i + 1) + ctrl_pts.col(i);

        // 代价函数为二阶差分的平方和
        cost += acc.squaredNorm();          // 加速度的平方

        // 计算梯度（总代价分别对三个点（Pi+2、 Pi+1、 Pi）求导，得到第 i 个点对总代价的偏导数）
        // 总代价就是 (P(i+2)​−2P(i+1)​+Pi)^2，就是分别对 Pi+2、 Pi+1、 Pi 求导
        /*
        链式求导：
            J = a²   a = P(i+2) - 2P(i+1) + P(i)
            E.g. ∂J / ∂P(i+1) = ∂J/∂a * ∂a/∂P(i+1) = 2a * (-2) = -4a
        */
       // 注意，此处是累加赋值而不是覆盖赋值；因为一个点的梯度受周围好几个点的影响
        Eigen::Vector2d temp = 2.0 * acc;       // acc ^ 2 求导得到 2 * acc
        grad.col(i)     += temp;
        grad.col(i + 1) += -2.0 * temp;
        grad.col(i + 2) += temp;
    }

    return cost;
}

// 计算 ESDF 距离项的代价以及梯度
double LbfgsBackend::computeDistanceCost(
    OptimizationContext & context,
    const Eigen::MatrixXd & ctrl_pts,
    Eigen::MatrixXd & grad) const
{
    if (context.degree != 3) 
    {                       
        context.invalid_evaluation = true;
        context.failure_reason =
            "Curve-sampled ESDF cost currently supports cubic B-spline only.";
        return 0.0;
    }

    const int control_point_count = static_cast<int>(ctrl_pts.cols());

    // 每 4 个控制点生成一个曲线点，那么 k 个控制点可以生成 k-3 个曲线点
    const int span_count = control_point_count - context.degree;
    const int samples_per_span = context.config.optimization_samples_per_span;              // 每段样条的采样个数

    if (span_count <= 0 || samples_per_span <= 0)
    {
        return 0.0;
    }

    // 防止增大采样数以后，距离项的总权重也跟着线性增大。
    const double sample_weight = 1.0 / static_cast<double>(samples_per_span);

    double cost = 0.0;

    for (int span = 0; span < span_count; ++span)
    {
        for (int sample = 0; sample < samples_per_span; ++sample)
        {
            // 每个采样区间取中点，避免相邻 span 重复查询节点。
            const double s = (static_cast<double>(sample) + 0.5) / static_cast<double>(samples_per_span);

            const CubicBasis basis = evaluateCubicBasis(s);         // 计算基函数，获取四个控制点权重系数

            // 2x4 * 4x1 = 2x1
            const Eigen::Vector2d curve_point = ctrl_pts.middleCols(span, 4) * basis.position;       // 四个控制点计算出一个曲线点

            field_map_builder::EsdfQueryResult result;

            if (!context.esdf->distance_and_gradient(curve_point.x(), curve_point.y(), result))
            {
                // 这通常只是 L-BFGS 线搜索试探的步长过大，不能将整个优化永久标记为失败。
                context.trial_out_of_bounds = true;

                // 清除当前距离项已经累计的部分梯度。
                grad.setZero();

                return kOutOfBoundsTrialCost;
            }

            const double distance = static_cast<double>(result.distance);

            if (distance >= context.config.obstacle_dist)         // 只有大于 obstacle_dist_ 才会产生代价
            {
                continue;
            }

            // 梯度（ESDF 值的基础上对 x、y 求导，梯度方向指向距离增加最快的方向（即远离障碍物的方向））
            const Eigen::Vector2d distance_gradient(static_cast<double>(result.gradient_x), static_cast<double>(result.gradient_y));

            const double error = context.config.obstacle_dist - distance;

            cost += sample_weight * error * error;

            // J是总代价，p是曲线点，d是距离，则距离对位置求导，也就是∂d/∂p，就得到梯度（distance_gradient）
            // 那么此处就是要求 ∂J/∂p = ∂J/∂d * ∂d/∂p = -2 * error * distance_gradient
            const Eigen::Vector2d cost_gradient_at_curve_point = -2.0 * sample_weight * error * distance_gradient;

            // 通过链式法则，把曲线点梯度分给四个控制点。
            for (int local = 0; local < 4; ++local)
            {
                grad.col(span + local) += basis.position(local) * cost_gradient_at_curve_point;
            }
        }
    }

    return cost;
}

double LbfgsBackend::computeReferenceCost(
    const OptimizationContext & context,
    const Eigen::MatrixXd & ctrl_pts,
    Eigen::MatrixXd & grad) const
{
    if (context.reference_ctrl_pts.rows() != ctrl_pts.rows() || context.reference_ctrl_pts.cols() != ctrl_pts.cols())
    {
        return 0.0;
    }

    const int span_count = static_cast<int>(ctrl_pts.cols()) - context.degree;

    const int samples_per_span = context.config.optimization_samples_per_span;

    if (span_count <= 0 || samples_per_span <= 0)
    {
        return 0.0;
    }

    const double sample_weight = 1.0 / static_cast<double>(samples_per_span);

    double cost = 0.0;

    for (int span = 0; span < span_count; ++span)
    {
        for (int sample = 0; sample < samples_per_span; ++sample)
        {
            const double s = (static_cast<double>(sample) + 0.5) / static_cast<double>(samples_per_span);

            const CubicBasis basis = evaluateCubicBasis(s);

            // 通过 ctrl_pts、reference_ctrl_pts_ 计算曲线点 
            const Eigen::Vector2d current_point = ctrl_pts.middleCols(span, 4) * basis.position;
            const Eigen::Vector2d reference_point = context.reference_ctrl_pts.middleCols(span, 4) * basis.position;

            const Eigen::Vector2d error = current_point - reference_point;
            const double distance = error.norm();
            const double excess = distance - context.config.reference_tolerance;        // 过量的部分

            if (excess <= 0.0 || distance <= 1.0e-8)
            {
                continue;
            }

            cost += sample_weight * excess * excess;

            // J是总代价函数，那么J=w⋅excess^2，excess=|error|−tolerance，error=p−pref
            // 现在要求 ∂J/∂p，则 ∂J/∂p = ∂J/∂excess * ∂excess/∂error * ∂error/∂p
            // 其中，∂excess/∂error 部分由于error的正负不确定，因此用单位向量表示，也就是 error/|error|
            // 则 ∂J/∂p = 2 * sample_weight * excess * error/|error| * 1 = 2.0 * sample_weight * excess / distance * error
            const Eigen::Vector2d point_gradient = 2.0 * sample_weight * excess / distance * error;

            for (int local = 0; local < 4; ++local)
            {
                grad.col(span + local) += basis.position(local) * point_gradient;
            }
        }
    }

    return cost;
}

double LbfgsBackend::computeCurvatureCost(
    const OptimizationContext & context,
    const Eigen::MatrixXd & ctrl_pts,
    Eigen::MatrixXd & grad) const
{
    const int span_count = static_cast<int>(ctrl_pts.cols()) - context.degree;
    const int samples_per_span = context.config.optimization_samples_per_span;

    const double sample_weight = 1.0 / static_cast<double>(samples_per_span);

    const double epsilon_squared = context.config.tangent_epsilon * context.config.tangent_epsilon;

    double cost = 0.0;

    /*
    对于参数曲线 P(s)，其曲率公式：
        κ = ∣Px′​Py′′​−Py′​Px′′​∣​ / (Px′^​2+Py′^​2)^(3/2) 

    Px′^​2+Py′^​2 可视作速度大小
    */

    for (int span = 0; span < span_count; ++span)
    {
        const Eigen::Matrix<double, 2, 4> local_ctrl_pts = ctrl_pts.middleCols(span, 4);

        for (int sample = 0; sample < samples_per_span; ++sample)
        {
            const double s = (static_cast<double>(sample) + 0.5) / static_cast<double>(samples_per_span);

            const CubicBasis basis = evaluateCubicBasis(s);

            // tangent = P′(s)，记作 t=(tx,ty)^T
            const Eigen::Vector2d tangent = local_ctrl_pts * basis.first_derivative;        // 速度（切向量）
            // second_derivative =P′′(s)，记作 d=(dx,dy)^T
            const Eigen::Vector2d second_derivative = local_ctrl_pts * basis.second_derivative;     // 加速度

            const double tangent_norm_squared = tangent.squaredNorm();      // 速度大小

            // 曲线退化时曲率没有良好定义。
            if (tangent_norm_squared < epsilon_squared)
            {
                continue;
            }

            const double regularized_speed_squared = tangent_norm_squared + epsilon_squared;

            const double inverse_speed = 1.0 / std::sqrt(regularized_speed_squared);    // q^(-1/2)
            const double inverse_speed_cubed = inverse_speed / regularized_speed_squared;       // q^(-3/2)
            const double inverse_speed_fifth = inverse_speed_cubed / regularized_speed_squared;     // q^(-5/2)

            // cross = txdy − tydx​（分子，带正负号，表示左转/右转）
            // Px′​Py′′​−Py′​Px′′
            const double cross = tangent.x() * second_derivative.y() - tangent.y() * second_derivative.x();
            const double curvature = cross * inverse_speed_cubed;    // κ = cross / q^(3/2)

            const double excess = std::abs(curvature) - context.config.max_curvature;     // 超出曲率限制的部分

            if (excess <= 0.0)
            {
                continue;
            }

            cost += sample_weight * excess * excess;        // 代价计算

            // 下边计算梯度，目标是 ∂J/∂C（C是控制点）
            /*
            对每个采样点，代价为： J=w⋅excess^2,excess=∣κ∣−κmax
            κ是曲率，公式见上边注释
            则 ∂J/∂C = ∂J/∂κ * ∂κ/∂d * ∂d/∂C
            或 ∂J/∂C = ∂J/∂κ * ∂κ/∂t * ∂t/∂C
            */

           // ∂J/∂κ = 2w⋅excess⋅∂∣κ∣/∂κ ​= 2w⋅excess⋅sign(κ)
            const double curvature_sign = curvature >= 0.0 ? 1.0 : -1.0;        // ∂∣κ∣/∂κ
            const double cost_derivative_curvature = 2.0 * sample_weight * excess * curvature_sign;

            // ∂κ/∂t（见 /image 下截图）
            const Eigen::Vector2d curvature_derivative_tangent =
                Eigen::Vector2d(second_derivative.y(), -second_derivative.x()) *inverse_speed_cubed
                 - 3.0 * cross * tangent * inverse_speed_fifth;

            // ∂κ/∂d（见 /image 下截图）
            const Eigen::Vector2d curvature_derivative_second =
                Eigen::Vector2d(-tangent.y(), tangent.x()) * inverse_speed_cubed;

            // ∂J/∂t ​= ∂J/∂κ​⋅∂κ/∂t​
            // ∂J/∂d ​= ∂J/∂κ​⋅∂κ/∂d​
            const Eigen::Vector2d cost_derivative_tangent =
                cost_derivative_curvature * curvature_derivative_tangent;
            const Eigen::Vector2d cost_derivative_second =
                cost_derivative_curvature * curvature_derivative_second;

            for (int local = 0; local < 4; ++local)
            {
                // ∂t/∂C = basis.first_derivative
                // ∂d/∂C = basis.second_derivative
                // t 和 d 都是控制点的线性组合，汇总两段梯度，按照权重分回控制点
                grad.col(span + local) +=
                    basis.first_derivative(local) * cost_derivative_tangent
                    + basis.second_derivative(local) * cost_derivative_second;
            }
        }
    }

    return cost;
}

// 将二维数组拍扁为一维数组（交给 LBFGS 优化）
Eigen::VectorXd LbfgsBackend::flattenInnerControlPoints(
    const Eigen::MatrixXd & ctrl_pts,
    int start_idx,
    int end_idx) const
{
    const int var_num = 2 * (end_idx - start_idx);
    Eigen::VectorXd x(var_num);

    int k = 0;
    for (int i = start_idx; i < end_idx; ++i)
    {
        x(k++) = ctrl_pts(0, i);
        x(k++) = ctrl_pts(1, i);
    }

    return x;
}
// 将一维数组还原为二维数组（用于绘制最终路径）
void LbfgsBackend::assignInnerControlPoints(
    const Eigen::VectorXd & x,
    int start_idx,
    int end_idx,
    Eigen::MatrixXd & ctrl_pts) const
{
    int k = 0;
    for (int i = start_idx; i < end_idx; ++i)
    {
        ctrl_pts(0, i) = x(k++);
        ctrl_pts(1, i) = x(k++);
    }
}

} // namespace path_optimizer