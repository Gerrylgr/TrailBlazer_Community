#include "path_optimizer/lbfgs_bspline_optimizer.hpp"

#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;

namespace path_optimizer
{
    void LbfgsBsplineOptimizing::configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name) 
    {
        node_ = parent;
        plugin_name_ = plugin_name;

        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        reset_runtime_state();

        load_parameters(node);

        create_publishers(node);

        LbfgsBackendConfig backend_config;
        backend_config.if_reference_cost = if_reference_cost_;
        backend_config.if_curvature_cost = if_curvature_cost_;
        backend_config.lambda_smooth = lambda_smooth_;
        backend_config.lambda_distance = lambda_distance_;
        backend_config.obstacle_dist = obstacle_dist_;
        backend_config.lambda_reference = lambda_reference_;
        backend_config.reference_tolerance = reference_tolerance_;
        backend_config.lambda_curvature = lambda_curvature_;
        backend_config.max_curvature = max_curvature_;
        backend_config.tangent_epsilon = tangent_epsilon_;
        backend_config.max_iterations = max_iterations_;
        backend_config.g_epsilon = g_epsilon_;
        backend_config.fixed_boundary_control_points = fixed_boundary_control_points_;
        backend_config.optimization_samples_per_span = optimization_samples_per_span_;
        backend_config.memory_size = memory_size_;  
        backend_config.past = lbfgs_past_;
        backend_config.delta = lbfgs_delta_;
        backend_config.max_linesearch = max_linesearch_; 
        lbfgs_backend_.initialize(backend_config);

        RCLCPP_INFO(
            node->get_logger(),
            "LbfgsBsplineOptimizing plugin '%s' configured.",
            plugin_name_.c_str()
        );
    }

    void LbfgsBsplineOptimizing::load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const std::string p = plugin_name_ + ".";

        if_silence_ = declare_or_get_parameter<bool>(node, p + "if_silence", false);
        if_debug_ = declare_or_get_parameter<bool>(node, p + "if_debug", true);

        init_bspline_topic_ = declare_or_get_parameter<std::string>(node, p + "init_bspline_topic", "/init_bspline_path");
        optimized_path_topic_ = declare_or_get_parameter<std::string>(node, p + "optimized_path_topic", "/optimized_global_path");
        input_esdf_topic_ = declare_or_get_parameter<std::string>(node, p + "input_esdf_topic", "/esdf_map");
        original_path_topic_ = declare_or_get_parameter<std::string>(node, p + "original_path_topic", "/original_global_path");

        input_path_resolution_ = declare_or_get_parameter<double>(node, p + "input_path_resolution", 0.1);
        output_path_resolution_ = declare_or_get_parameter<double>(node, p + "output_path_resolution", 0.05);
        output_parameter_step_ = declare_or_get_parameter<double>(node, p + "output_parameter_step", 0.1);
        
        if_reference_cost_ = declare_or_get_parameter<bool>(node, p + "if_reference_cost", true);
        if_curvature_cost_ = declare_or_get_parameter<bool>(node, p + "if_curvature_cost", true);
        
        lambda_smooth_ = declare_or_get_parameter<double>(node, p + "lambda_smooth", 1.0);
        lambda_distance_ = declare_or_get_parameter<double>(node, p + "lambda_distance", 1.0);
        obstacle_dist_ = declare_or_get_parameter<double>(node, p + "obstacle_dist", 0.8);
        lambda_reference_ = declare_or_get_parameter<double>(node, p + "lambda_reference", 1.0);
        reference_tolerance_ = declare_or_get_parameter<double>(node, p + "reference_tolerance", 0.2);
        lambda_curvature_ = declare_or_get_parameter<double>(node, p + "lambda_curvature", 1.0);
        max_curvature_ = declare_or_get_parameter<double>(node, p + "max_curvature", 1.0);
        tangent_epsilon_ = declare_or_get_parameter<double>(node, p + "tangent_epsilon", 1e-4);

        max_iterations_ = declare_or_get_parameter<int>(node, p + "max_iterations", 500);
        if_lbfgs_ = declare_or_get_parameter<bool>(node, p + "if_lbfgs", true);
        g_epsilon_ = declare_or_get_parameter<double>(node, p + "g_epsilon", 1e-3);
        fixed_boundary_control_points_ = declare_or_get_parameter<int>(node, p + "fixed_boundary_control_points", 3);
        optimization_samples_per_span_ = declare_or_get_parameter<int>(node, p + "optimization_samples_per_span", 5);

        memory_size_ = declare_or_get_parameter<int>(node, p + "memory_size", 100);
        lbfgs_past_ = declare_or_get_parameter<int>(node, p + "lbfgs_past", 5);
        lbfgs_delta_ = declare_or_get_parameter<double>(node, p + "lbfgs_delta", 1.0e-4);
        max_linesearch_ = declare_or_get_parameter<int>(node, p + "max_linesearch", 20);

        RCLCPP_INFO(
            node->get_logger(),
            "Params-loading accomplished!!!"
        );
    }

    void LbfgsBsplineOptimizing::create_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        bspline_init_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
            init_bspline_topic_,
            path_qos
        );

        optimized_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
            optimized_path_topic_,
            path_qos
        );
    }

    void LbfgsBsplineOptimizing::reset_runtime_state()
    {
        lifecycle_active_.store(false);

        esdf_buffer_.clear();

        cancel_flag_.store(false, std::memory_order_release);
    }

    void LbfgsBsplineOptimizing::activate()
    {
        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        if (bspline_init_path_pub_)
            bspline_init_path_pub_->on_activate();

        if (optimized_path_pub_)
            optimized_path_pub_->on_activate();

        lifecycle_active_.store(true);

        create_subscriptions(node);
        // create_timers(node);
        // create_services(node);

        RCLCPP_INFO(
            node->get_logger(),
            "LbfgsBsplineOptimizing plugin '%s' activated.",
            plugin_name_.c_str());
    }

    void LbfgsBsplineOptimizing::create_subscriptions(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const auto esdf_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        const auto original_path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        esdf_map_sub_ = node->create_subscription<trailblazer_map_interfaces::msg::EsdfMap>(
            input_esdf_topic_,
            esdf_qos,
            std::bind(&LbfgsBsplineOptimizing::esdf_map_callback, this, std::placeholders::_1)
        );

        path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
            original_path_topic_,
            original_path_qos,
            std::bind(&LbfgsBsplineOptimizing::originalPathCallback, this, std::placeholders::_1)
        );
    }

    void LbfgsBsplineOptimizing::esdf_map_callback(const trailblazer_map_interfaces::msg::EsdfMap::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        esdf_buffer_.update(msg);
    }

    void LbfgsBsplineOptimizing::originalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        auto node = node_.lock();
        if (!node)
            throw std::runtime_error("Parent node expired.");

        // 尝试拿锁
        std::unique_lock<std::mutex> planning_lock(planning_mutex_, std::try_to_lock);
        if (!planning_lock.owns_lock())
        {
            // 上一次全局规划还没有结束
            return;
        }

        if (!lifecycle_active_.load(std::memory_order_acquire))
            return;

        const auto start_time = std::chrono::steady_clock::now();

        if (msg->poses.size() < 4)
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1500,
                "Input path too short for cubic B-spline."
            );
            return;
        }

        const auto esdf = esdf_buffer_.snapshot();          // 每一轮 optimize 时固定下来 esdf

        if (!esdf)
        {
            RCLCPP_WARN(node->get_logger(), "ESDF map is not ready.");
            return;
        }

        // ros path to eigen path
        std::vector<Eigen::Vector2d> raw_points;
        if (!rosPathToEigenPoints(*msg, raw_points))
        {
            return;
        }

        // resample path
        std::vector<Eigen::Vector2d> coarse_points = path_optimizer::UniformBspline::resamplePolylineByArcLength(raw_points, input_path_resolution_);

        if (coarse_points.size() < 4)
        {
            // 短路径单独处理
            return;
        }

        // 起点终点的加速度为0
        std::vector<Eigen::Vector2d> start_end_derivatives(2, Eigen::Vector2d::Zero());

        // 一阶导（速度）不应该为0,否则曲率可能会无限大导致问题
        start_end_derivatives[0] = (coarse_points[1] - coarse_points[0]) / kKnotInterval;
        start_end_derivatives[1] = (coarse_points.back() - coarse_points[coarse_points.size() - 2]) / kKnotInterval;

        // // 起终点二阶导先保持为零
        // start_end_derivatives[2].setZero();
        // start_end_derivatives[3].setZero(); 

        Eigen::MatrixXd init_ctrl_pts;
        // B样条参数化得到控制点
        bool ok = path_optimizer::UniformBspline::parameterizeToBspline(
            kKnotInterval, coarse_points, start_end_derivatives, init_ctrl_pts);

        if (!ok)
        {
            RCLCPP_WARN(node->get_logger(), "Failed to parameterize B-spline.");
            return;
        }
        else
        {
            if(!if_silence_)
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Parameterize B-spline successful!"
                );
            }
        }

        if (init_ctrl_pts.cols() <= 2 * fixed_boundary_control_points_)
        {
            RCLCPP_DEBUG(
                node->get_logger(),
                "Path is too short to contain free B-spline control points.");

            optimized_path_pub_->publish(sampleToRosPath(raw_points, msg->header));
            return;
        }

        // 还原发布 bspline-path
        path_optimizer::UniformBspline init_traj(init_ctrl_pts, 3, kKnotInterval);
        std::vector<Eigen::Vector2d> init_sampled_pts = init_traj.sampleByArcLength(output_path_resolution_, output_parameter_step_);

        bspline_init_path_pub_->publish(sampleToRosPath(init_sampled_pts, msg->header));

        OptimizeRequest request;
        request.initial_ctrl_pts = init_ctrl_pts;
        request.degree = 3;
        request.use_lbfgs = if_lbfgs_;
        request.esdf = esdf;  
        // request.cancel_requested 留空（nullptr）则不会生效
        request.cancel_requested = &cancel_flag_; 

        const OptimizeResult opt_result = lbfgs_backend_.optimize(request);

        Eigen::MatrixXd optimized_ctrl_pts = init_ctrl_pts;   // 完全失败时的兜底
        if (opt_result.success || opt_result.used_fallback) 
        {
            optimized_ctrl_pts = opt_result.ctrl_pts;
        }

        if (!opt_result.success) 
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "LBFGS backend failed (%s), publish fallback path instead.",
                opt_result.failure_reason.c_str());
        }

        // B样条采样生成最终曲线点
        path_optimizer::UniformBspline optimized_traj(optimized_ctrl_pts, 3, kKnotInterval);
        std::vector<Eigen::Vector2d> optimized_sampled_pts = optimized_traj.sampleByArcLength(output_path_resolution_, output_parameter_step_);

        if (!lifecycle_active_.load(std::memory_order_acquire) || cancel_flag_.load(std::memory_order_acquire))
        {
            return;         // 取消规划
        }

        optimized_path_pub_->publish(sampleToRosPath(optimized_sampled_pts, msg->header));

        const auto time0 = std::chrono::steady_clock::now();
        double cost_ms = std::chrono::duration<double, std::milli>(time0 - start_time).count();

        if(if_debug_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Bspline+LBFGS optimization cost: %.2f ms",
                cost_ms
            );
        }

        if(!if_silence_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Optimized Bspline path published!"
            );
        }

    }

    bool LbfgsBsplineOptimizing::rosPathToEigenPoints(const nav_msgs::msg::Path & path_msg, std::vector<Eigen::Vector2d> & points) const
    {
        points.clear();
        points.reserve(path_msg.poses.size());

        for(auto &pose : path_msg.poses)
        {
            const double x = pose.pose.position.x;
            const double y = pose.pose.position.y;

            if (!std::isfinite(x) || !std::isfinite(y))
            {
                points.clear();
                return false;
            }

            if (points.empty() || (Eigen::Vector2d(x, y) - points.back()).norm() > 1e-8)
            {
                points.emplace_back(x, y);
            }
        }

        return points.size() >= 4;
    }

    nav_msgs::msg::Path LbfgsBsplineOptimizing::sampleToRosPath(const std::vector<Eigen::Vector2d> & sampled_pts, const std_msgs::msg::Header & header) const
    {
        nav_msgs::msg::Path path;
        path.header = header;
        path.poses.reserve(sampled_pts.size());

        for (size_t i = 0; i < sampled_pts.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = header;
            pose.pose.position.x = sampled_pts[i].x();
            pose.pose.position.y = sampled_pts[i].y();
            pose.pose.position.z = 0.0;

            double yaw = 0.0;
            if (sampled_pts.size() >= 2)
            {
                if (i + 1 < sampled_pts.size())
                {
                    const double dx = sampled_pts[i + 1].x() - sampled_pts[i].x();
                    const double dy = sampled_pts[i + 1].y() - sampled_pts[i].y();
                    yaw = std::atan2(dy, dx);
                }
                else
                {
                    // 最后一个点
                    const double dx = sampled_pts[i].x() - sampled_pts[i - 1].x();
                    const double dy = sampled_pts[i].y() - sampled_pts[i - 1].y();
                    yaw = std::atan2(dy, dx);
                }
            }

            pose.pose.orientation.w = std::cos(yaw * 0.5);
            pose.pose.orientation.z = std::sin(yaw * 0.5);
            path.poses.push_back(pose);
        }

        return path;
    }

    void LbfgsBsplineOptimizing::deactivate()
    {
        auto node = node_.lock();

        // 先挡住所有 callback
        lifecycle_active_.store(false, std::memory_order_release);

        path_sub_.reset();
        esdf_map_sub_.reset();

        // 停止 timers

        // 停止本轮 optimize
        cancel_flag_.store(true, std::memory_order_release);

        // 等待正在进行的 originalPathCallback 退出
        {
            std::lock_guard<std::mutex> lock(planning_mutex_);
        }

        // deactivate publishers
        if (bspline_init_path_pub_)
            bspline_init_path_pub_->on_deactivate();

        if (optimized_path_pub_)
            optimized_path_pub_->on_deactivate();

        cancel_flag_.store(false, std::memory_order_release);

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "LbfgsBsplineOptimizing plugin '%s' deactivated.",
                plugin_name_.c_str());
        }
    }

    void LbfgsBsplineOptimizing::cleanup()
    {
        auto node = node_.lock();

        lifecycle_active_.store(false);

        path_sub_.reset();
        esdf_map_sub_.reset();

        // configure 阶段创建的 publishers 全部 reset
        bspline_init_path_pub_.reset();
        optimized_path_pub_.reset();

        reset_runtime_state();

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "LbfgsBsplineOptimizing plugin '%s' cleaned up.",
                plugin_name_.c_str());
        }

        plugin_name_.clear();

        // 最后一件事再丢掉 node weak reference
        node_.reset();
    }
}       // namespace path_optimizer

PLUGINLIB_EXPORT_CLASS(path_optimizer::LbfgsBsplineOptimizing, path_optimizer::PathOptimizerBase)
