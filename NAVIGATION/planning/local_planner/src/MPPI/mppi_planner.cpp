#include "local_planner/MPPI/mppi_planner.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace
{
    std::vector<local_planner::mppi_core::Point2D>sampleFootprintEdges(const std::vector<local_planner::mppi_core::Point2D> &vertices, double sample_step)
    {
        using Point2D = local_planner::mppi_core::Point2D;

        std::vector<Point2D> samples;

        if (vertices.size() < 3)
        {
            return samples;
        }

        sample_step = std::max(sample_step, 1e-3);

        for (std::size_t i = 0; i < vertices.size(); ++i)
        {
            const auto & start = vertices[i];
            const auto & end = vertices[(i + 1) % vertices.size()];

            const double dx = static_cast<double>(end.x - start.x);
            const double dy = static_cast<double>(end.y - start.y);
            const double length = std::hypot(dx, dy);

            const int step_count = std::max(1, static_cast<int>(std::ceil(length / sample_step)));          // 采样次数

            // 不加入末点，避免与下一条边的起点重复
            for (int step = 0; step < step_count; ++step)
            {
                const double ratio = static_cast<double>(step) / static_cast<double>(step_count);
                samples.push_back(Point2D{static_cast<float>(start.x + ratio * dx), static_cast<float>(start.y + ratio * dy)});
            }
        }

        // 增加车体中心作为辅助检测点
        samples.push_back(Point2D{0.0f, 0.0f});

        return samples;
    }

}  // namespace

namespace local_planner
{
    void MPPILocalPlanner::configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name) 
    {
        node_ = parent;
        plugin_name_ = plugin_name;

        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        reset_runtime_state();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_, node, false);

        load_parameters(node);

        createPlannerCore();

        create_publishers(node);

        RCLCPP_INFO(
            node->get_logger(),
            "MPPILocalPlanner plugin '%s' configured.",
            plugin_name_.c_str()
        );
    }

    void MPPILocalPlanner::load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const std::string p = plugin_name_ + ".";

        if_silence_ = declare_or_get_parameter<bool>(node, p + "if_silence", false);
        if_debug_ = declare_or_get_parameter<bool>(node, p + "if_debug", true);

        cmd_vel_topic_ = declare_or_get_parameter<std::string>(node, p + "cmd_vel_topic", "/cmd_vel");
        input_esdf_topic_ = declare_or_get_parameter<std::string>(node, p + "input_esdf_topic", "/esdf_map");
        optimized_path_topic_ = declare_or_get_parameter<std::string>(node, p + "optimized_path_topic", "/optimized_global_path");
        goal_topic_ = declare_or_get_parameter<std::string>(node, p + "goal_topic", "/goal_pose");
        odom_topic_ = declare_or_get_parameter<std::string>(node, p + "odom_topic", "/Odometry_centered");
        mppi_predicted_path_topic_ = declare_or_get_parameter<std::string>(node, p + "mppi_predicted_path_topic", "/mppi_predicted_path");

        control_period_sec_ = declare_or_get_parameter<double>(node, p + "control_period_sec", 0.05);

        odom_timeout_sec_ = declare_or_get_parameter<double>(node, p + "odom_timeout_sec", 1.0);
        esdf_timeout_sec_ = declare_or_get_parameter<double>(node, p + "esdf_timeout_sec", 2.0);
        transform_timeout_sec_ = declare_or_get_parameter<double>(node, p + "transform_timeout_sec", 0.05);
        compensate_state_latency_ = declare_or_get_parameter<bool>(node, p + "compensate_state_latency", true);
        max_state_extrapolation_sec_ = declare_or_get_parameter<double>(node, p + "max_state_extrapolation_sec", 0.10);

        footprint_raw_ = declare_or_get_parameter<std::vector<double>>(
            node,
            p + "footprint_raw",
            {
                0.15,  0.15,
                0.15, -0.15,
                -0.15, -0.15,
                -0.15,  0.15
            });

        footprint_sample_step_ = declare_or_get_parameter<double>(node, p + "footprint_sample_step", 0.05);

        // optimizer settings
        // model_dt * time_steps（例如 50 × 0.05 s = 2.5 s）为当前控制器预测时域
        /*
        *  在 time_steps=0.05s、 az_max=1.5 时，每个周期最多变化 1.5✖0.05=0.075rad/s 的角速度（也就是从 0 加速到 0.75rad/s 需要 0.5s）
        *  在 vx_max=0.25m/s、wz_max=0.8rad/s 时，最小转弯半径为：vx / |wz| = 0.25/0.8 = 0.3125m ，也就是最小可以转半径为 0.3125m 的圈
        */
        optimizer_settings_.model_dt = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.model_dt", control_period_sec_));      // 控制周期与时间步最好一致
        optimizer_settings_.time_steps = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "optimizer_settings.time_steps", 40));
        optimizer_settings_.batch_size = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "optimizer_settings.batch_size", 384));
        optimizer_settings_.iteration_count = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "optimizer_settings.iteration_count", 2));
        optimizer_settings_.temperature = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.temperature", 0.7));
        optimizer_settings_.adaptive_temperature = declare_or_get_parameter<bool>(node, p + "optimizer_settings.adaptive_temperature", true);
        optimizer_settings_.target_ess_ratio = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.target_ess_ratio", 0.25));
        optimizer_settings_.min_temperature = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.min_temperature", 0.20));
        optimizer_settings_.max_temperature = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.max_temperature", 2.00));
        optimizer_settings_.gamma = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.gamma", 0.01));
        optimizer_settings_.vx_min = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.vx_min", 0.0));
        optimizer_settings_.vx_max = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.vx_max", 0.3));
        optimizer_settings_.wz_max = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.wz_max", 0.8));
        optimizer_settings_.vx_std = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.vx_std", 0.12));
        optimizer_settings_.wz_std = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.wz_std", 0.45));
        optimizer_settings_.ax_max = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.ax_max", 0.8));
        optimizer_settings_.ax_min = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.ax_min", -0.8));
        optimizer_settings_.az_max = static_cast<float>(declare_or_get_parameter<double>(node, p + "optimizer_settings.az_max", 2.0));     
        optimizer_settings_.collision_distance = declare_or_get_parameter<double>(node, p + "optimizer_settings.collision_distance", 0.03);
        optimizer_settings_.weight_logging_enabled = declare_or_get_parameter<bool>(node, p + "optimizer_settings.weight_logging_enabled", false);
        optimizer_settings_.logging_file_path = declare_or_get_parameter<std::string>(node, p + "optimizer_settings.weight_log_file", "/tmp/mppi_weights_" + std::to_string(optimizer_settings_.temperature) + ".csv");

        // critic params
        constraint_settings_.enabled = declare_or_get_parameter<bool>(node, p + "critics.constraint.enabled", true);
        constraint_settings_.cost_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.constraint.cost_weight", 4.0));
        constraint_settings_.cost_power = static_cast<unsigned int>(declare_or_get_parameter<int>(node, p + "critics.constraint.cost_power", 1));
        constraint_settings_.vx_min = optimizer_settings_.vx_min;
        constraint_settings_.vx_max = optimizer_settings_.vx_max;
        constraint_settings_.wz_max = optimizer_settings_.wz_max;

        path_follow_settings_.enabled = declare_or_get_parameter<bool>(node, p + "critics.path_follow.enabled", true);
        path_follow_settings_.cost_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_follow.cost_weight", 3.0));
        path_follow_settings_.cost_power = static_cast<unsigned int>(declare_or_get_parameter<int>(node, p + "critics.path_follow.cost_power", 1));
        path_follow_settings_.max_endpoint_path_distance = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_follow.max_endpoint_path_distance", 0.30));
        path_follow_settings_.offset_from_furthest = static_cast<size_t>(declare_or_get_parameter<int>(node, p + "critics.path_follow.offset_from_furthest", 3));

        path_align_settings_.enabled = declare_or_get_parameter<bool>(node, p + "critics.path_align.enabled", false);
        path_align_settings_.cost_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_align.cost_weight", 5.0));
        path_align_settings_.cost_power = static_cast<unsigned int>(declare_or_get_parameter<int>(node, p + "critics.path_align.cost_power", 1));
        path_align_settings_.trajectory_step = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "critics.path_align.trajectory_step", 2));

        esdf_footprint_settings_.enabled = declare_or_get_parameter<bool>(node, p + "critics.esdf.enabled", true);
        esdf_footprint_settings_.safe_distance =static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.esdf.safe_distance", 0.50));
        esdf_footprint_settings_.collision_distance = static_cast<float>(optimizer_settings_.collision_distance);
        esdf_footprint_settings_.repulsion_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.esdf.repulsion_weight", 20.0));
        esdf_footprint_settings_.collision_cost = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.esdf.collision_cost", 1.0e6));
        esdf_footprint_settings_.trajectory_step = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "critics.esdf.trajectory_step", 2));

        path_angle_settings_.enabled = declare_or_get_parameter<bool>(node, p + "critics.path_angle.enabled", false);
        path_angle_settings_.cost_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_angle.cost_weight", 10.0));
        path_angle_settings_.cost_power = static_cast<unsigned int>(declare_or_get_parameter<int>(node, p + "critics.path_angle.cost_power", 1));
        path_angle_settings_.trajectory_step = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "critics.path_angle.trajectory_step", 2));

        path_tracking_settings_.enabled = declare_or_get_parameter<bool>(node, p + "critics.path_tracking.enabled", true);
        path_tracking_settings_.cost_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.cost_weight", 1.0));
        path_tracking_settings_.lateral_error_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.lateral_error_weight", 18.0));
        path_tracking_settings_.heading_error_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.heading_error_weight", 6.0));
        path_tracking_settings_.speed_error_weight = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.speed_error_weight", 16.0));
        path_tracking_settings_.trajectory_step = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "critics.path_tracking.trajectory_step", 1));
        path_tracking_settings_.time_discount = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.time_discount", 0.98));
        path_tracking_settings_.max_linear_speed = optimizer_settings_.vx_max;
        path_tracking_settings_.min_curve_speed = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.min_curve_speed", 0.06));
        path_tracking_settings_.curvature_gain = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.curvature_gain", 1.20));
        path_tracking_settings_.lateral_error_speed_gain = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.lateral_error_speed_gain", 2.50));
        path_tracking_settings_.heading_error_speed_gain = static_cast<float>(declare_or_get_parameter<double>(node, p + "critics.path_tracking.heading_error_speed_gain", 1.00));
        path_tracking_settings_.curvature_lookahead = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "critics.path_tracking.curvature_lookahead", 3));

        // path_manager settings
        path_manager_settings_.local_path_horizon = declare_or_get_parameter<double>(node, p + "path_manager.local_path_horizon", 5.0);
        path_manager_settings_.path_resolution = declare_or_get_parameter<double>(node, p + "path_manager.path_resolution", 0.10);
        path_manager_settings_.goal_tolerance = declare_or_get_parameter<double>(node, p + "path_manager.goal_tolerance", 0.15);
        path_manager_settings_.nearest_search_window = static_cast<std::size_t>(declare_or_get_parameter<int>(node, p + "path_manager.nearest_search_window", 100));


        if (std::abs(static_cast<double>(optimizer_settings_.model_dt) - control_period_sec_) > 1.0e-6)
        {
            throw std::invalid_argument(
                "optimizer_settings.model_dt must equal control_period_sec.");
        }

        RCLCPP_INFO(
            node->get_logger(),
            "Params-loading accomplished!!!"
        );
    }

    void MPPILocalPlanner::createPlannerCore()
    {
        path_manager_ = std::make_unique<mppi_core::PathManager>(path_manager_settings_);

        auto critic_manager = std::make_unique<mppi_core::CriticManager>();

        critic_manager->addCritic(std::make_unique<mppi_core::ConstraintCritic>(constraint_settings_));
        critic_manager->addCritic(std::make_unique<mppi_core::PathFollowCritic>(path_follow_settings_));
        critic_manager->addCritic(std::make_unique<mppi_core::PathTrackingCritic>(path_tracking_settings_));
        critic_manager->addCritic(std::make_unique<mppi_core::PathAlignCritic>(path_align_settings_));
        critic_manager->addCritic(std::make_unique<mppi_core::PathAngleCritic>(path_angle_settings_));

        std::vector<mppi_core::Point2D> footprint_vertices;                 // 机体轮廓顶点

        if (footprint_raw_.size() < 6 || footprint_raw_.size() % 2 != 0)              // 至少是个三角形
        {
            throw std::runtime_error("mppi.footprint_vertices must contain at least 3 x/y pairs.");
        }

        for (std::size_t i = 0; i < footprint_raw_.size(); i += 2)               // 计算机体 footprint 顶点
        {
            footprint_vertices.push_back(mppi_core::Point2D{static_cast<float>(footprint_raw_[i]), static_cast<float>(footprint_raw_[i + 1])});
        }

        optimizer_settings_.footprint_samples = sampleFootprintEdges(footprint_vertices, footprint_sample_step_);

        esdf_footprint_settings_.footprint_samples = optimizer_settings_.footprint_samples;

        critic_manager->addCritic(std::make_unique<mppi_core::EsdfFootprintCritic>(esdf_footprint_settings_));

        optimizer_ = std::make_unique<mppi_core::Optimizer>(optimizer_settings_, std::move(critic_manager));
    }

    void MPPILocalPlanner::create_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        cmd_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic_,
            10
        );

        predicted_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
            mppi_predicted_path_topic_,
            10
        );
    }

    void MPPILocalPlanner::reset_runtime_state()
    {
        lifecycle_active_.store(false);

        esdf_buffer_.clear();

        path_manager_.reset();
        
        {
            std::lock_guard<std::mutex> lock(optimizer_mutex_);
            optimizer_.reset();
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            has_path_ = false;
            has_new_goal_.store(true);
            has_esdf_ = false;
            has_odom_ = false;
            has_last_command_ = false;
            last_command_ = mppi_core::ControlCommand{};
            planning_duration_ema_sec_ = 0.0;
        }
    }

    void MPPILocalPlanner::activate()
    {
        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        lifecycle_active_.store(true);

        // activate lifecycle pubs
        cmd_pub_->on_activate();
        predicted_path_pub_->on_activate();

        create_subscriptions(node);
        create_timers(node);
        // create_services(node);

        optimizer_->enableWeightLogging();  

        RCLCPP_INFO(
            node->get_logger(),
            "MPPILocalPlanner plugin '%s' activated.",
            plugin_name_.c_str());
    }

    void MPPILocalPlanner::create_subscriptions(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const auto esdf_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        const auto optimized_path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        const auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

        esdf_map_sub_ = node->create_subscription<trailblazer_map_interfaces::msg::EsdfMap>(
            input_esdf_topic_,
            esdf_qos,
            std::bind(&MPPILocalPlanner::esdf_map_callback, this, std::placeholders::_1)
        );

        optimized_path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
            optimized_path_topic_,
            optimized_path_qos,
            std::bind(&MPPILocalPlanner::optimized_path_callback, this, std::placeholders::_1)
        );

        goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_,
            goal_qos,
            std::bind(&MPPILocalPlanner::goal_callback, this, std::placeholders::_1)
        );

        auto odom_qos = rclcpp::SensorDataQoS();
        odom_qos.keep_last(1);
        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            odom_qos,
             std::bind(&MPPILocalPlanner::odom_callback, this, std::placeholders::_1)
        );
    }

    void MPPILocalPlanner::create_timers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        planning_timer = node->create_wall_timer(
            std::chrono::duration<double>(control_period_sec_),
            std::bind(&MPPILocalPlanner::control_timer_callback, this)
        );
    }

    void MPPILocalPlanner::esdf_map_callback(const trailblazer_map_interfaces::msg::EsdfMap::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        esdf_buffer_.update(msg);

        std::lock_guard<std::mutex> lock(state_mutex_);

        has_esdf_ = true;
        last_esdf_time_ = std::chrono::steady_clock::now();
    }

    void MPPILocalPlanner::optimized_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!lifecycle_active_) 
            return;

        auto node = node_.lock();
        if (!node)
            throw std::runtime_error("Parent node expired.");

        if (!msg || msg->poses.size() < 2)
        {
            path_manager_->clear();
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                has_path_ = false;
            }

            {
                std::lock_guard<std::mutex> lock(optimizer_mutex_);
                optimizer_->reset();
            }
            publish_stop();
            return;
        }

        const bool accepted = path_manager_->setPath(*msg);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            has_path_ = accepted;
        }

        if(accepted)
        {
            const bool need_reset = has_new_goal_.exchange(false);      // 只在有新目标点时 reset optimizer
            if(need_reset)
            {
                std::lock_guard<std::mutex> lock(optimizer_mutex_);

                optimizer_->reset();

                RCLCPP_INFO(
                    node->get_logger(),
                    "[MPPI] New goal path received, optimizer reset.");
            }
        }
        else
        {
            publish_stop();
        }
    }

    void MPPILocalPlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(!msg || !lifecycle_active_)
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_odom_ = *msg;
        has_odom_ = true;
        last_odom_time_ = std::chrono::steady_clock::now();
    }

    void MPPILocalPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if(!msg || !lifecycle_active_)
            return;

        has_new_goal_.store(true);
    }

    void MPPILocalPlanner::control_timer_callback()
    {
        if(!lifecycle_active_)
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

        // --------------- 一轮 optimize 前保存 runtime state 变量快照 ---------------
        nav_msgs::msg::Odometry odom;

        std::shared_ptr<const field_map_builder::EsdfMapSnapshot> esdf;

        bool has_path = false;
        bool has_odom = false;
        bool has_esdf = false;
        bool has_last_command = false;

        mppi_core::ControlCommand last_command;
        double planning_duration_ema_sec = 0.0;

        std::chrono::steady_clock::time_point last_odom_time;
        std::chrono::steady_clock::time_point last_esdf_time;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            has_path = has_path_;
            has_odom = has_odom_;
            has_esdf = has_esdf_;

            odom = latest_odom_;
            esdf = esdf_buffer_.snapshot();

            last_odom_time = last_odom_time_;
            last_esdf_time = last_esdf_time_;
            last_command = last_command_;
            has_last_command = has_last_command_;
            planning_duration_ema_sec = planning_duration_ema_sec_;
        }

        if (!has_path || !has_odom)
        {
            publish_stop();
            return;
        }

        // --------------------- 检查 odom/esdf 是否超时 ---------------------
        const auto now = std::chrono::steady_clock::now();

        if (std::chrono::duration<double>(now - last_odom_time).count() > odom_timeout_sec_)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "Haven't received odom for %.3fs, publishing stop command",
                odom_timeout_sec_
            );
            publish_stop();
            return;
        }

        if (!has_esdf || std::chrono::duration<double>(now - last_esdf_time).count() > esdf_timeout_sec_)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "Haven't received esdf map for %.3fs, publishing stop command",
                esdf_timeout_sec_
            );
            publish_stop();
            return;
        }

        // optimizer、全局路径和预测轨迹必须使用同一个 frame。
        const std::string local_path_frame = path_manager_->frameId();
        mppi_core::Pose2D robot_pose;
        if (!odomPoseInFrame(odom, local_path_frame, robot_pose))
        {
            publish_stop();
            return;
        }

        mppi_core::Twist2D robot_speed = odomToTwist(odom);

        const double arrival_age_sec =
            std::chrono::duration<double>(now - last_odom_time).count();
        double message_age_sec = arrival_age_sec;               // 现在距离上个 odom 的时间差值

        const rclcpp::Time odom_stamp(odom.header.stamp, node->get_clock()->get_clock_type());
        if (odom_stamp.nanoseconds() > 0)
        {
            const double stamped_age_sec = (node->now() - odom_stamp).seconds();
            if (std::isfinite(stamped_age_sec) && stamped_age_sec >= 0.0)
            {
                message_age_sec = std::max(message_age_sec, stamped_age_sec);
            }
        }

        if (message_age_sec > odom_timeout_sec_)
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "[MPPI] Odom sample is %.1f ms old according to its stamp; stopping.",
                1000.0 * message_age_sec);
            publish_stop();
            return;
        }

        // 总延迟时间（里程计延迟➕预计规划用时）
        const double compensated_latency_sec = compensate_state_latency_ ?
            std::clamp(
                message_age_sec + planning_duration_ema_sec,
                0.0,
                max_state_extrapolation_sec_) : 0.0;

        compensateStateLatency(
            robot_pose,
            robot_speed,
            last_command,
            has_last_command,
            compensated_latency_sec);

        // ----------------------- construct optimization input，start optimizing -----------------------
        // 截取局部路径作为 ReferencePath
        auto local_path_result = path_manager_->updateAndBuildLocalPath(robot_pose);   
        mppi_core::ReferencePath local_path = local_path_result.path;         
        if (local_path.size() < 2 || local_path_result.goal_reached)           // 同时检查是否到达目标点
        {
            publish_stop();
            return;
        }

        mppi_core::OptimizerInput input;
        input.robot_pose = robot_pose;
        input.robot_speed = robot_speed;
        input.reference_path = std::move(local_path);
        input.esdf = esdf;

        mppi_core::OptimizerResult result;            

        // 规划开始的时间
        const auto planning_start = std::chrono::steady_clock::now();

        try
        {
            std::lock_guard<std::mutex> lock(optimizer_mutex_);

            result = optimizer_->evalControl(input);                // optimize
        }
        catch (const std::exception & error)
        {
            RCLCPP_ERROR_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "MPPI failed: %s",
                error.what());

            publish_stop();
            return;
        }

        // 规划用时
        const double planning_duration_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - planning_start).count();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            planning_duration_ema_sec_ = planning_duration_ema_sec_ <= 0.0 ?
                planning_duration_sec :
                0.8 * planning_duration_ema_sec_ + 0.2 * planning_duration_sec;     // 历史用时*0.8+本次用时*0.2
        }

        if (planning_duration_sec > 0.8 * control_period_sec_)
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "[MPPI] planning took %.1f ms (period %.1f ms). "
                "Prediction/execution timing will diverge if this persists.",
                1000.0 * planning_duration_sec,
                1000.0 * control_period_sec_);
        }

        if (!result.valid)
        {
            publish_stop();
            return;
        }

        // ------------------------------- publish cmd_vel command -------------------------------
        geometry_msgs::msg::Twist command;
        command.linear.x = result.command.vx;
        command.angular.z = result.command.wz;

        cmd_pub_->publish(command);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_command_ = result.command;
            has_last_command_ = true;
        }

        RCLCPP_INFO_THROTTLE(
            node->get_logger(),
            *node->get_clock(),
            1000,
            "[MPPI]cmd_vel published: linear: %.2f, angular: %.2f",
            command.linear.x, command.angular.z
        );


        publish_predicted_path(result.optimized_trajectory, local_path_frame);
    }

    void MPPILocalPlanner::deactivate()
    {
        auto node = node_.lock();

        // 先挡住所有 callback
        lifecycle_active_.store(false, std::memory_order_release);

        esdf_map_sub_.reset();
        optimized_path_sub_.reset();
        odom_sub_.reset();
        goal_sub_.reset();

        // 停止 timers
        planning_timer.reset();

        // 停止本轮 optimize

        // 等待正在进行的 originalPathCallback 退出
        {
            std::lock_guard<std::mutex> lock(planning_mutex_);
        }

        // deactivate publishers
        if (cmd_pub_)
            cmd_pub_->on_deactivate();

        if (predicted_path_pub_)
            predicted_path_pub_->on_deactivate();

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "MPPILocalPlanner plugin '%s' deactivated.",
                plugin_name_.c_str());
        }
    }

    void MPPILocalPlanner::cleanup()
    {
        auto node = node_.lock();

        lifecycle_active_.store(false);

        esdf_map_sub_.reset();
        optimized_path_sub_.reset();
        odom_sub_.reset();
        goal_sub_.reset();

        // configure 阶段创建的 publishers 全部 reset
        cmd_pub_.reset();
        predicted_path_pub_.reset();

        reset_runtime_state();

        tf_listener_.reset();
        tf_buffer_.reset();

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "MPPILocalPlanner plugin '%s' cleaned up.",
                plugin_name_.c_str());
        }

        plugin_name_.clear();

        // 最后一件事再丢掉 node weak reference
        node_.reset();
    }

    // =================================== helpers ===================================

    void MPPILocalPlanner::publish_stop()
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_pub_->publish(cmd_vel_msg);

        std::lock_guard<std::mutex> lock(state_mutex_);
        last_command_ = mppi_core::ControlCommand{};
        has_last_command_ = true;
    }

    bool MPPILocalPlanner::odomPoseInFrame(
        const nav_msgs::msg::Odometry & odom,
        const std::string & target_frame,
        mppi_core::Pose2D & pose) const
    {
        auto node = node_.lock();
        if (!node || !tf_buffer_ || target_frame.empty() || odom.header.frame_id.empty())
        {
            return false;
        }

        geometry_msgs::msg::PoseStamped source;
        source.header = odom.header;
        source.pose = odom.pose.pose;

        geometry_msgs::msg::PoseStamped transformed = source;

        if (odom.header.frame_id != target_frame)
        {
            try
            {
                const auto transform = tf_buffer_->lookupTransform(
                    target_frame,
                    odom.header.frame_id,
                    rclcpp::Time(
                        odom.header.stamp,
                        node->get_clock()->get_clock_type()),
                    rclcpp::Duration::from_seconds(transform_timeout_sec_));
                tf2::doTransform(source, transformed, transform);
            }
            catch (const tf2::TransformException & error)
            {
                RCLCPP_ERROR_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    1000,
                    "[MPPI] Cannot transform odom pose from '%s' to path frame '%s': %s",
                    odom.header.frame_id.c_str(),
                    target_frame.c_str(),
                    error.what());
                return false;
            }
        }

        pose.x = static_cast<float>(transformed.pose.position.x);
        pose.y = static_cast<float>(transformed.pose.position.y);
        pose.yaw = static_cast<float>(tf2::getYaw(transformed.pose.orientation));

        return std::isfinite(pose.x) &&
               std::isfinite(pose.y) &&
               std::isfinite(pose.yaw);
    }

    mppi_core::Twist2D MPPILocalPlanner::odomToTwist(const nav_msgs::msg::Odometry & odom) const
    {
        mppi_core::Twist2D twist2d;
        twist2d.vx = odom.twist.twist.linear.x;
        twist2d.vy = odom.twist.twist.linear.y;
        twist2d.wz = odom.twist.twist.angular.z;

        return twist2d;
    }

    void MPPILocalPlanner::compensateStateLatency(
        mppi_core::Pose2D & pose,
        mppi_core::Twist2D & speed,
        const mppi_core::ControlCommand & last_command,
        bool has_last_command,
        double latency_sec) const
    {
        if (latency_sec <= 0.0)
        {
            return;
        }

        const float dt = static_cast<float>(latency_sec);
        const float initial_vx = speed.vx;
        const float initial_wz = speed.wz;
        const float target_vx = has_last_command ? last_command.vx : initial_vx;
        const float target_wz = has_last_command ? last_command.wz : initial_wz;

        // last_command 是现在正在执行的命令
        // 正常来说 speed 应该和 last_command 很接近
        // 但如果因为延迟导致二者并不一致，下边代码会预估直到本次规划前机器人能达到的速度
        const float predicted_vx = initial_vx + std::clamp(
            target_vx - initial_vx,
            optimizer_settings_.ax_min * dt,
            optimizer_settings_.ax_max * dt);
        const float predicted_wz = initial_wz + std::clamp(
            target_wz - initial_wz,
            -optimizer_settings_.az_max * dt,
            optimizer_settings_.az_max * dt);

        const float average_vx = 0.5f * (initial_vx + predicted_vx);
        const float average_wz = 0.5f * (initial_wz + predicted_wz);
        const float yaw_delta = average_wz * dt;
        const float midpoint_yaw = pose.yaw + 0.5f * yaw_delta;

        // 中点法积分位姿
        pose.x += average_vx * std::cos(midpoint_yaw) * dt;
        pose.y += average_vx * std::sin(midpoint_yaw) * dt;
        pose.yaw = std::atan2(
            std::sin(pose.yaw + yaw_delta),
            std::cos(pose.yaw + yaw_delta));

        speed.vx = predicted_vx;
        speed.vy = 0.0f;
        speed.wz = predicted_wz;
    }

    void MPPILocalPlanner::publish_predicted_path(const std::vector<mppi_core::Pose2D> & trajectory, const std::string & frame_id)
    {
        auto node = node_.lock();
        if (!node)
            throw std::runtime_error("Parent node expired.");

        nav_msgs::msg::Path predictrd_path_msg;
        predictrd_path_msg.header.frame_id = frame_id;
        predictrd_path_msg.header.stamp = node->now();
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header = predictrd_path_msg.header;
        for(const auto &pose2d : trajectory)
        {
            pose.pose.position.x = pose2d.x;
            pose.pose.position.y = pose2d.y;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, pose2d.yaw);
            pose.pose.orientation = tf2::toMsg(q);
            predictrd_path_msg.poses.push_back(pose);
        }

        predicted_path_pub_->publish(predictrd_path_msg);
    }

}       // namespace local_planner

PLUGINLIB_EXPORT_CLASS(local_planner::MPPILocalPlanner, local_planner::LocalPlannerBase)
