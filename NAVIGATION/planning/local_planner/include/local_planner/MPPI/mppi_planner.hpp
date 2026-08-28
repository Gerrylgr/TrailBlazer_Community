#ifndef LOCAL_PLANNER__MPPI_PLANNER_HPP_
#define LOCAL_PLANNER__MPPI_PLANNER_HPP_

#include "local_planner/local_planner_base.hpp"
#include "field_map_builder/utility/esdf_query.hpp"
#include "local_planner/MPPI/tools/path_manager.hpp"
#include "local_planner/MPPI/core/critic_manager.hpp"
#include "local_planner/MPPI/critics/constraint_critic.hpp"
#include "local_planner/MPPI/critics/path_follow_critic.hpp"
#include "local_planner/MPPI/critics/path_align_critic.hpp"
#include "local_planner/MPPI/critics/esdf_footprint_critic.hpp"
#include "local_planner/MPPI/critics/path_angle_critic.hpp"
#include "local_planner/MPPI/critics/path_tracking_critic.hpp"
#include "local_planner/MPPI/core/optimizer_settings.hpp"
#include "local_planner/MPPI/core/optimizer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "trailblazer_map_interfaces/msg/esdf_map.hpp"

namespace local_planner
{
    class MPPILocalPlanner : public LocalPlannerBase
    {
        public:
            MPPILocalPlanner() = default;
            ~MPPILocalPlanner() override = default;

            /*
            * configure 阶段创建“静态资源/结构资源”
            * activate 阶段启动“会让节点真正开始工作的运行时资源”
            */
            void configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name) override;
            void activate() override;
            void deactivate() override;
            void cleanup() override;

        private:
            template<typename T>
            T declare_or_get_parameter(
                const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
                const std::string & name,
                const T & default_value)
            {
                if (!node->has_parameter(name)) 
                {
                    return node->declare_parameter<T>(name, default_value);
                }

                return node->get_parameter(name).get_value<T>();
            }

            void load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            void create_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            void create_subscriptions(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
            // void startMappingSubscriptions();
            // void stopMappingSubscriptions();

            void create_timers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            // void create_services(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            void reset_runtime_state();

            // ----------------------------- callbacks -----------------------------
            void esdf_map_callback(const trailblazer_map_interfaces::msg::EsdfMap::SharedPtr msg);
            void optimized_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

            void control_timer_callback();

            // ----------------------------- other functions in pipeline -----------------------------
            void createPlannerCore();
         
            // ----------------------------- helpers -----------------------------
            void publish_stop();
            bool odomPoseInFrame(
                const nav_msgs::msg::Odometry & odom,
                const std::string & target_frame,
                mppi_core::Pose2D & pose) const;
            mppi_core::Twist2D odomToTwist(const nav_msgs::msg::Odometry & odom) const;
            void compensateStateLatency(
                mppi_core::Pose2D & pose,
                mppi_core::Twist2D & speed,
                const mppi_core::ControlCommand & last_command,
                bool has_last_command,
                double latency_sec) const;
            void publish_predicted_path(const std::vector<mppi_core::Pose2D> & trajectory, const std::string & frame_id);

        private:
            LifecycleNodeWeakPtr node_;
            std::string plugin_name_;

            std::atomic_bool lifecycle_active_{false};            // 存储插件状态(active/inactive)

            // ----------------------------- Parameters -----------------------------
            bool if_silence_{false};
            bool if_debug_{true};

            // 机器人 footprint 外参(定义顶点和采样距离，后续解析成顶点坐标)
            std::vector<double> footprint_raw_;
            double footprint_sample_step_;

            std::string cmd_vel_topic_;
            std::string input_esdf_topic_;
            std::string optimized_path_topic_;
            std::string goal_topic_;
            std::string odom_topic_;
            std::string mppi_predicted_path_topic_;

            double control_period_sec_;
            
            double odom_timeout_sec_;
            double esdf_timeout_sec_;
            double transform_timeout_sec_;
            bool compensate_state_latency_;                 // 是否开启延迟补偿
            double max_state_extrapolation_sec_;            // 最大的延迟时间补偿

            mppi_core::PathManagerSettings path_manager_settings_;

            // critic settings
            mppi_core::ConstraintCriticSettings constraint_settings_;
            mppi_core::PathFollowCriticSettings path_follow_settings_;
            mppi_core::PathAlignCriticSettings path_align_settings_;
            mppi_core::EsdfFootprintCriticSettings esdf_footprint_settings_;
            mppi_core::PathAngleCriticSettings path_angle_settings_;
            mppi_core::PathTrackingCriticSettings path_tracking_settings_;

            mppi_core::OptimizerSettings optimizer_settings_;

            // ----------------------------- Subscriptions -----------------------------
            rclcpp::Subscription<trailblazer_map_interfaces::msg::EsdfMap>::SharedPtr esdf_map_sub_;
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr optimized_path_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
            
            // ----------------------------- Lifecycle publishers -----------------------------
            rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            // ----------------------------- Timer -----------------------------
            rclcpp::TimerBase::SharedPtr planning_timer;

            // ----------------------------- Servers -----------------------------

            // ----------------------------- Runtime state -----------------------------
            field_map_builder::EsdfMapBuffer esdf_buffer_;

            std::unique_ptr<mppi_core::PathManager> path_manager_;

            std::unique_ptr<mppi_core::Optimizer> optimizer_;

            std::mutex state_mutex_;
            std::mutex optimizer_mutex_;

            bool has_path_{false};
            std::atomic_bool has_new_goal_{true};                   // 在收到新的 goal 时才会 reset optimizer
            bool has_esdf_{false};

            std::chrono::steady_clock::time_point last_esdf_time_;

            nav_msgs::msg::Odometry latest_odom_;
            bool has_odom_{false};
            std::chrono::steady_clock::time_point last_odom_time_;

            mppi_core::ControlCommand last_command_;
            bool has_last_command_{false};
            double planning_duration_ema_sec_{0.0};

            std::mutex planning_mutex_; 
    };

}  // namespace local_planner

#endif  // LOCAL_PLANNER__MPPI_PLANNER_HPP_
