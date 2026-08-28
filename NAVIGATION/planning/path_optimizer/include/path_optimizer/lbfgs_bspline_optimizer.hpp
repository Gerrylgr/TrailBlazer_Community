#ifndef PATH_OPTIMIZER__LBFGS_BSPLINE_OPTIMIZER_HPP_
#define PATH_OPTIMIZER__LBFGS_BSPLINE_OPTIMIZER_HPP_

#include "path_optimizer/path_optimizer_base.hpp"
#include "field_map_builder/utility/esdf_query.hpp"
#include "path_optimizer/utility/uniform_bspline.hpp"
#include "path_optimizer/lbfgs_backend.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav_msgs/msg/path.hpp"

#include "trailblazer_map_interfaces/msg/esdf_map.hpp"

#include <Eigen/Core>

namespace path_optimizer
{
    class LbfgsBsplineOptimizing : public PathOptimizerBase
    {
        public:
            LbfgsBsplineOptimizing() = default;
            ~LbfgsBsplineOptimizing() override = default;

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

            // void create_timers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            // void create_services(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            void reset_runtime_state();

            // ----------------------------- callbacks -----------------------------
            void esdf_map_callback(const trailblazer_map_interfaces::msg::EsdfMap::SharedPtr msg);
            void originalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

            // ----------------------------- other functions in pipeline -----------------------------
            bool rosPathToEigenPoints(
                const nav_msgs::msg::Path & path_msg, 
                std::vector<Eigen::Vector2d> & points) const;       // 将 nav_msgs/msg/Path 变为 std::vector<Eigen::Vector2d> 类型
            nav_msgs::msg::Path sampleToRosPath(
                const std::vector<Eigen::Vector2d> & sampled_pts, 
                const std_msgs::msg::Header & header) const;        // 将 std::vector<Eigen::Vector2d> 变为 nav_msgs/msg/Path

            // ----------------------------- helpers -----------------------------

        private:
            LifecycleNodeWeakPtr node_;
            std::string plugin_name_;

            std::atomic_bool lifecycle_active_{false};            // 存储插件状态(active/inactive)

            // ----------------------------- Parameters -----------------------------
            bool if_silence_{false};
            bool if_debug_{true};

            std::string init_bspline_topic_;
            std::string optimized_path_topic_;
            std::string input_esdf_topic_;
            std::string original_path_topic_;

            // 输入、输出路径的空间分辨率
            double input_path_resolution_{0.10};
            double output_path_resolution_{0.05};
            double output_parameter_step_{0.1};

            // 几何B样条使用无量纲、固定的knot间隔
            static constexpr double kKnotInterval{1.0};

            bool if_reference_cost_{true};              // 是否开启参考路径代价
            bool if_curvature_cost_{true};              // 是否开启曲率代价

            double lambda_smooth_;
            double lambda_distance_;
            double obstacle_dist_;              // 对圆形机器人，obstacle_dist_ = robot_radius + safety_margin;
            double lambda_reference_{1.0};          // 参考路径代价权重
            double reference_tolerance_{0.2};        // 距离原始路径这么远才会开始产生代价
            double lambda_curvature_{1.0};          // 曲率代价权重
            double max_curvature_{1.0};             // 1/m
            double tangent_epsilon_{1e-4};
            
            int max_iterations_;
            bool if_lbfgs_;
            double g_epsilon_;
            int fixed_boundary_control_points_{3};
            int optimization_samples_per_span_{5};

            int memory_size_{8};
            int lbfgs_past_{5};
            double lbfgs_delta_{1.0e-4};
            int max_linesearch_{20};

            // ----------------------------- Subscriptions -----------------------------
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
            rclcpp::Subscription<trailblazer_map_interfaces::msg::EsdfMap>::SharedPtr esdf_map_sub_;
            
            // ----------------------------- Lifecycle publishers -----------------------------
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr bspline_init_path_pub_;
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;

            // ----------------------------- Timer -----------------------------

            // ----------------------------- Servers -----------------------------

            // ----------------------------- Runtime state -----------------------------
            LbfgsBackend lbfgs_backend_;               
        
            field_map_builder::EsdfMapBuffer esdf_buffer_;

            std::mutex planning_mutex_; 
            
            std::atomic_bool cancel_flag_{false};               // 传给 lbfgs 后端的标志位，用于停止本轮优化
    };

}  // namespace path_optimizer

#endif  // PATH_OPTIMIZER__LBFGS_BSPLINE_OPTIMIZER_HPP_