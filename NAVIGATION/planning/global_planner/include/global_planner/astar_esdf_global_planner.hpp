#ifndef GLOBAL_PLANNER__ASTAR_ESDF_GLOBAL_PLANNER_HPP_
#define GLOBAL_PLANNER__ASTAR_ESDF_GLOBAL_PLANNER_HPP_

#include "global_planner/global_planner_base.hpp"
#include "field_map_builder/utility/esdf_query.hpp"
#include "global_planner/utility/path_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "trailblazer_map_interfaces/msg/esdf_map.hpp"

namespace global_planner
{
    struct Node
    {
        int x{0};
        int y{0};
        double f{0.0};
        double h{0.0}; // 用于 tie-break，h 更小者优先

        bool operator>(const Node & other) const
        {
            if (f != other.f)
                return f > other.f;

            return h > other.h;
        }
    };

    class AstarEsdfGlobalPlanning : public GlobalPlannerBase
    {
        public:
            AstarEsdfGlobalPlanning() = default;
            ~AstarEsdfGlobalPlanning() override = default;

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
            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void occupancy_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
            void esdf_map_callback(const trailblazer_map_interfaces::msg::EsdfMap::SharedPtr msg);
            void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

            void planningTimerCallback();

            // ----------------------------- other functions in pipeline -----------------------------
            std::vector<std::pair<int, int>> astar_esdf(
                const std::pair<int, int>& start,
                const std::pair<int, int>& goal,
                const std::vector<std::vector<int>>& grid,
                const field_map_builder::EsdfMapSnapshot& esdf,
                int obstacle_threshold,
                bool treat_unknown_as_obstacle,
                double unknown_cost_penalty,
                double cost_scale);
            double compute_esdf_clearance_penalty(double esdf_dist) const;              // 计算 ESDF 代价（距离小于 safe_clearance_ 会产生额外代价）

            // ----------------------------- helpers -----------------------------
            geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

        private:
            LifecycleNodeWeakPtr node_;
            std::string plugin_name_;

            std::atomic_bool lifecycle_active_{false};            // 存储插件状态(active/inactive)

            // ----------------------------- Parameters -----------------------------
            bool if_silence_{false};
            bool if_debug_{true};

            std::string global_path_topic_;
            std::string input_costmap_topic_;
            std::string input_esdf_topic_;
            std::string input_odom_topic_;
            std::string input_goal_topic_;
            
            bool if_keep_planning_;                     // 是否开启重复规划

            double min_dist_to_plan_;                   // 开启规划的最小距离
            bool treat_unknown_as_obstacle_;            // 是否将未知视作障碍物
            int obstacle_threshold_;                    // 大于等于这个数的代价会被当作障碍物
            double min_esdf_value_for_goal_;             // 终点能够容忍的最小 ESDF 距离

            int unknown_cost_penalty_;                  // 如果不将未知格子视作障碍物，那么代价是多少
            double cost_scale_;
            // double heuristic_tie_breaker_;              // 让多个节点的 f 都差不多时，稍微增大 h 使得搜索朝着目标方向进行
            
            bool if_path_downsampling_;                 // 是否开启路径下采样
            int corner_deg_;                            // 路径下采样角度阈值
            int corner_dilate_;                         // 路径拐点上采样数目 

            bool use_esdf_soft_cost_;                   // 是否启用 ESDF 代价
            double safe_clearance_;                     // 产生 ESDF 代价的距离
            double esdf_cost_scale_;                    // ESDF 代价权重

            // ----------------------------- Subscriptions -----------------------------
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_sub_;
            rclcpp::Subscription<trailblazer_map_interfaces::msg::EsdfMap>::SharedPtr esdf_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
            
            // ----------------------------- Lifecycle publishers -----------------------------
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

            // ----------------------------- Timer -----------------------------
            rclcpp::TimerBase::SharedPtr planning_timer;

            // ----------------------------- Servers -----------------------------

            // ----------------------------- Runtime state -----------------------------
            field_map_builder::EsdfMapBuffer esdf_buffer_;

            bool grid_initialized_{false};             // 私有地图变量是否有初始化
            int last_grid_height_ = 0;                
            int last_grid_width_ = 0;

            std::vector<std::vector<int>> grid_cost_;       // 私有的地图变量，存储代价地图的值

            struct GoalState
            {
                double x{0.0};
                double y{0.0};
                uint64_t generation{0};
                bool planned{false};                    // 标记这个 goal 是否已经规划过
                std::string frame_id;
            };

            std::mutex state_mutex_;

            nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_msg_;

            double robot_x_{0.0};
            double robot_y_{0.0};
            bool has_odom_{false};

            std::optional<GoalState> goal_;
            uint64_t next_goal_generation_{0};

            std::mutex planning_mutex_;                     
    };

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__ASTAR_ESDF_GLOBAL_PLANNER_HPP_