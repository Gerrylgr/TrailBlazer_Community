#ifndef FIELD_MAP_BUILDER__ESDF_MAP_BUILDER_HPP_
#define FIELD_MAP_BUILDER__ESDF_MAP_BUILDER_HPP_

#include "field_map_builder/field_map_builder_base.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "trailblazer_map_interfaces/msg/esdf_map.hpp"

namespace field_map_builder
{

    class EsdfMapBuilding : public FieldMapBuilderBase
    {
        public:
            EsdfMapBuilding() = default;
            ~EsdfMapBuilding() override = default;

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
            void final_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

            // ----------------------------- other functions in pipeline -----------------------------
            // map meta / buffer management
            bool map_meta_changed(const nav_msgs::msg::OccupancyGrid & msg) const;          // 检查地图元数据是否改变
            void update_map_meta_from_msg(const nav_msgs::msg::OccupancyGrid & msg);        // 更新地图元数据
            void resize_buffers_if_needed();                    // 分配或重置各种缓存
            void reset_esdf_debug_map_msg();                    // reset esdf_debug_map_msg_
            void reset_esdf_map_msg();                          // reset esdf_map_msg_

            // build occupancy mask
            void build_obstacle_mask_from_occupancy(const nav_msgs::msg::OccupancyGrid & msg);

            // Signed ESDF: free > 0, occupied < 0.
            void build_signed_esdf();
            void initialize_sq_distance_grid(uint8_t seed_value);           // 计算平方距离种子图
            void compute_edt_2d();                  // 二维 EDT 入口
            void compute_row_edt();
            void compute_column_edt();
            void distance_transform_1d(int n);              // EDT 实现
            float squared_cells_to_metric_distance(float squared_cells) const;          // 平方格转米制距离

            void publish_esdf_debug_map(const builtin_interfaces::msg::Time & stamp);       // debug publish
            void publish_esdf_map(const builtin_interfaces::msg::Time & stamp);         // ESDF publish

            // ----------------------------- helpers -----------------------------
            
        private:
            LifecycleNodeWeakPtr node_;
            std::string plugin_name_;

            std::atomic_bool lifecycle_active_{false};            // 存储插件状态(active/inactive)

            // ----------------------------- Parameters -----------------------------
            bool if_silence_{false};
            bool if_debug_{true};

            std::string input_costmap_topic_;
            std::string esdf_debug_topic_;
            std::string esdf_topic_;
            bool unknown_as_obstacle_;              // 是否将未知当作障碍物
            bool publish_debug_esdf_;
            float max_esdf_distance_;

            // Map Meta
            int grid_cols_;
            int grid_rows_;
            double resolution_;
            double origin_x_;
            double origin_y_;
            std::string frame_id_;
            bool map_initialized_{false};
            
            // ----------------------------- Subscriptions -----------------------------
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
            
            // ----------------------------- Lifecycle publishers -----------------------------
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_debug_pub_;
            rclcpp_lifecycle::LifecyclePublisher<trailblazer_map_interfaces::msg::EsdfMap>::SharedPtr esdf_pub_;

            // ----------------------------- Timer -----------------------------

            // ----------------------------- Servers -----------------------------

            // ----------------------------- Runtime state -----------------------------
            std::vector<uint8_t> obstacle_mask_;            // costmap 的二值 mask
            std::vector<float> edt_tmp_;                    // ESDF 主缓存
            std::vector<float> edt_sq_dist_;                // 平方距离种子图（障碍物为0，空地为无穷大）

            // Reused 1-D EDT workspace.
            std::vector<float> edt_line_input_;                 // EDT-line-input（0 或者无穷大）
            std::vector<float> edt_line_output_;
            std::vector<int> edt_parabola_indices_;             // 最终“有效”的抛物线索引（下包络线）
            std::vector<float> edt_parabola_boundaries_;        

            nav_msgs::msg::OccupancyGrid esdf_debug_map_msg_;
            trailblazer_map_interfaces::msg::EsdfMap esdf_map_msg_;
    };

}  // namespace field_map_builder

#endif  // FIELD_MAP_BUILDER__ESDF_MAP_BUILDER_HPP_