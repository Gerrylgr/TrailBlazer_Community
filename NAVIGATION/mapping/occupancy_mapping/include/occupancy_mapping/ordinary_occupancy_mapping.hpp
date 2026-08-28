#ifndef OCCUPANCY_MAPPING__ORDINARY_OCCUPANCY_MAPPING_HPP_
#define OCCUPANCY_MAPPING__ORDINARY_OCCUPANCY_MAPPING_HPP_

#include "occupancy_mapping/occupancy_mapping_base.hpp"
#include "occupancy_mapping/mapping_types.hpp"
#include "occupancy_mapping/grid_geometry.hpp"
#include "occupancy_mapping/static_map_io.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "trailblazer_map_interfaces/msg/map_status.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

namespace occupancy_mapping
{

    class OrdinaryOccupancyMapping : public OccupancyMappingBase
    {
        public:
            OrdinaryOccupancyMapping() = default;
            ~OrdinaryOccupancyMapping() override = default;

            /*
            * configure 阶段创建“静态资源/结构资源”
            * activate 阶段启动“会让节点真正开始工作的运行时资源”
            */
            void configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name) override;
            void activate() override;
            void deactivate() override;
            void cleanup() override;

        private:
            // 地图加载结果
            enum class DefaultMapLoadResult
            {
                NOT_FOUND,
                LOADED,
                INVALID
            };

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
            void startMappingSubscriptions();
            void stopMappingSubscriptions();

            void create_timers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            void create_services(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

            void reset_runtime_state();
            void resetMapDataLocked();                      // 将 reset_runtime_state 中 map data 部分摘出来，要求调用者自己持有锁

            // callbacks
            void groundCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
            void nonGroundCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void submap_cloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

            void stateTimerCallback();
            void mapPublishTimerCallback();

            void startMappingCallback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response
            );
            void stopMappingCallback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response
            );
            void errorClearingCallback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response
            );
            void startIncrementalCallback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response
            );

            // other functions in pipeline
            DefaultMapLoadResult tryLoadDefaultMap(std::string & message);          // try loading map
            bool staticMapDataRestore(StaticMapData & loaded_data);         // 将 StaticMapData 转换为 geometry_、final_cells_

            bool takeSynchronizedFrameLocked(                       // 尝试对一帧 ground-points/nonground-points 做同步
                sensor_msgs::msg::PointCloud2::SharedPtr & ground, 
                sensor_msgs::msg::PointCloud2::SharedPtr & non_ground
            );
            void processFrame(                                      //处理一帧点云的总入口，得到一帧的混合地图
                const sensor_msgs::msg::PointCloud2::SharedPtr & ground, 
                const sensor_msgs::msg::PointCloud2::SharedPtr & non_ground
            );

            bool ensureFrameBounds(                                 // 确保当前变量能容纳点云
                const sensor_msgs::msg::PointCloud2 & ground, 
                const sensor_msgs::msg::PointCloud2 & non_ground
            );
            bool accumulateCloudBounds(                           // 输入一帧点云，计算边界
                const sensor_msgs::msg::PointCloud2 &msg, 
                double &min_x, double &max_x, double &min_y, double &max_y
            );
            bool ensureBounds(                                    // 输入点云边界，如果不够则扩充
                double min_x, double max_x, double min_y, double max_y, double expand_margin
            );

            void resetFrameBuffers();                               // 重置所有单帧的临时变量

            void processGroundCloud(const sensor_msgs::msg::PointCloud2 & msg);         // 处理一帧 ground points
            void finalizeGroundCells(const std::vector<std::size_t> & updated_indices);

            void processNonGroundCloud(const sensor_msgs::msg::PointCloud2 & msg);              // 处理一帧 non-ground points
            bool queryGroundHeight(int mx, int my, float & ground_z) const;
            void finalizeObstacleCells(const std::vector<std::size_t> & updated_indices);

            int raycastCloudFreeSpace(                              // 对输入点云做 free-space raycast
                const sensor_msgs::msg::PointCloud2 & msg, 
                double sensor_x, double sensor_y, bool include_endpoint
            );
            int raycastFreeCells(                                   // 对起点到终点路径上的点云做 free-space raycast(Bresenham)
                int start_mx, int start_my, 
                int end_mx, int end_my, bool include_endpoint
            );
            bool markRaycastFreeCell(int mx, int my);                   // 将一个格子覆盖为 free-space(覆盖 unknown，不覆盖 obstacle)

            void fuseFrameToFinal(const rclcpp::Time & stamp);          // 根据单帧数据，将置信度填入 final_cells 中

            nav_msgs::msg::OccupancyGrid toOccupancyGrid(const rclcpp::Time & stamp);           // 将现有的 final_cells 转为 occupancy 消息
            int8_t updateAndGetOccupancy(FinalCell & cell);                  // 根据 final cell 的置信度判断格子是 occupied/free/unknown

            StaticMapData snapShotGetter();                 // 获取一帧的 StaticMapData 地图数据，用于保存地图

            void inflateSingleCell(int occ_mx, int occ_my, nav_msgs::msg::OccupancyGrid &map_msg);          // 对一个障碍物格子做两圈膨胀
            nav_msgs::msg::OccupancyGrid inflateObstacleLayer(nav_msgs::msg::OccupancyGrid &map_msg);            // 对整个 occupancy map 做膨胀 

            nav_msgs::msg::OccupancyGrid buildEmptySubmapLocked(const builtin_interfaces::msg::Time & stamp);       // 初始化空的 submap msg

            // helpers
            bool whetherCurrentValid();

            float compute_percentile(std::vector<float> values, float q) const;       // 计算一串数的指定分位数

            std::string mapStateToString(MapState state);

        private:
            LifecycleNodeWeakPtr node_;
            std::string plugin_name_;

            std::atomic_bool lifecycle_active_{false};            // 存储插件状态(active/inactive)

            // Parameters
            bool if_silence_{false};
            bool if_debug_{true};

            std::string input_odom_topic_;
            std::string ground_topic_;
            std::string non_ground_topic_;
            std::string submap_cloud_topic_;
            std::string submap_output_topic_;
            std::string map_status_topic_;
            std::string static_map_topic_;
            std::string inflation_map_topic_;

            std::string start_mapping_service_;
            std::string stop_mapping_service_;
            std::string error_clearing_service_;
            std::string start_incremental_service_;

            std::string frame_id_;

            int map_publish_frequency_{1};              // （构建地图时）发布地图的频率

            double initial_width_{10.0};                // 地图初始数据
            double initial_height_{10.0};
            double resolution_{0.05};

            double cloud_sync_tolerance_;                       // 点云同步容忍时间差

            double expand_margin_;                  // 点云边界扩充冗余量

            bool if_strict_;                        // 开启会要求障碍物点附近格子有地面格子，来查询参考高度；关闭则使用障碍物点的绝对高度判断

            int min_ground_points_per_cell_{4};             // 地面格子中地面点最少数目
            double ground_percentile_{0.4};                 // 估计地面高度的分位数

            double min_obstacle_height_{0.02};               // 算做障碍物点的最低高度
            double strong_obstacle_height_{0.12};            // 算做强证据点的障碍物高度

            int min_non_ground_count_{4};                // 算做障碍物格子的最少障碍物点数
            double min_mean_height_{0.04};                  // 算做障碍物格子的最低平均高度
            int min_strong_count_{2};                    // 算做障碍物格子的强证据点数

            bool enable_ground_raycast_{false};             // 是否开启 ground-points 的 free-space raycast
            bool enable_non_ground_raycast_{true};          // 是否开启 non-ground-points 的 free-space raycast

            int raycast_point_step_{1};                     // raycast step
            double raycast_min_range_{0.15};                // raycast min distance
            double raycast_max_range_{20.0};                // raycast max distance

            int raycast_endpoint_margin_cells_{0};          // raycast 边界距离终点格子数

            double hit_logodds_{1.0};
            double miss_logodds_{0.5};
            double raycast_miss_logodds_{0.3};
            double logodds_min_{-2.5};
            double logodds_max_{3.5};
            double occ_enter_thresh_{0.1};                  // 高于这个值就算做障碍物
            double occ_exit_thresh_{-0.5};                  // occupied 退出阈值
            double free_thresh_{-1.0};                      // free 阈值
            
            std::string map_directory_;                     // 地图存储路径
            std::string map_name_;                  // map name

            bool inflate_unknown_;                  // 是否在未知区域膨胀
            int inflation_ring1_cost_;              // 第一圈膨胀代价
            int inflation_ring2_cost_;              // 第二圈膨胀代价

            int submap_min_points_per_cell_;        // 子地图一个格子最少的点数
            bool submap_inflate_;                   // 是否膨胀子地图
            
            // Subscriptions
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_sub_;

            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr submap_cloud_sub_;

            // Lifecycle publishers
            rclcpp_lifecycle::LifecyclePublisher<trailblazer_map_interfaces::msg::MapStatus>::SharedPtr state_pub_;
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_pub_;
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_inflation_pub_;
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr submap_pub_;

            // Timer
            rclcpp::TimerBase::SharedPtr state_timer_;
            rclcpp::TimerBase::SharedPtr map_publish_timer_;

            // Servers
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mapping_server_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_mapping_server_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr error_clearing_server_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_incremental_server_;

            // Runtime state
            std::mutex map_mutex_;                      // map status mutex
            MapState state_{MapState::NO_MAP};                      // MAP STATUS

            std::unique_ptr<StaticMapIO> map_io_;  

            GridGeometry geometry_;

            std::vector<GroundCell> ground_cells_;
            std::vector<ObstacleCell> obstacle_cells_;

            std::vector<std::uint8_t> raycast_free_mask_;

            std::vector<int8_t> frame_grid_;                    // 单帧融合地图
            std::vector<FinalCell> final_cells_;                // final cells

            std::mutex frame_mutex_;                                // for pending_ground_ && pending_non_ground_
            sensor_msgs::msg::PointCloud2::SharedPtr pending_ground_;               // 用于单帧同步的 ground msg/non-ground msg
            sensor_msgs::msg::PointCloud2::SharedPtr pending_non_ground_;

            std::mutex odom_mutex_;
            double robot_x_{0.0};
            double robot_y_{0.0};
            bool has_odom_{false};

            bool frame_has_observation_{false};                 // frame_has_observation_ 为 true 才允许 fuseFinalCells
    };

}  // namespace occupancy_mapping

#endif  // OCCUPANCY_MAPPING__ORDINARY_OCCUPANCY_MAPPING_HPP_