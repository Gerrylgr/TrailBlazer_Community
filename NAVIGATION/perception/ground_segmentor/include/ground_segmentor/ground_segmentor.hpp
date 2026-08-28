#ifndef GROUND_SEGMENTOR__GROUND_SEGMENTOR_HPP_
#define GROUND_SEGMENTOR__GROUND_SEGMENTOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <atomic>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <chrono>

#include <Eigen/Dense>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "ground_segmentor/ground_segmentor_base.hpp"

namespace ground_segmentor
{

    // 表示一个点
    struct PointXYZIConf
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float intensity = 0.0f;
        float ground_confidence = 0.0f;
    };

    struct Plane
    {
        // 平面的中心坐标
        float cx = 0.0f;
        float cy = 0.0f;
        float cz = 0.0f;

        // 平面的法向量
        float nx = 0.0f;
        float ny = 0.0f;
        float nz = 1.0f;

        // 平面的特征值
        float ev0 = 0.0f;
        float ev1 = 0.0f;
        float ev2 = 0.0f;
    };

    // block 格子索引
    struct GridIndex
    {
        int band = 0;   // 表示格子所属范围：0: [0,3), 1: [3,10), 2: [10,+inf)
        int ix = 0;
        int iy = 0;

        bool operator==(const GridIndex & other) const
        {
            return band == other.band && ix == other.ix && iy == other.iy;
        }
    };

    struct GridIndexHash
    {
        std::size_t operator()(const GridIndex & idx) const
        {
            std::size_t h1 = std::hash<int>{}(idx.band);
            std::size_t h2 = std::hash<int>{}(idx.ix);
            std::size_t h3 = std::hash<int>{}(idx.iy);

            std::size_t seed = h1;
            seed ^= (h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2));
            seed ^= (h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2));
            return seed;
        }
    };

    // 一个 block 格子的具体数据
    struct CellData
    {
        std::vector<int> point_indices;     // 这个 block 中包含的点在 processed_points 中的索引

        // 这些点的中心
        float mean_x = 0.0f;
        float mean_y = 0.0f;
        float mean_z = 0.0f;

        // 这些点的最高、最低值
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        bool has_plane = false;
        bool is_seed = false;
        bool is_ground = false;
        int group_id = -1;

        int band = 0;
        float cell_resolution = 0.4f;     // 记录格子的分辨率大小

        Plane plane;

        // mixed cell recovery 
        bool has_recovered_ground_plane = false;
        Plane recovered_ground_plane;
        bool is_mixed_suspect = false;
    };

    struct GroundGroup
    {
        int id = -1;
        Plane ref_plane;
        std::vector<GridIndex> cells;
    };

    class GroundSegmentor : public GroundSegmentorBase
    {
        public:
            GroundSegmentor() = default;
            ~GroundSegmentor() override = default;

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

            void reset_runtime_state();

            void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

            bool update_frame_robot_pose_snapshot();

            // ===== basic =====
            void reset_grid_map();
            void reset_ground_groups();

            // ===== multi-band grid =====
            int get_range_band(const PointXYZIConf & pt) const;
            float get_grid_resolution_for_band(int band) const;
            float get_band_min_range(int band) const;
            float get_band_max_range(int band) const;
            GridIndex point_to_grid_index(const PointXYZIConf & pt) const;

            // ===== grid =====
            void build_grid_map();
            std::vector<GridIndex> get_8_neighbors(const GridIndex & idx) const;
            bool grid_index_exists(const GridIndex & idx) const;

            // ===== plane fit =====
            float compute_planarity(float ev0, float ev1, float ev2) const;
            float compute_flatness(float ev0, float ev2) const;
            bool fit_plane_for_cell(CellData & cell);
            void fit_planes_for_all_cells();

            // ===== seed =====
            bool estimate_ground_height_from_nearby_points();
            float compute_seed_angle_deg(const CellData & cell) const;
            bool is_seed_cell(CellData & cell) const;
            void generate_ground_seeds();

            // ===== grow && merge ground =====
            float angle_between_planes_deg(const Plane & a, const Plane & b) const;
            float compute_percentile(std::vector<float> values, float q) const;
            bool extract_border_band_zs(
                const GridIndex & current_idx,
                const GridIndex & neighbor_idx,
                std::vector<float> & current_border_zs,
                std::vector<float> & neighbor_border_zs) const;
            bool check_border_continuity_with_thresh(
                    const GridIndex & current_idx,
                    const GridIndex & neighbor_idx,
                    const double thresh) const;
            bool can_grow_to_neighbor(
                const GridIndex & current_idx,
                const GridIndex & neighbor_idx,
                const Plane & ref_plane) const;
            void grow_ground_groups();
            int select_main_ground_group() const;
            void mark_selected_ground_groups(const std::unordered_set<int> &ground_set);

            float compute_group_min_cell_distance(const GroundGroup &a, const GroundGroup &b) const;
            bool should_merge_group_to_main(const GroundGroup &main_group, const GroundGroup &other_group) const;
            std::unordered_set<int> collect_groups_to_merge(int main_group_id) const;
            bool find_best_adjacent_cell_pair_between_groups(
                const GroundGroup &main_group,
                const GroundGroup &other_group,
                GridIndex &main_idx_out,
                GridIndex &other_idx_out) const;
            bool check_group_merge_border_continuity(
                    const GroundGroup &main_group,
                    const GroundGroup &other_group) const;
            bool check_group_gap_compatibility(
                    const GroundGroup &main_group,
                    const GroundGroup &other_group) const;
            bool find_best_nearby_cell_pair_between_groups(
                    const GroundGroup &main_group,
                    const GroundGroup &other_group,
                    GridIndex &main_idx_out,
                    GridIndex &other_idx_out,
                    float &min_dist_out) const;
            float predict_plane_z_at_xy(
                    const Plane &plane,
                    float x,
                    float y) const;
            bool check_pair_plane_compatibility_for_gap(
                    const CellData &main_cell,
                    const CellData &other_cell,
                    double max_predict_height_diff,
                    double hard_angle_thresh) const;
            float point_to_plane_distance(
                const PointXYZIConf &pt,
                const Plane &plane) const;

            std::vector<int> extract_low_percentile_point_indices(
                const CellData &cell,
                float q) const;
            bool fit_plane_from_point_indices(
                const std::vector<int> &indices,
                Plane &plane_out) const;
            bool is_mixed_cell_suspect(const GridIndex &idx) const;
            bool find_best_neighbor_ground_plane(
                const GridIndex &idx,
                Plane &plane_out,
                GridIndex &ground_nbr_idx_out) const; 
            void recover_mixed_cells_near_ground();

            void classify_points_and_publish(const std::string &frame_id, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        private:
            LifecycleNodeWeakPtr node_;
            std::string plugin_name_;

            std::atomic_bool active_{false};            // 存储插件状态(active/inactive)

            // --------------------- Parameters ---------------------
            bool if_silence_ = false;
            bool if_debug_ = false;
            std::string input_pointcloud_frame_ = "map";          // "map"或者"body"，表示输入点云的坐标系(全局或机体坐标系)
            std::string frame_id_override_ = std::string("");       // 当不为空时发布的地面分割结果使用这个为 frame_id

            std::string input_cloud_topic_ = "/points_cropped";
            std::string input_odom_topic_ = "/Odometry";
            std::string output_ground_points_topic_ = "/ground_points";
            std::string output_non_ground_points_topic_ = "/non_ground_points";

            double min_range_ = 0.3;
            double max_range_ = 50.0;
            double min_z_ = -2.0;
            double max_z_ = 2.0;
            bool enable_voxel_downsample_ = true;
            double voxel_leaf_size_ = 0.03;       // 滤波时的体素格子

            // 三段 grid_resolution
            double near_band_max_range_ = 3.0;     // [0, 3)
            double mid_band_max_range_ = 10.0;     // [3, 10)
            double near_grid_resolution_ = 0.40;
            double mid_grid_resolution_ = 0.80;
            double far_grid_resolution_ = 1.2;

            bool if_show_grid_map_ = true;

            // plane-fit
            int min_points_per_cell_ = 4;
            double plane_fit_z_thresh_ = 0.25;
            double max_plane_flatness_ = 0.03;
            double min_plane_planarity_ = 0.2;
            bool if_show_plane_cells_ = true;

            // seed
            double seed_max_angle_deg_ = 15.0;
            double seed_local_height_range_thresh_ = 0.2;         // seed 格子内部 max_z 与 min_z 的最大差值
            double seed_height_thresh_ = 0.2;         // seed 格子内部的中心距离预测的主地面高度允许的最大差值
            double seed_search_radius_ = 2.5;         // 拿周围 2.5m 以内的点做主平面高度预测
            double ground_percentile_for_init_ = 0.15;          // 取周围点高度的 15 分位数作为主平面高度
            bool if_show_seed_cells_ = true;

            // grow && merge ground
            double grow_max_angle_deg_ = 15.0;            // 邻居格子法向和当前平面法向允许最大差值
            double grow_center_height_diff_ = 0.2;        // 邻居格子和当前平面中心最大高度差
            double grow_border_height_diff_ = 0.15;       // 格子边界连续性判断阈值
            int grow_min_neighbor_points_ = 4;        // 允许生长格子要包含的最少点数
            double border_band_width_ = 0.15;         // 取 0.15m 的边界点用来做边界连续性检测
            bool allow_no_plane_neighbor_ = true;           // 允许没有平面的格子加入
            double low_percentile_for_border_ = 0.2;        // 边界低分位数
            bool if_show_ground_groups_ = true;

            double group_merge_distance_ = 1.0;     // 平面间允许合并的最远距离
            double group_merge_height_diff_ = 0.2;      // 平面间允许合并的最大高度差
            double group_merge_angle_deg_ = 10;       // 平面间允许合并的最大法向角度差
            double grow_ground_height_diff_ = 0.15;     // 平面合并时边界连续性检查阈值

            // 在 merge_group 时若找不到相邻的格子，允许 group 最近格子之间有 gap_tolerance_ 的间隙（单位是 m）
            double gap_tolerance_ = 1.5;     
            double group_merge_gap_angle_deg_ = 20;           // group gap 时相邻格子允许最大角度差
            double group_gap_max_predict_height_diff_ = 0.1;    // group gap 时相邻格子（在对方平面中）预测高度的最大高度差

            // mixed cell recovery
            double mixed_cell_height_range_thresh_ = 0.2;      // 格子内部的高度差要有多大，才怀疑它是个“混合格子”
            double mixed_cell_low_percentile_ = 0.15;        // 取混合格子的多少分位以内的点来重新拟合平面
            int mixed_cell_min_low_points_ = 4;         // 重新拟合平面所需的最少点数
            double mixed_cell_recovered_max_angle_deg_ = 15.0;        // 恢复出来的 low plane 与邻域 ground plane 最大允许夹角
            double mixed_cell_recovered_height_diff_ = 0.15;        // low plane 和邻域 ground 的最大允许高度差
            double mixed_cell_point_to_plane_thresh_ = 0.08;        // low plane 中距离平面小于这个值的才算是地面点

            // publish ground-points/non-grounds
            double point_to_plane_ground_thresh_ = 0.15;      // 对于地面格子中的点，如果距离格子平面大于这个值，就算作障碍物点
            double confidence_thresh_ = 0.1;                 // 置信度阈值，低于这个置信度的点会被划分为非地面点

            // --------------------- Subscriptions ---------------------
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

            // --------------------- Lifecycle publishers ---------------------
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_pub_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_fit_pub_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr seed_cells_pub_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_group_pub_;

            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_cloud_pub_;

            // --------------------- Runtime state ---------------------
            std::vector<PointXYZIConf> processed_points_;
            std::unordered_map<GridIndex, CellData, GridIndexHash> grid_map_;
            std::vector<GroundGroup> ground_groups_;

            bool has_estimated_ground_z_ = false;
            float estimated_ground_z_ = 0.0f;
            double robot_x_, robot_y_, robot_z_;        // 最新的机器人位置
            double frame_robot_x_ = 0.0, frame_robot_y_ = 0.0, frame_robot_z_ = 0.0;      // 一帧中的机器人位置
            bool has_frame_pose_ = false;
            bool has_odom_{false};
            mutable std::mutex odom_mutex_;
    };
}       // namespace ground_segmentor

#endif  // GROUND_SEGMENTOR__GROUND_SEGMENTOR_HPP_