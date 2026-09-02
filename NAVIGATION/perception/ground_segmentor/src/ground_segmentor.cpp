#include "ground_segmentor/ground_segmentor.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace
{
    constexpr std::size_t kMaxPcaSamplePoints = 48;

    std::vector<int> sample_indices_evenly(const std::vector<int> &indices, std::size_t max_samples)
    {
        if (indices.size() <= max_samples)
            return indices;

        std::vector<int> sampled;
        sampled.reserve(max_samples);

        // 保证覆盖首尾元素；一列 n 个数，中间有 n-1 个空
        const double step = static_cast<double>(indices.size() - 1) /
                            static_cast<double>(max_samples - 1);
        for (std::size_t i = 0; i < max_samples; ++i)
        {
            // std::llround：将浮点数四舍五入到最近的整数（返回 long long 类型）
            const std::size_t idx = static_cast<std::size_t>(std::llround(step * static_cast<double>(i)));
            sampled.push_back(indices[std::min(idx, indices.size() - 1)]);
        }
        return sampled;
    }
}

namespace ground_segmentor
{
    void GroundSegmentor::configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name)
    {
        node_ = parent;
        plugin_name_ = plugin_name;

        auto node = node_.lock();
        if (!node) 
        {
            throw std::runtime_error("GroundSegmentor::configure() failed: parent node is expired.");
        }

        load_parameters(node);
        create_publishers(node);
        reset_runtime_state();

        RCLCPP_INFO(
            node->get_logger(),
            "GroundSegmentor plugin '%s' configured.",
            plugin_name_.c_str()
        );
    }

    void GroundSegmentor::load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const std::string p = plugin_name_ + ".";

        if_silence_ = declare_or_get_parameter<bool>(node, p + "if_silence", false);
        if_debug_ = declare_or_get_parameter<bool>(node, p + "if_debug", true);

        // frame_id_override_ = declare_or_get_parameter<std::string>(node, p + "frame_id_override", "aft_mapped_continuous");
        frame_id_override_ = declare_or_get_parameter<std::string>(node, p + "frame_id_override", std::string(""));
        input_pointcloud_frame_ = declare_or_get_parameter<std::string>(node, p + "input_pointcloud_frame", "map");

        input_cloud_topic_ = declare_or_get_parameter<std::string>(node, p + "input_cloud_topic", "/points_cropped");
        input_odom_topic_ = declare_or_get_parameter<std::string>(node, p + "input_odom_topic", "/Odometry");
        output_ground_points_topic_ = declare_or_get_parameter<std::string>(node, p + "output_ground_points_topic", "/ground_points");
        output_non_ground_points_topic_ = declare_or_get_parameter<std::string>(node, p + "output_non_ground_points_topic", "/non_ground_points");

        min_range_ = declare_or_get_parameter<double>(node, p + "min_range", 0.1);
        max_range_ = declare_or_get_parameter<double>(node, p + "max_range", 20.0);
        min_z_ = declare_or_get_parameter<double>(node, p + "min_z", -2.0);
        max_z_ = declare_or_get_parameter<double>(node, p + "max_z", 2.0);
        enable_voxel_downsample_ = declare_or_get_parameter<bool>(node, p + "enable_voxel_downsample", false);
        voxel_leaf_size_ = declare_or_get_parameter<double>(node, p + "voxel_leaf_size", 0.03);

        near_band_max_range_ = declare_or_get_parameter<double>(node, p + "near_band_max_range", 4.0);
        mid_band_max_range_ = declare_or_get_parameter<double>(node, p + "mid_band_max_range", 10.0);
        near_grid_resolution_ = declare_or_get_parameter<double>(node, p + "near_grid_resolution", 0.80);
        mid_grid_resolution_ = declare_or_get_parameter<double>(node, p + "mid_grid_resolution", 1.2);
        far_grid_resolution_ = declare_or_get_parameter<double>(node, p + "far_grid_resolution", 1.6);
        if_show_grid_map_ = declare_or_get_parameter<bool>(node, p + "if_show_grid_map", true);

        min_points_per_cell_ = declare_or_get_parameter<int>(node, p + "min_points_per_cell", 4);
        plane_fit_z_thresh_ = declare_or_get_parameter<double>(node, p + "plane_fit_z_thresh", 0.2);
        max_plane_flatness_ = declare_or_get_parameter<double>(node, p + "max_plane_flatness", 0.03);
        min_plane_planarity_ = declare_or_get_parameter<double>(node, p + "min_plane_planarity", 0.02);
        if_show_plane_cells_ = declare_or_get_parameter<bool>(node, p + "if_show_plane_cells", true);

        seed_max_angle_deg_ = declare_or_get_parameter<double>(node, p + "seed_max_angle_deg", 15.0);
        seed_local_height_range_thresh_ = declare_or_get_parameter<double>(node, p + "seed_local_height_range_thresh", 0.2);
        seed_height_thresh_ = declare_or_get_parameter<double>(node, p + "seed_height_thresh", 0.2);
        seed_search_radius_ = declare_or_get_parameter<double>(node, p + "seed_search_radius", 2.5);
        ground_percentile_for_init_ = declare_or_get_parameter<double>(node, p + "ground_percentile_for_init", 0.15);
        if_show_seed_cells_ = declare_or_get_parameter<bool>(node, p + "if_show_seed_cells", true);

        grow_max_angle_deg_ = declare_or_get_parameter<double>(node, p + "grow_max_angle_deg", 15.0);
        grow_center_height_diff_ = declare_or_get_parameter<double>(node, p + "grow_center_height_diff", 0.2);
        grow_border_height_diff_ = declare_or_get_parameter<double>(node, p + "grow_border_height_diff", 0.15);
        grow_min_neighbor_points_ = declare_or_get_parameter<int>(node, p + "grow_min_neighbor_points", 4);
        border_band_width_ = declare_or_get_parameter<double>(node, p + "border_band_width", 0.25);
        allow_no_plane_neighbor_ = declare_or_get_parameter<bool>(node, p + "allow_no_plane_neighbor", true);
        low_percentile_for_border_ = declare_or_get_parameter<double>(node, p + "low_percentile_for_border", 0.3);
        if_show_ground_groups_ = declare_or_get_parameter<bool>(node, p + "if_show_ground_groups", true);

        // group_merge_distance_ = node_->declare_parameter("group_merge_distance", 1.0);
        group_merge_height_diff_ = declare_or_get_parameter<double>(node, p + "group_merge_height_diff", 0.15);
        group_merge_angle_deg_ = declare_or_get_parameter<double>(node, p + "group_merge_angle_deg", 15.0);
        grow_ground_height_diff_ = declare_or_get_parameter<double>(node, p + "grow_ground_height_diff", 0.15);
        gap_tolerance_ = declare_or_get_parameter<double>(node, p + "gap_tolerance", 1.5);
        group_merge_gap_angle_deg_ = declare_or_get_parameter<double>(node, p + "group_merge_gap_angle_deg", 20);
        group_gap_max_predict_height_diff_ = declare_or_get_parameter<double>(node, p + "group_gap_max_predict_height_diff", 0.1);

        mixed_cell_height_range_thresh_ = declare_or_get_parameter<double>(node, p + "mixed_cell_height_range_thresh", 0.2);
        mixed_cell_low_percentile_ = declare_or_get_parameter<double>(node, p + "mixed_cell_low_percentile", 0.12);
        mixed_cell_min_low_points_ = declare_or_get_parameter<int>(node, p + "mixed_cell_min_low_points", 4);
        mixed_cell_recovered_max_angle_deg_ = declare_or_get_parameter<double>(node, p + "mixed_cell_recovered_max_angle_deg", 40.0);
        mixed_cell_recovered_height_diff_ = declare_or_get_parameter<double>(node, p + "mixed_cell_recovered_height_diff", 0.15);
        mixed_cell_point_to_plane_thresh_ = declare_or_get_parameter<double>(node, p + "mixed_cell_point_to_plane_thresh", 0.05);
        
        point_to_plane_ground_thresh_ = declare_or_get_parameter<double>(node, p + "point_to_plane_ground_thresh", 0.15);
        confidence_thresh_ = declare_or_get_parameter<double>(node, p + "confidence_thresh", 0.1);

        RCLCPP_INFO_STREAM(
            node->get_logger(),
            "GroundSegmentor parameters loaded."
            << " plugin_name=" << plugin_name_
            << ", if_silence=" << if_silence_
            << ", if_debug=" << if_debug_
            << ", frame_id_override=" << frame_id_override_
            << ", input_pointcloud_frame=" << input_pointcloud_frame_
            << ", input_cloud_topic=" << input_cloud_topic_
            << ", input_odom_topic=" << input_odom_topic_
            << ", output_ground_points_topic=" << output_ground_points_topic_
            << ", output_non_ground_points_topic=" << output_non_ground_points_topic_
        );
    }

    void GroundSegmentor::create_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        grid_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map_output", rclcpp::SensorDataQoS());
        plane_fit_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_fit_output", rclcpp::SensorDataQoS());
        seed_cells_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_seed_output", rclcpp::SensorDataQoS());
        ground_group_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_group_output", rclcpp::SensorDataQoS());
        ground_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(output_ground_points_topic_, rclcpp::SensorDataQoS());
        non_ground_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(output_non_ground_points_topic_, rclcpp::SensorDataQoS());
    }

    void GroundSegmentor::reset_runtime_state()
    {
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            robot_x_ = 0.0, robot_y_ = 0.0, robot_z_ = 0.0; 
            has_odom_ = false;
        }

        processed_points_.clear();
        grid_map_.clear();
        ground_groups_.clear();

        has_estimated_ground_z_ = false;
        estimated_ground_z_ = 0.0f;
        frame_robot_x_ = 0.0, frame_robot_y_ = 0.0, frame_robot_z_ = 0.0;
        has_frame_pose_ = false;
    }

    void GroundSegmentor::activate()
    {
        auto node = node_.lock();
        if (!node) 
        {
            throw std::runtime_error("GroundSegmentor::activate() failed: parent node is expired.");
        }

        // 激活发布者
        if (grid_map_pub_) 
            grid_map_pub_->on_activate();
        if (plane_fit_pub_) 
            plane_fit_pub_->on_activate();
        if (seed_cells_pub_) 
            seed_cells_pub_->on_activate();
        if (ground_group_pub_) 
            ground_group_pub_->on_activate();
        if (ground_cloud_pub_) 
            ground_cloud_pub_->on_activate();
        if (non_ground_cloud_pub_) 
            non_ground_cloud_pub_->on_activate();
        
        create_subscriptions(node);

        active_.store(true);

        RCLCPP_INFO(
            node->get_logger(),
            "GroundSegmentor plugin '%s' activated.",
            plugin_name_.c_str()
        );
    }

    void GroundSegmentor::create_subscriptions(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&GroundSegmentor::pointcloud_callback, this, std::placeholders::_1)
        );

        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&GroundSegmentor::odom_callback, this, std::placeholders::_1)
        );
    }

    void GroundSegmentor::deactivate()
    {
        auto node = node_.lock();

        active_.store(false);

        // 重置 subscription，停止处理高频点云
        cloud_sub_.reset();
        odom_sub_.reset();

        if (grid_map_pub_) 
            grid_map_pub_->on_deactivate();
        if (plane_fit_pub_) 
            plane_fit_pub_->on_deactivate();
        if (seed_cells_pub_) 
            seed_cells_pub_->on_deactivate();
        if (ground_group_pub_) 
            ground_group_pub_->on_deactivate();
        if (ground_cloud_pub_) 
            ground_cloud_pub_->on_deactivate();
        if (non_ground_cloud_pub_) 
            non_ground_cloud_pub_->on_deactivate();

        if (node) 
        {
            RCLCPP_INFO(
                node->get_logger(),
                "GroundSegmentor plugin '%s' deactivated.",
                plugin_name_.c_str()
            );
        }
    }

    void GroundSegmentor::cleanup()
    {
        active_.store(false);

        cloud_sub_.reset();
        odom_sub_.reset();

        grid_map_pub_.reset();
        plane_fit_pub_.reset();
        seed_cells_pub_.reset();
        ground_group_pub_.reset();
        ground_cloud_pub_.reset();
        non_ground_cloud_pub_.reset();


        reset_runtime_state();

        node_.reset();
        plugin_name_.clear();
    }

    void GroundSegmentor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!active_.load())                    // 只在插件 active 状态时执行回调
            return;

        std::lock_guard<std::mutex> lock(odom_mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_z_ = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    // reset grid_map_
    void GroundSegmentor::reset_grid_map()
    {
        grid_map_.clear();
    }

    // 根据点的坐标判断它属于那个圈层
    int GroundSegmentor::get_range_band(const PointXYZIConf &pt) const
    {
        auto node = node_.lock();
        if (!node) 
            return -1;          // 返回一个无效值

        float r = 0.0;
        if(input_pointcloud_frame_ == "map")
        {
            const float dx = pt.x - static_cast<float>(frame_robot_x_);
            const float dy = pt.y - static_cast<float>(frame_robot_y_);
            r = std::sqrt(dx * dx + dy * dy);
        }
        else if(input_pointcloud_frame_ == "body")
        {
            r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "input_pointcloud_frame_ NOT valid, expected value is map/body !!!"
            );
            r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        }

        if (r < static_cast<float>(near_band_max_range_))
            return 0;
        if (r < static_cast<float>(mid_band_max_range_))
            return 1;
        return 2;
    }

    // 已知点的圈层，返回其格子的 grid_resolution
    float GroundSegmentor::get_grid_resolution_for_band(int band) const
    {
        if (band == 0)
            return static_cast<float>(near_grid_resolution_);
        if (band == 1)
            return static_cast<float>(mid_grid_resolution_);
        return static_cast<float>(far_grid_resolution_);
    }

    float GroundSegmentor::get_band_min_range(int band) const
    {
        if (band == 0)
            return 0.0f;
        if (band == 1)
            return static_cast<float>(near_band_max_range_);
        return static_cast<float>(mid_band_max_range_);
    }

    float GroundSegmentor::get_band_max_range(int band) const
    {
        if (band == 0)
            return static_cast<float>(near_band_max_range_);
        if (band == 1)
            return static_cast<float>(mid_band_max_range_);
        return std::numeric_limits<float>::infinity();
    }

    // 输入点，计算其格子的 grid_resolution 并划分到格子，返回格子索引
    GridIndex GroundSegmentor::point_to_grid_index(const PointXYZIConf &pt) const
    {
        GridIndex idx;
        idx.band = get_range_band(pt);

        const float res = get_grid_resolution_for_band(idx.band);
        idx.ix = static_cast<int>(std::floor(pt.x / res));
        idx.iy = static_cast<int>(std::floor(pt.y / res));
        return idx;
    }

    // 构建 2.5D 网格地图
    void GroundSegmentor::build_grid_map()
    {
        auto node = node_.lock();
        if (!node) 
            return;        

        reset_grid_map();           

        if (processed_points_.empty())
            return;

        for (std::size_t i = 0; i < processed_points_.size(); ++i)
        {
            const auto &pt = processed_points_[i];
            const GridIndex idx = point_to_grid_index(pt);

            // 获取对应格子内容并赋值
            auto &cell = grid_map_[idx];
            cell.band = idx.band;
            cell.cell_resolution = get_grid_resolution_for_band(idx.band);

            cell.point_indices.push_back(static_cast<int>(i));

            if (pt.z < cell.min_z)
                cell.min_z = pt.z;
            if (pt.z > cell.max_z)
                cell.max_z = pt.z;

            cell.mean_x += pt.x;
            cell.mean_y += pt.y;
            cell.mean_z += pt.z;
        }

        // 计算每个格子的中心
        for (auto &kv : grid_map_)
        {
            auto &cell = kv.second;
            const float n = static_cast<float>(cell.point_indices.size());
            if (n > 0.0f)
            {
                cell.mean_x /= n;
                cell.mean_y /= n;
                cell.mean_z /= n;
            }
        }

        if (!if_silence_)
        {
            std::size_t band0_cells = 0;
            std::size_t band1_cells = 0;
            std::size_t band2_cells = 0;

            // 格子中最多、最少有多少个点
            std::size_t max_points_in_cell = 0;
            std::size_t min_points_in_cell = std::numeric_limits<std::size_t>::max();

            for (const auto &kv : grid_map_)
            {
                const auto sz = kv.second.point_indices.size();
                if (sz > max_points_in_cell)
                    max_points_in_cell = sz;
                if (sz < min_points_in_cell)
                    min_points_in_cell = sz;

                if (kv.first.band == 0)
                    ++band0_cells;
                else if (kv.first.band == 1)
                    ++band1_cells;
                else
                    ++band2_cells;
            }

            if (grid_map_.empty())
                min_points_in_cell = 0;

            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "[GridMap] cells=%zu, points=%zu, band0=%zu(res=%.2f), band1=%zu(res=%.2f), band2=%zu(res=%.2f), min_pts_per_cell=%zu, max_pts_per_cell=%zu",
                grid_map_.size(),
                processed_points_.size(),
                band0_cells, near_grid_resolution_,
                band1_cells, mid_grid_resolution_,
                band2_cells, far_grid_resolution_,
                min_points_in_cell, max_points_in_cell);
        }
    }

    float GroundSegmentor::compute_planarity(float ev0, float ev1, float ev2) const
    {
        const float eps = 1e-6f;
        if (ev2 < eps)
            return 0.0f;
        return (ev1 - ev0) / (ev2 + eps);
    }

    float GroundSegmentor::compute_flatness(float ev0, float ev2) const
    {
        const float eps = 1e-6f;
        if (ev2 < eps)
            return std::numeric_limits<float>::max();
        return ev0 / (ev2 + eps);
    }

    // 为单个格子做 PCA 拟合平面，拟合成功则返回 true
    bool GroundSegmentor::fit_plane_for_cell(CellData &cell)
    {
        cell.has_plane = false;

        const std::size_t point_num = cell.point_indices.size();
        if (point_num < static_cast<std::size_t>(min_points_per_cell_))         // 点数不够
            return false;

        const float local_height_range = cell.max_z - cell.min_z;
        // 怀疑是 mixed-cell
        const bool mixed_like = local_height_range > static_cast<float>(plane_fit_z_thresh_);      

        auto assign_plane_to_cell = [&](const Plane &plane_candidate)
        {
            cell.has_plane = true;
            cell.plane = plane_candidate;
        };

        // 检查拟合出来的平面是否合法
        auto validate_plane = [&](const Plane &plane_candidate, bool from_low_percentile) -> bool
        {
            const float flatness = compute_flatness(plane_candidate.ev0, plane_candidate.ev2);
            const float planarity = compute_planarity(plane_candidate.ev0, plane_candidate.ev1, plane_candidate.ev2);
            if (flatness > static_cast<float>(max_plane_flatness_))
                return false;
            if (planarity < static_cast<float>(min_plane_planarity_))
                return false;

            if (from_low_percentile)
            {
                // 如果是低分位拟合出来的，额外检查是否接近水平面
                /*
                    注意，虽然后边的 SEED 筛选阶段会做法向偏角的检验，
                    但对于一个被误检的障碍物格子，如果这里通过了，那么它的 cell 就会被写上 has_plane = true，
                    这不会直接影响到 SEED 生成，但可能会与影响到后边的 SEED 生长、recovery 等步骤；
                    主打一个严谨！！！😜🤪😆
                */
                Eigen::Vector3f normal(plane_candidate.nx, plane_candidate.ny, plane_candidate.nz);
                normal.normalize();
                const float dot = std::clamp(normal.z(), -1.0f, 1.0f);
                const float angle_deg = std::acos(dot) * 180.0f / static_cast<float>(M_PI);
                if (angle_deg > static_cast<float>(seed_max_angle_deg_ + 10.0))
                    return false;
            }

            return true;
        };

        // 对均匀采样后的点进行 plane-fit
        auto try_fit_indices = [&](const std::vector<int> &indices, bool from_low_percentile) -> bool
        {
            if (indices.size() < static_cast<std::size_t>(min_points_per_cell_))
                return false;

            Plane plane_candidate;
            if (!fit_plane_from_point_indices(indices, plane_candidate))        // 尝试拟合平面
                return false;

            if (!validate_plane(plane_candidate, from_low_percentile))
                return false;

            assign_plane_to_cell(plane_candidate);      // 将成功拟合的平面填入格子
            return true;
        };

        // Layer-1: clean cell 直接做轻量 sampled PCA
        if (!mixed_like)
        {
            const std::vector<int> sampled = sample_indices_evenly(cell.point_indices, kMaxPcaSamplePoints);
            return try_fit_indices(sampled, false);
        }

        // Layer-2: 对于 mixed-cell，只取下百分位以内的点做采样 PCA
        const float low_q = std::clamp(static_cast<float>(mixed_cell_low_percentile_), 0.08f, 0.35f);
        std::vector<int> low_indices = extract_low_percentile_point_indices(cell, low_q);
        if (!low_indices.empty())
        {
            low_indices = sample_indices_evenly(low_indices, kMaxPcaSamplePoints);
            if (try_fit_indices(low_indices, true))
                return true;
        }

        // Layer-3: fallback sampled PCA（这里会对那些 mixed-cell 做整体的 PCA，是“最后的尝试”）
        // 仍保留全语义兜底，不再像旧优化版那样因为 height range 大就直接 return false。
        const std::vector<int> sampled = sample_indices_evenly(cell.point_indices, kMaxPcaSamplePoints);
        return try_fit_indices(sampled, false);
    }

    // 尝试为所有格子拟合平面；包含平面的格子可作为候选种子
    void GroundSegmentor::fit_planes_for_all_cells()
    {
        auto node = node_.lock();
        if (!node) 
            return;   

        if (grid_map_.empty())
            return;

        std::size_t filtered_blocks = 0;
        for (auto &cell : grid_map_)
        {
            if (fit_plane_for_cell(cell.second))
                ++filtered_blocks;
        }

        if (!if_silence_)
        {
            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(), 1000,
                "[PlaneFit]: grid_map=%zu, qualified=%zu, unqualified=%zu",
                grid_map_.size(), filtered_blocks, grid_map_.size() - filtered_blocks);
        }
    }

    // 取机器人周围的格子，根据低分位数来预测主地面高度
    bool GroundSegmentor::estimate_ground_height_from_nearby_points()
    {
        auto node = node_.lock();
        if (!node) 
            return false;   

        has_estimated_ground_z_ = false;

        if (processed_points_.empty())
            return false;

        std::vector<float> nearby_zs;
        nearby_zs.reserve(processed_points_.size());

        const float radius_sq = static_cast<float>(seed_search_radius_ * seed_search_radius_);

        for (const auto &pt : processed_points_)
        {

            float dist_xy_sq = 0.0;

            if(input_pointcloud_frame_ == "map")
            {
                const float dx = pt.x - static_cast<float>(frame_robot_x_);
                const float dy = pt.y - static_cast<float>(frame_robot_y_);
                dist_xy_sq = dx * dx + dy * dy;
            }
            else if(input_pointcloud_frame_ == "body")
            {
                dist_xy_sq = pt.x * pt.x + pt.y * pt.y;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    1000,
                    "input_pointcloud_frame_ NOT valid, expected value is map/body !!!"
                );
                dist_xy_sq = pt.x * pt.x + pt.y * pt.y;
            }

            if (dist_xy_sq <= radius_sq)
                nearby_zs.push_back(pt.z);
        }

        if (nearby_zs.size() < 10)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "[SEED] Nearby points around robot is under 10, cannot estimate ground height.");
            return false;
        }

        std::sort(nearby_zs.begin(), nearby_zs.end());

        float q = static_cast<float>(ground_percentile_for_init_);
        q = std::clamp(q, 0.0f, 1.0f);

        // 根据分位数计算索引
        const std::size_t idx =
            std::min(static_cast<std::size_t>(q * static_cast<float>(nearby_zs.size() - 1)),
                     nearby_zs.size() - 1);

        estimated_ground_z_ = nearby_zs[idx];
        has_estimated_ground_z_ = true;

        if (!if_silence_)
        {
            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "[SEED] Estimate-ground-height: %.4f, nearby points num: %zu",
                estimated_ground_z_, nearby_zs.size());
        }
        return true;
    }

    // 计算格子法向与垂直夹角
    float GroundSegmentor::compute_seed_angle_deg(const CellData & cell) const
    {
        Eigen::Vector3f normal(cell.plane.nx, cell.plane.ny, cell.plane.nz);
        normal.normalize();

        const float dot = std::clamp(normal.z(), -1.0f, 1.0f);
        const float angle_rad = std::acos(dot);
        return angle_rad * 180.0f / static_cast<float>(M_PI);
    }

    // 判断一个格子是否是种子
    bool GroundSegmentor::is_seed_cell(CellData & cell) const
    {
        cell.is_seed = false;

        if (!cell.has_plane)
            return false;

        const float angle_deg = compute_seed_angle_deg(cell);
        if (angle_deg > static_cast<float>(seed_max_angle_deg_))        // 法向偏角不能太大
            return false;

        const float local_height_range = cell.max_z - cell.min_z;
        if (local_height_range > static_cast<float>(seed_local_height_range_thresh_))       // 内部起伏不能太大
            return false;

        if (!has_estimated_ground_z_)       
            return false;

        const float delta_z = cell.plane.cz - static_cast<float>(estimated_ground_z_);
        if (delta_z > static_cast<float>(seed_height_thresh_))      // 格子中心高度距离主平面高度不能差太远
            return false;

        return true;
    }

    // 估计主平面并筛选出种子
    void GroundSegmentor::generate_ground_seeds()
    {
        auto node = node_.lock();
        if (!node) 
            return ;   

        if (grid_map_.empty())
            return;

        for (auto &pt : grid_map_)
            pt.second.is_seed = false;

        if (!estimate_ground_height_from_nearby_points())
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(), 1000,
                "[SEED] Cannot estimate ground-height, returning.");
            return;
        }

        std::size_t seed_count = 0;
        for (auto &pt : grid_map_)
        {
            auto &cell = pt.second;
            if (is_seed_cell(cell))
            {
                cell.is_seed = true;
                ++seed_count;
            }
        }

        if (!if_silence_)
        {
            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "[SEED] Estimate-ground-height: %.2f, seed num: %zu",
                estimated_ground_z_, seed_count);
        }
    }

    std::vector<GridIndex> GroundSegmentor::get_8_neighbors(const GridIndex & idx) const
    {
        std::vector<GridIndex> neighbors;
        neighbors.reserve(8);

        // 只在同 band 内做 8 邻域
        for (int i = -1; i <= 1; ++i)
        {
            for (int j = -1; j <= 1; ++j)
            {
                if (i == 0 && j == 0)
                    continue;

                GridIndex nbr;
                nbr.band = idx.band;
                nbr.ix = idx.ix + i;
                nbr.iy = idx.iy + j;
                neighbors.emplace_back(nbr);
            }
        }
        return neighbors;
    }

    bool GroundSegmentor::grid_index_exists(const GridIndex & idx) const
    {
        return grid_map_.find(idx) != grid_map_.end();
    }

    float GroundSegmentor::angle_between_planes_deg(const Plane & a, const Plane & b) const
    { 
        Eigen::Vector3f na(a.nx, a.ny, a.nz);
        Eigen::Vector3f nb(b.nx, b.ny, b.nz);
        na.normalize();
        nb.normalize();

        const float dot_result = std::clamp(na.dot(nb), -1.0f, 1.0f);
        const float radian = std::acos(dot_result);
        return radian * 180.0f / static_cast<float>(M_PI);
    }

    float GroundSegmentor::compute_percentile(std::vector<float> values, float q) const
    {
        if (values.empty())
            return std::numeric_limits<float>::quiet_NaN();

        q = std::clamp(q, 0.0f, 1.0f);
        std::sort(values.begin(), values.end());
        const std::size_t idx = std::min(
            static_cast<std::size_t>(static_cast<float>(values.size() - 1) * q),
            values.size() - 1);
        return values[idx];
    }

    // 取两个相邻格子交界处特定范围内的所有点的 z 轴高度
    bool GroundSegmentor::extract_border_band_zs(
        const GridIndex & current_idx,
        const GridIndex & neighbor_idx,
        std::vector<float> & current_border_zs,
        std::vector<float> & neighbor_border_zs) const
    {
        current_border_zs.clear();
        neighbor_border_zs.clear();

        auto it_cur = grid_map_.find(current_idx);
        auto it_nbr = grid_map_.find(neighbor_idx);
        if (it_cur == grid_map_.end() || it_nbr == grid_map_.end())
            return false;

        const auto &cur_cell = it_cur->second;      // 获取格子内容
        const auto &nbr_cell = it_nbr->second;

        const float cur_half = 0.5f * cur_cell.cell_resolution;
        const float nbr_half = 0.5f * nbr_cell.cell_resolution;

        const float cur_band = std::min(static_cast<float>(border_band_width_), 0.5f * cur_cell.cell_resolution);
        const float nbr_band = std::min(static_cast<float>(border_band_width_), 0.5f * nbr_cell.cell_resolution);

        const float cur_cx = cur_cell.mean_x;           // 取要比较的格子的中心坐标
        const float cur_cy = cur_cell.mean_y;
        const float nbr_cx = nbr_cell.mean_x;
        const float nbr_cy = nbr_cell.mean_y;

        const int dx = neighbor_idx.ix - current_idx.ix;
        const int dy = neighbor_idx.iy - current_idx.iy;

        for (const int pid : cur_cell.point_indices)
        {
            const auto &pt = processed_points_[pid];
            const float local_x = pt.x - cur_cx;        // 当前点距离格子中心的相对位置大小
            const float local_y = pt.y - cur_cy;

            bool in_band = false;

            if (dx == 1 && dy == 0)          in_band = (local_x >= cur_half - cur_band);
            else if (dx == -1 && dy == 0)    in_band = (local_x <= -cur_half + cur_band);
            else if (dx == 0 && dy == 1)     in_band = (local_y >= cur_half - cur_band);
            else if (dx == 0 && dy == -1)    in_band = (local_y <= -cur_half + cur_band);
            else if (dx == 1 && dy == 1)     in_band = (local_x >= cur_half - cur_band || local_y >= cur_half - cur_band);
            else if (dx == 1 && dy == -1)    in_band = (local_x >= cur_half - cur_band || local_y <= -cur_half + cur_band);
            else if (dx == -1 && dy == 1)    in_band = (local_x <= -cur_half + cur_band || local_y >= cur_half - cur_band);
            else if (dx == -1 && dy == -1)   in_band = (local_x <= -cur_half + cur_band || local_y <= -cur_half + cur_band);

            if (in_band)
                current_border_zs.push_back(pt.z);
        }

        for (const int pid : nbr_cell.point_indices)
        {
            const auto &pt = processed_points_[pid];
            const float local_x = pt.x - nbr_cx;
            const float local_y = pt.y - nbr_cy;

            bool in_band = false;

            if (dx == 1 && dy == 0)          in_band = (local_x <= -nbr_half + nbr_band);
            else if (dx == -1 && dy == 0)    in_band = (local_x >= nbr_half - nbr_band);
            else if (dx == 0 && dy == 1)     in_band = (local_y <= -nbr_half + nbr_band);
            else if (dx == 0 && dy == -1)    in_band = (local_y >= nbr_half - nbr_band);
            else if (dx == 1 && dy == 1)     in_band = (local_x <= -nbr_half + nbr_band || local_y <= -nbr_half + nbr_band);
            else if (dx == 1 && dy == -1)    in_band = (local_x <= -nbr_half + nbr_band || local_y >= nbr_half - nbr_band);
            else if (dx == -1 && dy == 1)    in_band = (local_x >= nbr_half - nbr_band || local_y <= -nbr_half + nbr_band);
            else if (dx == -1 && dy == -1)   in_band = (local_x >= nbr_half - nbr_band || local_y >= nbr_half - nbr_band);

            if (in_band)
                neighbor_border_zs.push_back(pt.z);
        }

        return (!current_border_zs.empty() && !neighbor_border_zs.empty());
    }

    bool GroundSegmentor::check_border_continuity_with_thresh(
        const GridIndex & current_idx,
        const GridIndex & neighbor_idx,
        const double thresh) const
    {
        auto node = node_.lock();
        if (!node) 
            return false;   

        std::vector<float> curr_zs, neigh_zs;
        if (!extract_border_band_zs(current_idx, neighbor_idx, curr_zs, neigh_zs))
            return false;

        const float q = static_cast<float>(low_percentile_for_border_);
        const float curr = compute_percentile(curr_zs, q);
        const float neigh = compute_percentile(neigh_zs, q);

        if (!std::isfinite(curr) || !std::isfinite(neigh))
        {
            RCLCPP_WARN(
                node->get_logger(),
                "[SeedGrow] Boundary point's z is NOT finite, returning...");
            return false;
        }

        return std::fabs(curr - neigh) <= static_cast<float>(thresh);
    }

    bool GroundSegmentor::can_grow_to_neighbor(
        const GridIndex & current_idx,
        const GridIndex & neighbor_idx,
        const Plane & ref_plane) const
    {
        auto it_curr = grid_map_.find(current_idx);
        auto it_neigh = grid_map_.find(neighbor_idx);
        if (it_curr == grid_map_.end() || it_neigh == grid_map_.end())
            return false;

        const auto &curr_cell = it_curr->second;
        const auto &neigh_cell = it_neigh->second;

        if (neigh_cell.group_id >= 0)       // 已经分过组
            return false;

        if (neigh_cell.point_indices.size() < static_cast<std::size_t>(grow_min_neighbor_points_))
            return false;           // 点太少

        if (!check_border_continuity_with_thresh(current_idx, neighbor_idx, grow_border_height_diff_))
            return false;

        if (neigh_cell.has_plane)
        {
            if (std::fabs(neigh_cell.mean_z - curr_cell.mean_z) > static_cast<float>(grow_center_height_diff_))
                return false;

            const float d_angle = angle_between_planes_deg(neigh_cell.plane, ref_plane);
            if (d_angle > static_cast<float>(grow_max_angle_deg_))
                return false;

            return true;
        }

        if (allow_no_plane_neighbor_)
            return true;

        return false;
    }

    // 重置平面组
    void GroundSegmentor::reset_ground_groups()
    {
        ground_groups_.clear();

        for (auto &kv : grid_map_)
        {
            kv.second.group_id = -1;
            kv.second.is_ground = false;
        }
    }

    void GroundSegmentor::grow_ground_groups()
    {
        auto node = node_.lock();
        if (!node) 
            return ;   

        reset_ground_groups();

        int next_group_id = 0;

        for (auto &kv : grid_map_)
        {
            const GridIndex seed_idx = kv.first;        // 点所在格子的索引
            auto &seed_cell = kv.second;        // 格子内容

            if (!seed_cell.is_seed)         // 不是种子
                continue;
            if (seed_cell.group_id >= 0)        // 已经分过组
                continue;

            GroundGroup group;
            group.id = next_group_id;
            group.ref_plane = seed_cell.plane;

            std::queue<GridIndex> q;                // 待探索的格子索引（必须是 ground 的一部分）
            q.push(seed_idx);

            seed_cell.group_id = group.id;
            seed_cell.is_ground = true;

            while (!q.empty())
            {
                const GridIndex cur_idx = q.front();
                q.pop();

                group.cells.push_back(cur_idx);

                const auto neighbors = get_8_neighbors(cur_idx);
                for (const auto &nbr_idx : neighbors)
                {
                    if (!grid_index_exists(nbr_idx))
                        continue;

                    auto &nbr_cell = grid_map_[nbr_idx];
                    if (nbr_cell.group_id >= 0)
                        continue;

                    if (!can_grow_to_neighbor(cur_idx, nbr_idx, group.ref_plane))
                        continue;

                    nbr_cell.group_id = group.id;
                    nbr_cell.is_ground = true;
                    q.push(nbr_idx);
                }
            }

            ground_groups_.push_back(group);
            ++next_group_id;
        }

        if (!if_silence_)
        {
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "[SeedGrow] ground_groups=%zu", ground_groups_.size());
        }
    }

    // 选取距离机器人最近的一个 ground-group 作为 main-ground-group
    int GroundSegmentor::select_main_ground_group() const
    {
        auto node = node_.lock();
        if (!node) 
            return -1;   

        if (ground_groups_.empty())
            return -1;

        int best_group_id = -1;
        float best_dist_sq = std::numeric_limits<float>::max();

        for (const auto &group : ground_groups_)
        {
            for (const auto &idx : group.cells)
            {
                auto cell_it = grid_map_.find(idx);
                if (cell_it == grid_map_.end())
                    continue;

                const auto &cell = cell_it->second;

                float dist_sq = 0.0;

                if(input_pointcloud_frame_ == "map")
                {
                    const float dx = cell.mean_x - static_cast<float>(frame_robot_x_);
                    const float dy = cell.mean_y - static_cast<float>(frame_robot_y_);
                    dist_sq = dx * dx + dy * dy;
                }
                else if(input_pointcloud_frame_ == "body")
                {
                    dist_sq = cell.mean_x * cell.mean_x + cell.mean_y * cell.mean_y;
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(
                        node->get_logger(),
                        *node->get_clock(),
                        1000,
                        "input_pointcloud_frame_ NOT valid, expected value is map/body !!!"
                    );
                    dist_sq = cell.mean_x * cell.mean_x + cell.mean_y * cell.mean_y;
                }

                if (dist_sq < best_dist_sq)
                {
                    best_dist_sq = dist_sq;
                    best_group_id = group.id;
                }
            }
        }

        return best_group_id;
    }

    void GroundSegmentor::mark_selected_ground_groups(const std::unordered_set<int> &ground_set)
    {
        for (auto &kv : grid_map_)
        {
            kv.second.is_ground = (ground_set.find(kv.second.group_id) != ground_set.end());
        }
    }

    // 计算两个平面在XY平面的最近距离
    float GroundSegmentor::compute_group_min_cell_distance(const GroundGroup &a, const GroundGroup &b) const
    {
        float min_dist_sq = std::numeric_limits<float>::max();
        for(const auto &a_idx : a.cells)        // 这一步取出索引
        {
            for(const auto &b_idx : b.cells)
            {
                auto a_iter = grid_map_.find(a_idx);        // 这一步取出迭代器
                auto b_iter = grid_map_.find(b_idx);

                if(a_iter == grid_map_.end() || b_iter == grid_map_.end())
                    continue;

                auto &a_cell = a_iter->second;           // 这一步取出 CellData
                auto &b_cell = b_iter->second;

                float dist = (a_cell.mean_x - b_cell.mean_x) * (a_cell.mean_x - b_cell.mean_x) +
                                    (a_cell.mean_y - b_cell.mean_y) * (a_cell.mean_y - b_cell.mean_y);

                if(dist < min_dist_sq)
                    min_dist_sq = dist;
            }
        }

        if (min_dist_sq == std::numeric_limits<float>::max())
            return std::numeric_limits<float>::infinity();
        return std::sqrt(min_dist_sq);
    }

    // 按 XY 距离找寻两个 ground grounp 中最近的两个格子，并输出这个最近距离（min_dist_out）
    bool GroundSegmentor::find_best_nearby_cell_pair_between_groups(
        const GroundGroup &main_group,
        const GroundGroup &other_group,
        GridIndex &main_idx_out,
        GridIndex &other_idx_out,
        float &min_dist_out) const
    {
        bool found = false;
        float min_dist_sq = std::numeric_limits<float>::max();

        for (const auto &a_idx : main_group.cells)
        {
            const auto a_it = grid_map_.find(a_idx);
            if (a_it == grid_map_.end())
                continue;

            const auto &a_cell = a_it->second;

            // gap 平面预测必须要求 main cell 有 plane
            if (!a_cell.has_plane)
                continue;

            for (const auto &b_idx : other_group.cells)
            {
                const auto b_it = grid_map_.find(b_idx);
                if (b_it == grid_map_.end())
                    continue;

                const auto &b_cell = b_it->second;

                // gap 平面预测必须要求 other cell 有 plane
                if (!b_cell.has_plane)
                    continue;

                // 计算两个格子之间的 XY 距离
                const float dx = a_cell.mean_x - b_cell.mean_x;
                const float dy = a_cell.mean_y - b_cell.mean_y;
                const float dist_sq = dx * dx + dy * dy;

                if (dist_sq < min_dist_sq)
                {
                    min_dist_sq = dist_sq;
                    main_idx_out = a_idx;
                    other_idx_out = b_idx;
                    found = true;
                }
            }
        }

        if (!found)
            return false;

        min_dist_out = std::sqrt(min_dist_sq);
        return true;
    }

    // 根据一个已知的平面方程，计算该平面在任意给定坐标 (x,y) 处的高度值
    /*
        平面方程为：
            nx*​(x−cx​) + ny*​(y−cy​) + nz*​(z−cz​) = 0
        (cx​,cy​,cz​) 是平面上已知的一点；
        (nx​,ny​,nz​) 是平面的法向量
    */
    float GroundSegmentor::predict_plane_z_at_xy(
        const Plane &plane,
        float x,
        float y) const
    {
        if (std::fabs(plane.nz) < 1e-3f)
            return std::numeric_limits<float>::quiet_NaN();

        return plane.cz -
            (plane.nx * (x - plane.cx) + plane.ny * (y - plane.cy)) / plane.nz;
    }

    // 已知 group gap 情况下两个 group 的最近 cell，判断能否 merge group
    bool GroundSegmentor::check_pair_plane_compatibility_for_gap(
        const CellData &main_cell,
        const CellData &other_cell,
        double max_predict_height_diff,
        double hard_angle_thresh) const
    {
        auto node = node_.lock();
        if (!node) 
            return false;   

        if (!main_cell.has_plane || !other_cell.has_plane)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "[GROUP GAP]: main_cell/other_cell does NOT have a plane!!!"
            );
            return false;
        }

        const float pair_angle = angle_between_planes_deg(main_cell.plane, other_cell.plane);

        // RCLCPP_INFO(
        //     node_->get_logger(),
        //     "[GROUP GAP]: Angle between gap cells: %.4f",
        //     pair_angle
        // );

        // 最近 cell 的法向角度差
        if (static_cast<double>(pair_angle) > hard_angle_thresh)
            return false;

        // 相邻平面 cell 在主平面 cell 的高度
        const float main_pred_at_other = predict_plane_z_at_xy(main_cell.plane, other_cell.mean_x, other_cell.mean_y);

        // 主平面 cell 在相邻平面 cell 的高度
        const float other_pred_at_main = predict_plane_z_at_xy(other_cell.plane, main_cell.mean_x, main_cell.mean_y);

        if (!std::isfinite(main_pred_at_other) || !std::isfinite(other_pred_at_main))
        {
            RCLCPP_WARN(
                node->get_logger(),
                "[GROUP GAP]: main_pred_at_other/other_pred_at_main is NOT finite!!!"
            );
            return false;
        }

        // 相邻平面 cell 高度差
        const float err_main_to_other = std::fabs(main_pred_at_other - other_cell.plane.cz);

        // 主平面 cell 高度差
        const float err_other_to_main = std::fabs(other_pred_at_main - main_cell.plane.cz);

        const float height_err = std::max(err_main_to_other, err_other_to_main);

        // RCLCPP_INFO(
        //     node_->get_logger(),
        //     "[GROUP GAP]: Height_err between gap cells: %.3f",
        //     height_err
        // );

        // 高度差不能过大
        /*
        这里相当于在检查边界连续性（检查缓坡），因为缓坡应该是：
            A 平面延伸到 B 的位置，高度和 B 平面一致
            B 平面延伸到 A 的位置，高度和 A 平面一致
            法向可以有小变化，但不应剧烈突变
        */
        if (static_cast<double>(height_err) > max_predict_height_diff)
            return false;

        return true;
    }

    bool GroundSegmentor::check_group_gap_compatibility(
        const GroundGroup &main_group,
        const GroundGroup &other_group) const
    {
        GridIndex main_idx, other_idx;
        float min_dist = 0.0f;

        if (!find_best_nearby_cell_pair_between_groups(
                main_group, other_group,
                main_idx, other_idx,
                min_dist))
        {
            return false;
        }

        // 允许跨过一个小间隙
        const float max_gap = gap_tolerance_;
        if (min_dist > max_gap)
            return false;

        const auto &main_cell = grid_map_.at(main_idx);
        const auto &other_cell = grid_map_.at(other_idx);

        return check_pair_plane_compatibility_for_gap(
            main_cell,
            other_cell,
            group_gap_max_predict_height_diff_,   // max_predict_height_diff
            group_merge_gap_angle_deg_);  // hard angle
    }

    // 判断一个平面能否并入主平面
    bool GroundSegmentor::should_merge_group_to_main(const GroundGroup &main_group, 
                                                                    const GroundGroup &other_group) const
    {
        // 后边的平面边界连续性检查保证了两个 ground 具有相邻的格子,不需要做距离检查了
        // float min_dist = compute_group_min_cell_distance(main_group, other_group);
        // if(static_cast<double>(min_dist) > group_merge_distance_)
        //     return false;

        float normal_angle = angle_between_planes_deg(main_group.ref_plane, other_group.ref_plane);
        if(static_cast<double>(normal_angle) > group_merge_angle_deg_)
            return false;

        float center_height = std::abs(main_group.ref_plane.cz - other_group.ref_plane.cz);
        if(static_cast<double>(center_height) > group_merge_height_diff_)
            return false;

        // if(!check_group_merge_border_continuity(main_group, other_group))
        //     return false;

        // return true;

        GridIndex main_adj, other_adj;

        // 如果真的找到两个平面的相邻格子，做严格边界检查
        if (find_best_adjacent_cell_pair_between_groups(main_group, other_group, main_adj, other_adj))
        {
            return check_border_continuity_with_thresh(
                main_adj, other_adj, grow_ground_height_diff_);
        }

        // 没有找到相邻格子，不要直接 false，走小断裂 fallback
        return check_group_gap_compatibility(main_group, other_group);
    }

    // 已知主平面的id，将其身边合格的平面合入
    std::unordered_set<int> GroundSegmentor::collect_groups_to_merge(int main_group_id) const
    {
        std::unordered_set<int> ground_set;
        ground_set.insert(main_group_id);

        const GroundGroup *main_group_ptr = nullptr;          // 一直使用指针避免对象拷贝
        for(const auto &ground : ground_groups_)
        {
            if(ground.id == main_group_id)
            {
                main_group_ptr = &ground;
                break;
            }
        }

        if(!main_group_ptr)
        {
            return  ground_set;     // 没找到直接返回，避免空指针崩溃
        }

        const GroundGroup &main_group = *main_group_ptr;           // 主平面对象

        for(const auto &ground : ground_groups_)
        {
            if(ground.id != main_group_id)
            {
                if(should_merge_group_to_main(main_group, ground))
                {
                    ground_set.insert(ground.id);
                }
            }
        }

        return ground_set;
    }

    // （通过遍历两个平面的所有格子）找到两个平面最近的、且相邻的格子
    bool GroundSegmentor::find_best_adjacent_cell_pair_between_groups(
        const GroundGroup &main_group,
        const GroundGroup &other_group,
        GridIndex &main_idx_out,
        GridIndex &other_idx_out) const
    {
        bool found = false;
        float min_dist_sq = std::numeric_limits<float>::max();

        for(const auto &cell : main_group.cells)
        {
            const auto main_iter = grid_map_.find(cell);
            if(main_iter == grid_map_.end()) 
                continue;

            const auto &main_cell = main_iter->second;          // 取出格子的内容
            const auto neighbors = get_8_neighbors(cell);
            for(const auto neigh : neighbors)
            {
                const auto neigh_iter = grid_map_.find(neigh);
                if(neigh_iter == grid_map_.end()) 
                    continue;

                const auto &neigh_cell = neigh_iter->second;        // 邻居格子内容

                if(other_group.id != neigh_cell.group_id)
                    continue;

                found = true;

                float dist_sq = (main_cell.mean_x - neigh_cell.mean_x) * (main_cell.mean_x - neigh_cell.mean_x) +
                                (main_cell.mean_y - neigh_cell.mean_y) *  (main_cell.mean_y - neigh_cell.mean_y);

                if(dist_sq < min_dist_sq)
                {
                    main_idx_out = main_iter->first;
                    other_idx_out = neigh_iter->first;
                    min_dist_sq = dist_sq;
                }
            }
        }

        return found;
    }

    // 检查两个平面的边界是否连续
    bool GroundSegmentor::check_group_merge_border_continuity(
        const GroundGroup &main_group,
        const GroundGroup &other_group) const
    {
        GridIndex main_idx, other_idx;
        if(!find_best_adjacent_cell_pair_between_groups(main_group, other_group, main_idx, other_idx))
            return false;

        return check_border_continuity_with_thresh(main_idx, other_idx, grow_ground_height_diff_);
    }

    // 计算点到平面的距离
    float GroundSegmentor::point_to_plane_distance(
        const PointXYZIConf &pt,
        const Plane &plane) const
    {
        float dx = pt.x - plane.cx;
        float dy = pt.y - plane.cy;
        float dz = pt.z - plane.cz;
        return std::fabs(dx * plane.nx + dy * plane.ny + dz * plane.nz);         // 平面法向量已经归一化过
    }

    void GroundSegmentor::classify_points_and_publish(const std::string &frame_id, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> ground_cloud;
        pcl::PointCloud<pcl::PointXYZI> non_ground_cloud;

        ground_cloud.reserve(processed_points_.size());
        non_ground_cloud.reserve(processed_points_.size());

        for (const auto &kv : grid_map_)
        {
            const auto &cell = kv.second;

            for (const int idx : cell.point_indices)
            {
                if (idx < 0 || idx >= static_cast<int>(processed_points_.size()))
                    continue;

                auto &origin_point = processed_points_[idx];

                // 置信度先默认清零
                origin_point.ground_confidence = 0.0f;

                if (!cell.is_ground)
                {
                    // 对于 mixed_cell 同样给予机会
                    if (cell.has_recovered_ground_plane)
                    {
                        const float dist = std::fabs(point_to_plane_distance(origin_point, cell.recovered_ground_plane));

                        const float T = static_cast<float>(mixed_cell_point_to_plane_thresh_);
                        const float x = std::clamp(1.0f - dist / T, 0.0f, 1.0f);
                        origin_point.ground_confidence = std::pow(x, 3.0f);         // 使用幂函数让置信度与点到平面距离为非线性关系
                    }
                    else
                    {
                        origin_point.ground_confidence = 0.0f;
                    }
                }
                else
                {
                    if (!cell.has_plane)
                    {
                        // ground cell 但没有 plane，给一个低置信度
                        origin_point.ground_confidence = 0.1f;
                    }
                    else
                    {
                        const float dist = point_to_plane_distance(origin_point, cell.plane);

                        origin_point.ground_confidence =
                                            std::max(0.0f, 1.0f - dist / static_cast<float>(point_to_plane_ground_thresh_));
                    }
                }

                pcl::PointXYZI point;
                point.x = origin_point.x;
                point.y = origin_point.y;
                point.z = origin_point.z;
                point.intensity = origin_point.ground_confidence;

                if (origin_point.ground_confidence <= static_cast<float>(confidence_thresh_))
                    non_ground_cloud.emplace_back(point);
                else
                    ground_cloud.emplace_back(point);
            }
        }

        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(ground_cloud, ground_msg);
        ground_msg.header.frame_id = frame_id_override_.empty() ? frame_id : frame_id_override_;
        ground_msg.header.stamp = msg->header.stamp;
        ground_cloud_pub_->publish(ground_msg);

        sensor_msgs::msg::PointCloud2 non_ground_msg;
        pcl::toROSMsg(non_ground_cloud, non_ground_msg);
        non_ground_msg.header.frame_id = frame_id_override_.empty() ? frame_id : frame_id_override_;
        non_ground_msg.header.stamp = msg->header.stamp;
        non_ground_cloud_pub_->publish(non_ground_msg);
    }

    // 提取低分位点索引
    std::vector<int> GroundSegmentor::extract_low_percentile_point_indices(const CellData &cell, float q) const
    {
        std::vector<std::pair<float, int>> zs_and_idx;          // 存储 z 高度和 index
        zs_and_idx.reserve(cell.point_indices.size());

        for(const int idx : cell.point_indices)
        {
            if(idx < 0 || idx >= static_cast<int>(processed_points_.size()))
                continue;
            zs_and_idx.emplace_back(processed_points_[idx].z, idx);
        }

        // 按照 z 轴高度排序
        std::sort(zs_and_idx.begin(), zs_and_idx.end(), [](const auto &a, const auto &b)
                                                            {
                                                                return a.first < b.first;
                                                            });

        q = std::clamp(q, 0.0f, 1.0f);
        size_t keep_num = std::min(static_cast<size_t>(zs_and_idx.size()), 
                                            static_cast<size_t>(std::ceil(q * zs_and_idx.size())));
        if(keep_num < static_cast<size_t>(mixed_cell_min_low_points_))
            return {};
            
        std::vector<int> output_idx;
        output_idx.reserve(keep_num);
        for (std::size_t i = 0; i < keep_num; ++i)
            output_idx.emplace_back(zs_and_idx[i].second);
        return output_idx;
    }

    // 对所有的低分位点重新拟合平面
    bool GroundSegmentor::fit_plane_from_point_indices(const std::vector<int> &indices, Plane &plane_out) const
    {
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero(); 

        float mean_x = 0.0f, mean_y = 0.0f, mean_z = 0.0f;
        for(const int idx : indices)
        {
            mean_x += processed_points_[idx].x;
            mean_y += processed_points_[idx].y;
            mean_z += processed_points_[idx].z;
        }
        mean_x /= indices.size();       // 中心
        mean_y /= indices.size();
        mean_z /= indices.size();

        const Eigen::Vector3f mean(mean_x, mean_y, mean_z);
        for(const int idx : indices)
        {
            const auto &pt = processed_points_[idx];
            const Eigen::Vector3f p(pt.x, pt.y, pt.z);
            const Eigen::Vector3f d = p - mean;
            cov += d * d.transpose();           // 协方差矩阵          
        }

        cov /= static_cast<float>(indices.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        if(solver.info() != Eigen::Success)
            return false;

        const Eigen::Vector3f eigenvalues = solver.eigenvalues();
        const Eigen::Matrix3f eigenvectors = solver.eigenvectors();

        const float ev0 = eigenvalues(0);
        const float ev1 = eigenvalues(1);
        const float ev2 = eigenvalues(2);

        Eigen::Vector3f normal = eigenvectors.col(0);
        if(normal.z() < 0.0f)
            normal = -normal;       // 保证法向朝上

        plane_out.cx = mean_x;
        plane_out.cy = mean_y;
        plane_out.cz = mean_z;

        plane_out.ev0 = ev0;
        plane_out.ev1 = ev1;
        plane_out.ev2 = ev2;

        plane_out.nx = normal.x();
        plane_out.ny = normal.y();
        plane_out.nz = normal.z();

        return true;
    }

    // 判断一个格子是否为 mixed_cell
    bool GroundSegmentor::is_mixed_cell_suspect(const GridIndex &idx) const
    {
        const auto idx_iter = grid_map_.find(idx);
        if(idx_iter == grid_map_.end())
            return false;

        const auto &cell = idx_iter->second;
        if(cell.point_indices.size() < static_cast<size_t>(min_points_per_cell_))
            return false;

        if(cell.is_ground)
            return false;

        if(cell.max_z - cell.min_z < mixed_cell_height_range_thresh_)
            return false;

        const auto neighbors = get_8_neighbors(idx);
        for(const auto idx : neighbors)
        {
            const auto iter = grid_map_.find(idx);
            if(iter == grid_map_.end())
                continue;

            const auto &cell = iter->second;
            if (cell.is_ground && cell.has_plane)       // 要求邻居也必须有平面、是地面
            {
                return true;
            }

        }
        return false;
    }

    // 对于 mixed_cell 需要找到一个相邻的地面参考邻居格子
    bool GroundSegmentor::find_best_neighbor_ground_plane(
        const GridIndex &idx,
        Plane &plane_out,
        GridIndex &ground_nbr_idx_out) const
    {
        const auto iter = grid_map_.find(idx);
        if(iter == grid_map_.end())
            return false;

        const auto &cell = iter->second;

        bool if_found = false;
        float min_dist_sq = std::numeric_limits<float>::max();
        const auto neighbors = get_8_neighbors(idx);
        for(const auto neigh_idx : neighbors)
        {
            const auto neigh_iter = grid_map_.find(neigh_idx);
            if(neigh_iter == grid_map_.end())
                continue;

            const auto &neigh_cell = neigh_iter->second;
            if(!neigh_cell.is_ground || !neigh_cell.has_plane)      // 邻居必须有平面、是地面
                continue;

            float dx = neigh_cell.mean_x - cell.mean_x;
            float dy = neigh_cell.mean_y - cell.mean_y;
            float dist_sq = dx * dx + dy * dy;

            if(dist_sq < min_dist_sq)
            {
                plane_out = neigh_cell.plane;
                ground_nbr_idx_out = neigh_idx;
                min_dist_sq = dist_sq;
                if_found = true;
            }
        }

        return if_found;
    }

    void GroundSegmentor::recover_mixed_cells_near_ground()
    {
        auto node = node_.lock();
        if (!node) 
            return ;   

        for (auto &kv : grid_map_)
        {
            kv.second.is_mixed_suspect = false;
            kv.second.has_recovered_ground_plane = false;
        }

        // 在一个指定搜索半径内，为“混合格子”寻找一个距离它最近的、合法的“地面参考平面”
        auto find_reference_plane = [&](const GridIndex &idx,
                                        bool allow_recovered_reference,     // 是否允许恢复格子
                                        int radius,
                                        Plane &plane_out,
                                        GridIndex &ref_idx_out) -> bool
        {
            const auto iter = grid_map_.find(idx);
            if (iter == grid_map_.end())
                return false;

            const auto &cell = iter->second;
            bool found = false;
            float best_dist_sq = std::numeric_limits<float>::max();

            for (int dx = -radius; dx <= radius; ++dx)
            {
                for (int dy = -radius; dy <= radius; ++dy)
                {
                    if (dx == 0 && dy == 0)
                        continue;

                    GridIndex neigh_idx{idx.band, idx.ix + dx, idx.iy + dy};
                    const auto neigh_iter = grid_map_.find(neigh_idx);
                    if (neigh_iter == grid_map_.end())
                        continue;

                    const auto &neigh_cell = neigh_iter->second;

                    bool valid = false;
                    Plane candidate_plane;
                    if (neigh_cell.is_ground && neigh_cell.has_plane)
                    {
                        candidate_plane = neigh_cell.plane;
                        valid = true;
                    }
                    else if (allow_recovered_reference && neigh_cell.has_recovered_ground_plane)
                    {
                        candidate_plane = neigh_cell.recovered_ground_plane;
                        valid = true;
                    }

                    if (!valid)
                        continue;

                    const float ddx = neigh_cell.mean_x - cell.mean_x;
                    const float ddy = neigh_cell.mean_y - cell.mean_y;
                    const float dist_sq = ddx * ddx + ddy * ddy;
                    if (dist_sq < best_dist_sq)
                    {
                        best_dist_sq = dist_sq;
                        plane_out = candidate_plane;
                        ref_idx_out = neigh_idx;
                        found = true;
                    }
                }
            }

            return found;
        };

        auto try_recover_cell = [&](const GridIndex &idx,
                                    CellData &cell,
                                    bool allow_recovered_reference,
                                    int search_radius) -> bool
        {
            if (!is_mixed_cell_suspect(idx))
                return false;

            cell.is_mixed_suspect = true;

            Plane low_plane;
            bool has_low_plane = false;

            std::vector<int> low_indices =
                extract_low_percentile_point_indices(cell, static_cast<float>(mixed_cell_low_percentile_));
            if (!low_indices.empty())
            {
                low_indices = sample_indices_evenly(low_indices, kMaxPcaSamplePoints);
                has_low_plane = fit_plane_from_point_indices(low_indices, low_plane);
            }

            // fallback: 某些 mixed cell 低分位点过少，但 fit_plane_for_cell 已经拟合出较稳定 plane。
            /*
                如果一个混合格子少部分是障碍物、大部分是平面；
                （这种格子往往因为高度差比较大，无法通过第一轮 plane-fit）
                这时候即使使用低分位拟合也不一定成功，因为低分位点数可能太少、有噪声、共线等导致拟合平面失败(第二轮 plane-fit 失败)；
                但这时候之前 plane-fit 最后的 fallback 全局拟合阶段可以救场（第三轮 plane-fit 兜底），
                因为这时候对整个格子进行拟合往往还是能拟合出平面的；
                并且这里对于 mixed-cell 平面法向偏移的容忍度比较大
            */
            if (!has_low_plane && cell.has_plane)
            {
                low_plane = cell.plane;
                has_low_plane = true;
            }

            if (!has_low_plane)
                return false;

            Plane nbr_ground_plane;
            GridIndex nbr_ground_idx;
            // 从邻居寻找参考平面
            if (!find_reference_plane(idx, allow_recovered_reference, search_radius, nbr_ground_plane, nbr_ground_idx))
                return false;

            const float d_angle = angle_between_planes_deg(low_plane, nbr_ground_plane);
            if (d_angle > static_cast<float>(mixed_cell_recovered_max_angle_deg_))      // 法向量朝向不能差太多
                return false;

            const float d_height = std::fabs(low_plane.cz - nbr_ground_plane.cz);
            if (d_height > static_cast<float>(mixed_cell_recovered_height_diff_))       // 高度不能差太多
                return false;

            cell.has_recovered_ground_plane = true;
            cell.recovered_ground_plane = low_plane;
            return true;
        };

        std::size_t suspect_count = 0;
        std::size_t strict_recovered = 0;
        std::size_t wide_recovered = 0;
        std::size_t propagated_recovered = 0;

        // Pass-1: 严格恢复，只依赖 8 邻域内真实 ground plane。
        for (auto &kv : grid_map_)
        {
            const GridIndex idx = kv.first;
            auto &cell = kv.second;
            if (is_mixed_cell_suspect(idx))     // 是 mixed-cell
            {
                ++suspect_count;
                if (try_recover_cell(idx, cell, false, 1))
                    ++strict_recovered;
            }
        }

        // Pass-2: 宽邻域恢复，解决地面 skeleton 有小断裂的情况。
        for (auto &kv : grid_map_)
        {
            const GridIndex idx = kv.first;
            auto &cell = kv.second;
            if (cell.has_recovered_ground_plane)
                continue;
            if (try_recover_cell(idx, cell, false, 2))
                ++wide_recovered;
        }

        // Pass-3: 传播式恢复，允许引用上一轮刚恢复出来的 mixed ground plane。
        bool if_progress = true;
        while (if_progress)
        {
            if_progress = false;
            for (auto &kv : grid_map_)
            {
                const GridIndex idx = kv.first;
                auto &cell = kv.second;
                if (cell.has_recovered_ground_plane)
                    continue;
                if (try_recover_cell(idx, cell, true, 2))
                {
                    ++propagated_recovered;
                    if_progress = true;
                }
            }
        }

        if (!if_silence_)
        {
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "[MixedRecovery] suspect=%zu, strict=%zu, wide=%zu, propagated=%zu",
                suspect_count, strict_recovered, wide_recovered, propagated_recovered);
        }
    }

    bool GroundSegmentor::update_frame_robot_pose_snapshot()
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);

        if (!has_odom_)
            return false;

        frame_robot_x_ = robot_x_;
        frame_robot_y_ = robot_y_;
        frame_robot_z_ = robot_z_;
        has_frame_pose_ = true;

        return true;
    }

    void GroundSegmentor::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!active_.load()) 
            return;
    
        auto node = node_.lock();
        if (!node) 
            return ;   

        if (input_pointcloud_frame_ == "map")
        {
            if (!update_frame_robot_pose_snapshot())
            {
                RCLCPP_WARN_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    1000,
                    "Waiting for odometry before segmenting map-frame pointcloud.");
                return;
            }
        }
        else
        {
            has_frame_pose_ = false;
        }

        const auto start_time = std::chrono::steady_clock::now();
        // ============================== 基础过滤 ==============================
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->empty())
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "PointCloud from /points_cropped is EMPTY."
                );
            return;
        }

        // 去除 NaN/Inf 点
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_nan_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*input_cloud, *no_nan_cloud, indices);     // 不保证去除 Inf 点，后边会遍历手动去除

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        filtered_cloud->reserve(no_nan_cloud->size());

        const float min_range_sq = static_cast<float>(min_range_ * min_range_);
        const float max_range_sq = static_cast<float>(max_range_ * max_range_);
        const float min_z = static_cast<float>(min_z_);
        const float max_z = static_cast<float>(max_z_);

        for (auto &pt : no_nan_cloud->points)
        {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;

            float r2 = 0.0;

            if(input_pointcloud_frame_ == "map")
            {
                const float dx = pt.x - frame_robot_x_;
                const float dy = pt.y - frame_robot_y_;

                r2 = dx * dx + dy * dy;
            }
            else if(input_pointcloud_frame_ == "body")
            {
                r2 = pt.x * pt.x + pt.y * pt.y;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    1000,
                    "input_pointcloud_frame_ NOT valid, expected value is map/body !!!"
                );
                r2 = pt.x * pt.x + pt.y * pt.y;
            }
            
            if (r2 > max_range_sq || r2 < min_range_sq)
                continue;

            // 把 Z 裁剪改为相对高度
            const float z_rel = pt.z - static_cast<float>(frame_robot_z_);
            if (z_rel < min_z || z_rel > max_z) continue;

            filtered_cloud->push_back(pt);
        }

        filtered_cloud->width = static_cast<uint32_t>(filtered_cloud->points.size());
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;

        if (filtered_cloud->empty())
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "All points filtered out during early stage of Ground-Segment!!!");
            return;
        }

        // 体素下采样
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if(enable_voxel_downsample_)
        {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(filtered_cloud);
            voxel_filter.setLeafSize(
                static_cast<float>(voxel_leaf_size_),
                static_cast<float>(voxel_leaf_size_),
                static_cast<float>(voxel_leaf_size_));
            voxel_filter.filter(*downsampled_cloud);
        }
        else
        {
            downsampled_cloud = filtered_cloud->makeShared();
        }

        // 转为自定义结构体
        processed_points_.clear();
        processed_points_.reserve(downsampled_cloud->size());

        for (auto &pt : downsampled_cloud->points)
        {
            PointXYZIConf out_pt;
            out_pt.x = pt.x;
            out_pt.y = pt.y;
            out_pt.z = pt.z;
            out_pt.intensity = pt.intensity;
            out_pt.ground_confidence = 0.0f;
            processed_points_.push_back(out_pt);
        }

        const auto time0 = std::chrono::steady_clock::now();
        double cost_ms = std::chrono::duration<double, std::milli>(time0 - start_time).count();

        if (!if_silence_)
        {
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1000,
                "Ground-Segment Input Cloud: %zu, after NaN/Inf+range+z: %zu, after voxel: %zu",
                input_cloud->size(),
                filtered_cloud->size(),
                processed_points_.size());
        }

        if(if_debug_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Pre-process cost: %.2f ms",
                cost_ms
            );
        }

        // ============================== 2.5D 网格划分 ==============================
        build_grid_map();

        if (if_show_grid_map_)
        {
            if (grid_map_.empty())
                return;

            pcl::PointCloud<pcl::PointXYZI> grid_map_cloud;
            grid_map_cloud.reserve(grid_map_.size());

            for (const auto &kv : grid_map_)
            {
                const auto &cell = kv.second;
                if (cell.point_indices.empty())
                    continue;

                pcl::PointXYZI point;
                point.x = cell.mean_x;
                point.y = cell.mean_y;
                point.z = cell.mean_z;
                point.intensity = static_cast<float>(cell.band + 1); // 1/2/3 表示 band
                grid_map_cloud.emplace_back(point);
            }

            sensor_msgs::msg::PointCloud2::SharedPtr grid_map_msg(new sensor_msgs::msg::PointCloud2());
            pcl::toROSMsg(grid_map_cloud, *grid_map_msg);
            grid_map_msg->header.stamp = msg->header.stamp;
            grid_map_msg->header.frame_id = msg->header.frame_id;
            grid_map_pub_->publish(*grid_map_msg);
        }

        const auto time1 = std::chrono::steady_clock::now();
        cost_ms = std::chrono::duration<double, std::milli>(time1 - time0).count();

        if(if_debug_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "2.5D Map-Building cost: %.2f ms",
                cost_ms
            );
        }

        // ============================== PlaneFit ==============================
        fit_planes_for_all_cells();

        if (if_show_plane_cells_)
        {
            pcl::PointCloud<pcl::PointXYZI> plane_cells_cloud;
            plane_cells_cloud.reserve(grid_map_.size());

            for (const auto &kv : grid_map_)
            {
                const auto &cell = kv.second;
                if (!cell.has_plane)
                    continue;

                pcl::PointXYZI pt;
                pt.x = cell.plane.cx;
                pt.y = cell.plane.cy;
                pt.z = cell.plane.cz;
                pt.intensity = cell.plane.nz;
                plane_cells_cloud.emplace_back(pt);
            }

            sensor_msgs::msg::PointCloud2 plane_cells_msg;
            pcl::toROSMsg(plane_cells_cloud, plane_cells_msg);
            plane_cells_msg.header.stamp = msg->header.stamp;
            plane_cells_msg.header.frame_id = msg->header.frame_id;
            plane_fit_pub_->publish(plane_cells_msg);
        }

        const auto time2 = std::chrono::steady_clock::now();
        cost_ms = std::chrono::duration<double, std::milli>(time2 - time1).count();

        if(if_debug_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "PlaneFit cost: %.2f ms",
                cost_ms
            );
        }

        // ============================== Ground Seed 生成 ==============================
        generate_ground_seeds();

        if (if_show_seed_cells_)
        {
            pcl::PointCloud<pcl::PointXYZI> seed_cells_cloud;
            seed_cells_cloud.reserve(grid_map_.size());

            for (const auto &kv : grid_map_)
            {
                const auto &cell = kv.second;
                if (!cell.is_seed)
                    continue;

                pcl::PointXYZI pt;
                pt.x = cell.plane.cx;
                pt.y = cell.plane.cy;
                pt.z = cell.plane.cz;
                pt.intensity = 1.0f;
                seed_cells_cloud.emplace_back(pt);
            }

            sensor_msgs::msg::PointCloud2 seed_cells_msg;
            pcl::toROSMsg(seed_cells_cloud, seed_cells_msg);
            seed_cells_msg.header.stamp = msg->header.stamp;
            seed_cells_msg.header.frame_id = msg->header.frame_id;
            seed_cells_pub_->publish(seed_cells_msg);
        }

        const auto time3 = std::chrono::steady_clock::now();
        cost_ms = std::chrono::duration<double, std::milli>(time3 - time2).count();

        if(if_debug_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Ground Seed Generating cost: %.2f ms",
                cost_ms
            );
        }

        // ============================== Seed Growing && Merging Groups ==============================
        grow_ground_groups();

        const int main_group_id = select_main_ground_group();
        if (main_group_id >= 0)
        {
            const std::unordered_set<int> ground_set = collect_groups_to_merge(main_group_id);
            mark_selected_ground_groups(ground_set);
        }

        if (if_show_ground_groups_)
        {
            pcl::PointCloud<pcl::PointXYZI> ground_cell_cloud;
            ground_cell_cloud.reserve(grid_map_.size());

            for (const auto &kv : grid_map_)
            {
                const auto &cell = kv.second;
                if (!cell.is_ground || !cell.has_plane)
                    continue;

                pcl::PointXYZI pt;
                pt.x = cell.plane.cx;
                pt.y = cell.plane.cy;
                pt.z = cell.plane.cz;
                pt.intensity = 1.0f;
                ground_cell_cloud.emplace_back(pt);
            }

            sensor_msgs::msg::PointCloud2 ground_cell_msg;
            pcl::toROSMsg(ground_cell_cloud, ground_cell_msg);
            ground_cell_msg.header.stamp = msg->header.stamp;
            ground_cell_msg.header.frame_id = msg->header.frame_id;
            ground_group_pub_->publish(ground_cell_msg);
        }

        const auto time4 = std::chrono::steady_clock::now();
        cost_ms = std::chrono::duration<double, std::milli>(time4 - time3).count();

        if(if_debug_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Seed Growing && Merging Groups cost: %.2f ms",
                cost_ms
            );
        }

        // ============================== Mixed-cell Recovery ==============================
        recover_mixed_cells_near_ground();

        const auto time5 = std::chrono::steady_clock::now();
        cost_ms = std::chrono::duration<double, std::milli>(time5 - time4).count();

        if(if_debug_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Mixed-cell Recovery cost: %.2f ms",
                cost_ms
            );
        }

        // publish ground-points/non-grounds
        classify_points_and_publish(msg->header.frame_id, msg);

        const auto time6 = std::chrono::steady_clock::now();
        cost_ms = std::chrono::duration<double, std::milli>(time6 - time5).count();
        const double cost_ms_total = std::chrono::duration<double, std::milli>(time6 - start_time).count();

        if(if_debug_)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Classify && Publish cost: %.2f ms",
                cost_ms
            );

            RCLCPP_INFO(
                node->get_logger(),
                "Total time-cost: %.2f",
                cost_ms_total
            );
        }
    }

}  // namespace ground_segmentor

PLUGINLIB_EXPORT_CLASS(ground_segmentor::GroundSegmentor, ground_segmentor::GroundSegmentorBase);
/*
数据结构介绍：
PointXYZI：
struct PointXYZI {
    float x;  // X坐标
    float y;  // Y坐标
    float z;  // Z坐标
    float intensity; // 强度值（如激光雷达反射强度）
};

pcl::PointCloud<PointT> ———— 点云容器：
成员	     类型	                    描述
points	    std::vector<PointT>	    存储所有点的动态数组，是实际数据容器
width	    uint32_t	            对于无序点云，表示点云中点的总数；对于有序点云（如深度相机获取的图像格式点云），表示一行的点数
height	    uint32_t	            对于无序点云，值为1；对于有序点云，表示点云的总行数
is_dense	bool	                指示点云中的数据是否全部有效（不包含 Inf 或 NaN 等无效值）
header	    pcl::PCLHeader	        存储点云的元数据，如时间戳和坐标系名称

*/