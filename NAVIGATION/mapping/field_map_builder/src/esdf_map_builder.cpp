#include "field_map_builder/esdf_map_builder.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace field_map_builder
{
    namespace
    {
        constexpr float kEdtInfinity = 1.0e20f;
    }

    void EsdfMapBuilding::configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name)
    {
        node_ = parent;
        plugin_name_ = plugin_name;

        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        reset_runtime_state();

        load_parameters(node);

        create_publishers(node);

        RCLCPP_INFO(
            node->get_logger(),
            "EsdfMapBuilding plugin '%s' configured.",
            plugin_name_.c_str()
        );
    }

    void EsdfMapBuilding::load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const std::string p = plugin_name_ + ".";

        if_silence_ = declare_or_get_parameter<bool>(node, p + "if_silence", false);
        if_debug_ = declare_or_get_parameter<bool>(node, p + "if_debug", true);

        input_costmap_topic_ = declare_or_get_parameter<std::string>(node, p + "input_costmap_topic", "/occupancy_static_map");
        esdf_debug_topic_ = declare_or_get_parameter<std::string>(node, p + "esdf_debug_topic", "/esdf_debug");
        esdf_topic_ = declare_or_get_parameter<std::string>(node, p + "esdf_topic", "/esdf_map");
        unknown_as_obstacle_ = declare_or_get_parameter<bool>(node, p + "unknown_as_obstacle", true);
        publish_debug_esdf_ = declare_or_get_parameter<bool>(node, p + "publish_debug_esdf", true);
        max_esdf_distance_ = declare_or_get_parameter<float>(node, p + "max_esdf_distance", 10.0);

        RCLCPP_INFO(
            node->get_logger(),
            "Params-loading accomplished!!!"
        );
    }

    void EsdfMapBuilding::create_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const auto esdf_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        esdf_debug_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
            esdf_debug_topic_,
            esdf_qos
        );

        esdf_pub_ = node->create_publisher<trailblazer_map_interfaces::msg::EsdfMap>(
            esdf_topic_,
            esdf_qos
        );
    }

    void EsdfMapBuilding::reset_runtime_state()
    {
        lifecycle_active_.store(false);
        map_initialized_ = false;

        grid_cols_ = 0;
        grid_rows_ = 0;
        resolution_ = 0.0;
        origin_x_ = 0.0;
        origin_y_ = 0.0;
        frame_id_.clear();

        obstacle_mask_.clear();
        edt_tmp_.clear();
        edt_sq_dist_.clear();
        edt_line_input_.clear();
        edt_line_output_.clear();
        edt_parabola_indices_.clear();
        edt_parabola_boundaries_.clear();

        esdf_debug_map_msg_ = nav_msgs::msg::OccupancyGrid{};
        esdf_map_msg_ = trailblazer_map_interfaces::msg::EsdfMap{};
    }

    void EsdfMapBuilding::activate()
    {
        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        if (esdf_debug_pub_)
            esdf_debug_pub_->on_activate();

        if (esdf_pub_)
            esdf_pub_->on_activate();

        lifecycle_active_.store(true);

        create_subscriptions(node);
        // create_timers(node);
        // create_services(node);

        RCLCPP_INFO(
            node->get_logger(),
            "EsdfMapBuilding plugin '%s' activated.",
            plugin_name_.c_str());
    }

    void EsdfMapBuilding::create_subscriptions(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
                    input_costmap_topic_,
                    map_qos,
                    std::bind(&EsdfMapBuilding::final_costmap_callback, this, std::placeholders::_1)
                );
    }

    void EsdfMapBuilding::final_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if(!msg || !lifecycle_active_)
            return;

        if (msg->info.width == 0 || msg->info.height == 0)
            return;

        if (msg->data.size() != static_cast<std::size_t>(msg->info.width * msg->info.height))
            return;

        // 检查并更新地图元数据
        if(!map_initialized_ || map_meta_changed(*msg))
        {
            update_map_meta_from_msg(*msg);
            resize_buffers_if_needed();
            reset_esdf_debug_map_msg();
            reset_esdf_map_msg();
            map_initialized_ = true;
        }

        build_obstacle_mask_from_occupancy(*msg);
        build_signed_esdf();

        if (publish_debug_esdf_)
        {
            publish_esdf_debug_map(msg->header.stamp);
        }

        publish_esdf_map(msg->header.stamp);
    }

    bool EsdfMapBuilding::map_meta_changed(const nav_msgs::msg::OccupancyGrid & msg) const
    {
        return (
            static_cast<int>(msg.info.width) != grid_cols_ ||
            static_cast<int>(msg.info.height) != grid_rows_ ||
            std::abs(msg.info.resolution - resolution_) > 1e-6 ||
            std::abs(msg.info.origin.position.x - origin_x_) > 1e-6 ||
            std::abs(msg.info.origin.position.y - origin_y_) > 1e-6 ||
            msg.header.frame_id != frame_id_
        );
    }

    void EsdfMapBuilding::update_map_meta_from_msg(const nav_msgs::msg::OccupancyGrid & msg)
    {
        grid_cols_ = msg.info.width;
        grid_rows_ = msg.info.height;
        resolution_ = msg.info.resolution;
        origin_x_ = msg.info.origin.position.x;
        origin_y_ = msg.info.origin.position.y;
        frame_id_ = msg.header.frame_id;
    }

    void EsdfMapBuilding::resize_buffers_if_needed()
    {
        const std::size_t total_size = static_cast<std::size_t>(grid_cols_ * grid_rows_);
       
        obstacle_mask_.assign(total_size, 0);
        edt_tmp_.assign(total_size, 0.0f);
        edt_sq_dist_.assign(total_size, 0.0f);

        // 行变换需要 grid_cols_，列变换需要 grid_rows_，
        // 因此统一按照较大的长度分配。
        const std::size_t max_line_size = static_cast<std::size_t>(std::max(grid_cols_, grid_rows_));
        edt_line_input_.assign(max_line_size, 0.0f);
        edt_line_output_.assign(max_line_size, 0.0f);
        edt_parabola_indices_.assign(max_line_size, 0);
        edt_parabola_boundaries_.assign(max_line_size + 1U, 0.0f);
    }

    void EsdfMapBuilding::reset_esdf_debug_map_msg()
    {
        esdf_debug_map_msg_.header.frame_id = frame_id_;
        esdf_debug_map_msg_.info.resolution = static_cast<float>(resolution_);
        esdf_debug_map_msg_.info.width = static_cast<uint32_t>(grid_cols_);
        esdf_debug_map_msg_.info.height = static_cast<uint32_t>(grid_rows_);
        esdf_debug_map_msg_.info.origin.position.x = origin_x_;
        esdf_debug_map_msg_.info.origin.position.y = origin_y_;
        esdf_debug_map_msg_.info.origin.position.z = 0.0;
        esdf_debug_map_msg_.info.origin.orientation.x = 0.0;
        esdf_debug_map_msg_.info.origin.orientation.y = 0.0;
        esdf_debug_map_msg_.info.origin.orientation.z = 0.0;
        esdf_debug_map_msg_.info.origin.orientation.w = 1.0;
        esdf_debug_map_msg_.data.assign(static_cast<std::size_t>(grid_cols_ * grid_rows_), 100); 
    }

    void EsdfMapBuilding::reset_esdf_map_msg()
    {
        esdf_map_msg_.header.frame_id = frame_id_;
        esdf_map_msg_.info.resolution = resolution_;
        esdf_map_msg_.info.width = static_cast<uint32_t>(grid_cols_);
        esdf_map_msg_.info.height = static_cast<uint32_t>(grid_rows_);
        esdf_map_msg_.info.origin.position.x = origin_x_;
        esdf_map_msg_.info.origin.position.y = origin_y_;
        esdf_map_msg_.info.origin.position.z = 0.0;
        esdf_map_msg_.info.origin.orientation.x = 0.0;
        esdf_map_msg_.info.origin.orientation.y = 0.0;
        esdf_map_msg_.info.origin.orientation.z = 0.0;
        esdf_map_msg_.info.origin.orientation.w = 1.0;
        esdf_map_msg_.max_distance = max_esdf_distance_;
        esdf_map_msg_.unknown_as_obstacle = unknown_as_obstacle_;
        esdf_map_msg_.distances.assign(static_cast<std::size_t>(grid_cols_ * grid_rows_), 0.0f); 
    }

    void EsdfMapBuilding::build_obstacle_mask_from_occupancy(const nav_msgs::msg::OccupancyGrid & msg)
    {
        for (std::size_t i = 0; i < msg.data.size(); ++i)
        {
            const int8_t occupancy = msg.data[i];

            if (occupancy == 100)
            {
                obstacle_mask_[i] = 1U;
            }
            else if (occupancy == 0)
            {
                obstacle_mask_[i] = 0U;
            }
            else  
            {
                obstacle_mask_[i] = unknown_as_obstacle_ ? 1U : 0U;
            }
        }
    }

    void EsdfMapBuilding::build_signed_esdf()
    {
        auto & signed_distances = esdf_map_msg_.distances;

        // 第一次 EDT：所有 obstacle cell 是种子，得到自由空间到最近障碍的距离。
        initialize_sq_distance_grid(1U);
        compute_edt_2d();
        for (std::size_t i = 0; i < signed_distances.size(); ++i)
        {
            signed_distances[i] = squared_cells_to_metric_distance(edt_sq_dist_[i]);
        }

        // 第二次 EDT：所有 free cell 是种子，得到障碍内部到最近自由空间的距离。
        initialize_sq_distance_grid(0U);
        compute_edt_2d();
        for (std::size_t i = 0; i < signed_distances.size(); ++i)
        {
            if (obstacle_mask_[i] != 0U)
            {
                signed_distances[i] = -squared_cells_to_metric_distance(edt_sq_dist_[i]);
            }
        }
    }

    void EsdfMapBuilding::initialize_sq_distance_grid(uint8_t seed_value)
    {
        for (std::size_t i = 0; i < obstacle_mask_.size(); ++i)
        {
            edt_sq_dist_[i] = obstacle_mask_[i] == seed_value ? 0.0f : kEdtInfinity;
        }
    }

    void EsdfMapBuilding::compute_edt_2d()
    {
        compute_row_edt();
        compute_column_edt();
    }

    void EsdfMapBuilding::compute_row_edt()
    {
        for (int row = 0; row < grid_rows_; ++row)
        {
            const std::size_t start = static_cast<std::size_t>(row) * static_cast<std::size_t>(grid_cols_);

            for (int col = 0; col < grid_cols_; ++col)
            {
                edt_line_input_[static_cast<std::size_t>(col)] = edt_sq_dist_[start + static_cast<std::size_t>(col)];
            }

            distance_transform_1d(grid_cols_);

            for (int col = 0; col < grid_cols_; ++col)
            {
                edt_tmp_[start + static_cast<std::size_t>(col)] = edt_line_output_[static_cast<std::size_t>(col)];
            }
        }
    }

    void EsdfMapBuilding::compute_column_edt()
    {
        for (int col = 0; col < grid_cols_; ++col)
        {
            for (int row = 0; row < grid_rows_; ++row)
            {
                const std::size_t index =
                    static_cast<std::size_t>(row) * static_cast<std::size_t>(grid_cols_) +
                    static_cast<std::size_t>(col);
                edt_line_input_[static_cast<std::size_t>(row)] = edt_tmp_[index];
            }

            distance_transform_1d(grid_rows_);

            for (int row = 0; row < grid_rows_; ++row)
            {
                const std::size_t index =
                    static_cast<std::size_t>(row) * static_cast<std::size_t>(grid_cols_) +
                    static_cast<std::size_t>(col);
                edt_sq_dist_[index] = edt_line_output_[static_cast<std::size_t>(row)];
            }
        }
    }

    void EsdfMapBuilding::distance_transform_1d(int n)
    {
        if (n <= 0)
        {
            return;
        }

        int k = 0;
        edt_parabola_indices_[0] = 0;
        edt_parabola_boundaries_[0] = -std::numeric_limits<float>::infinity();      // 这里设为负无穷保证了每次 k 掉到 0 时，再计算出任何数都会被保留、不至于掉到 0 以下
        edt_parabola_boundaries_[1] = std::numeric_limits<float>::infinity();

        for (int q = 1; q < n; ++q)
        {
            bool inserted = false;

            while (k >= 0)
            {
                const int vertex = edt_parabola_indices_[static_cast<std::size_t>(k)];          // 当前抛物线中心对应的索引 i
                const float q_float = static_cast<float>(q);                // q 代表候选的、新抛物线的中心对应的索引 i
                const float vertex_float = static_cast<float>(vertex);
                // 求交点（见 /image 下图片）
                const float numerator =                     // 分子
                    (edt_line_input_[static_cast<std::size_t>(q)] + q_float * q_float) -
                    (edt_line_input_[static_cast<std::size_t>(vertex)] +
                    vertex_float * vertex_float);
                const float denominator = 2.0f * (q_float - vertex_float);
                const float intersection = numerator / denominator;         // 这个交点表示，从哪个位置开始，新抛物线会低于旧抛物线

                // 每次将交点与当前的右边界来相比
                // 交点更靠右，证明当前索引（k）对应的抛物线是有效的（有一部分是下包络线）
                if (intersection > edt_parabola_boundaries_[static_cast<std::size_t>(k)])
                {
                    ++k;
                    edt_parabola_indices_[static_cast<std::size_t>(k)] = q;
                    edt_parabola_boundaries_[static_cast<std::size_t>(k)] = intersection;
                    edt_parabola_boundaries_[static_cast<std::size_t>(k + 1)] = std::numeric_limits<float>::infinity();
                    inserted = true;
                    break;
                }

                --k;
            }

            // 使用有限的大数代替 +inf 后通常不会进入该分支；保留它避免异常数值
            // 让抛物线包络处于无效状态。
            if (!inserted)
            {
                k = 0;
                edt_parabola_indices_[0] = q;
                edt_parabola_boundaries_[0] = -std::numeric_limits<float>::infinity();
                edt_parabola_boundaries_[1] = std::numeric_limits<float>::infinity();
            }
        }

        k = 0;
        for (int q = 0; q < n; ++q)
        {
            while (edt_parabola_boundaries_[static_cast<std::size_t>(k + 1)] < static_cast<float>(q))
            {
                ++k;
            }

            const int vertex = edt_parabola_indices_[static_cast<std::size_t>(k)];
            const float delta = static_cast<float>(q - vertex);
            edt_line_output_[static_cast<std::size_t>(q)] = delta * delta + edt_line_input_[static_cast<std::size_t>(vertex)];
        }
    }

    float EsdfMapBuilding::squared_cells_to_metric_distance(float squared_cells) const
    {
        if (squared_cells >= kEdtInfinity * 0.5f)
        {
            return max_esdf_distance_;
        }

        const float non_negative_squared = std::max(0.0f, squared_cells);
        const float distance =
            std::sqrt(non_negative_squared) * static_cast<float>(resolution_);
        return std::min(distance, max_esdf_distance_);
    }

    void EsdfMapBuilding::publish_esdf_debug_map(const builtin_interfaces::msg::Time & stamp)
    {
        const auto & esdf_distance = esdf_map_msg_.distances;

        if (esdf_debug_map_msg_.data.size() != esdf_distance.size())
        {
            esdf_debug_map_msg_.data.assign(esdf_distance.size(), 0);
        }

        esdf_debug_map_msg_.header.stamp = stamp;
        esdf_debug_map_msg_.info.map_load_time = stamp;

        for(size_t i = 0; i < esdf_distance.size(); i++)
        {
            float dist = std::min(esdf_distance[i], static_cast<float>(max_esdf_distance_));

            int8_t value;
            if(dist >= 0)           // 障碍物外部
            {
                    // 将距离映射到[0, 100]，也就是距离为 0 -> 100，距离为 max_esdf_distance_ -> 0
                value = static_cast<int8_t>(
                    std::round(100.0f * (1.0f - dist / static_cast<float>(max_esdf_distance_)))
                );
            }
            else                    // 障碍物内部
            {
                // 将负数部分映射为 -2～-128，在 rviz2 中可以显示成红色到黄色渐变
                value = static_cast<int8_t>(
                    std::round(-2 - dist * (-1.0f) / static_cast<float>(max_esdf_distance_) * 126.0f)
                );
            }

            esdf_debug_map_msg_.data[i] = value;
        }
        esdf_debug_pub_->publish(esdf_debug_map_msg_);
    }

    void EsdfMapBuilding::publish_esdf_map(const builtin_interfaces::msg::Time & stamp)
    {
        esdf_map_msg_.header.stamp = stamp;
        esdf_map_msg_.header.frame_id = frame_id_;
        esdf_map_msg_.info.map_load_time = stamp;

        esdf_map_msg_.info.resolution = resolution_;
        esdf_map_msg_.info.width = static_cast<uint32_t>(grid_cols_);
        esdf_map_msg_.info.height = static_cast<uint32_t>(grid_rows_);
        esdf_map_msg_.info.origin.position.x = origin_x_;
        esdf_map_msg_.info.origin.position.y = origin_y_;

        esdf_map_msg_.max_distance = static_cast<float>(max_esdf_distance_);
        esdf_map_msg_.unknown_as_obstacle = unknown_as_obstacle_;

        esdf_pub_->publish(esdf_map_msg_);
    }

    void EsdfMapBuilding::deactivate()
    {
        auto node = node_.lock();

        // 挡住所有 callback 先
        lifecycle_active_.store(false);

        // reset subs
        costmap_sub_.reset();

        // deactivate publishers
        esdf_debug_pub_->on_deactivate();
        esdf_pub_->on_deactivate();

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "EsdfMapBuilding plugin '%s' deactivated.",
                plugin_name_.c_str());
        }
    }

    void EsdfMapBuilding::cleanup()
    {
        auto node = node_.lock();

        lifecycle_active_.store(false);

        // 幂等保护
        costmap_sub_.reset();

        // reset publishers（configure 阶段创建）
        esdf_debug_pub_.reset();
        esdf_pub_.reset();

        reset_runtime_state();

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "EsdfMapBuilding plugin '%s' cleaned up.",
                plugin_name_.c_str());
        }

        plugin_name_.clear();

        node_.reset();
    }

}       // namespace field_map_builder

PLUGINLIB_EXPORT_CLASS(field_map_builder::EsdfMapBuilding, field_map_builder::FieldMapBuilderBase)
