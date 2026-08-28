#include "occupancy_mapping/ordinary_occupancy_mapping.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <sstream>

namespace occupancy_mapping
{
    void OrdinaryOccupancyMapping::configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name)
    {
        node_ = parent;
        plugin_name_ = plugin_name;

        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        load_parameters(node);

        map_io_ = std::make_unique<StaticMapIO>(map_directory_);

        create_publishers(node);

        reset_runtime_state();

        // 在 configure 加载到内存即可，不需要 publisher active。
        // try loading map
        std::string load_message;
        const auto result = tryLoadDefaultMap(load_message);

        if (result == DefaultMapLoadResult::LOADED) 
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Default static map loaded during configure: %s",
                load_message.c_str());
        } 
        else if (result == DefaultMapLoadResult::NOT_FOUND) 
        {
            RCLCPP_INFO(
                node->get_logger(),
                "No default static map found. Server will enter NO_MAP: %s",
                load_message.c_str());
        } 
        else 
        {
            // 此处是加载地图失败，并非节点错误
            // 可通过 /clear_static_map 来清除错误、进入 NO_MAP 状态
            RCLCPP_ERROR(
                node->get_logger(),
                "Default static map is invalid. Server will enter ERROR: %s",
                load_message.c_str());
        }

        RCLCPP_INFO(
            node->get_logger(),
            "OrdinaryOccupancyMapping plugin '%s' configured.",
            plugin_name_.c_str()
        );
    }

    void OrdinaryOccupancyMapping::load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const std::string p = plugin_name_ + ".";

        if_silence_ = declare_or_get_parameter<bool>(node, p + "if_silence", false);
        if_debug_ = declare_or_get_parameter<bool>(node, p + "if_debug", true);

        input_odom_topic_ = declare_or_get_parameter<std::string>(node, p + "input_odom_topic", "/Odometry");
        ground_topic_ = declare_or_get_parameter<std::string>(node, p + "ground_topic", "/ground_points");
        non_ground_topic_ = declare_or_get_parameter<std::string>(node, p + "non_ground_topic", "/non_ground_points");
        submap_cloud_topic_ = declare_or_get_parameter<std::string>(node, p + "submap_cloud_topic", "/submap_points_cropped");
        submap_output_topic_ = declare_or_get_parameter<std::string>(node, p + "submap_output_topic", "/occupancy_submap");
        map_status_topic_ = declare_or_get_parameter<std::string>(node, p + "map_status_topic", "/map_status");
        static_map_topic_ = declare_or_get_parameter<std::string>(node, p + "static_map_topic", "/occupancy_static_map");
        inflation_map_topic_ = declare_or_get_parameter<std::string>(node, p + "inflation_map_topic", "/inflation_occupancy_map");

        start_mapping_service_ = declare_or_get_parameter<std::string>(node, p + "start_mapping_service", "/start_static_mapping");
        stop_mapping_service_ = declare_or_get_parameter<std::string>(node, p + "stop_mapping_service", "/stop_static_mapping");
        error_clearing_service_ = declare_or_get_parameter<std::string>(node, p + "error_clearing_service", "/error_clearing");
        start_incremental_service_ = declare_or_get_parameter<std::string>(node, p + "start_incremental_service", "/start_incremental");

        frame_id_ = declare_or_get_parameter<std::string>(node, p + "frame_id", "map");

        map_publish_frequency_ = declare_or_get_parameter<int>(node, p + "map_publish_frequency", 1);

        initial_width_ = declare_or_get_parameter<double>(node, p + "initial_width", 10.0);
        initial_height_ = declare_or_get_parameter<double>(node, p + "initial_height", 10.0);
        resolution_ = declare_or_get_parameter<double>(node, p + "resolution", 0.05);

        cloud_sync_tolerance_ = declare_or_get_parameter<double>(node, p + "cloud_sync_tolerance", 0.2);

        expand_margin_ = declare_or_get_parameter<double>(node, p + "expand_margin", 2.0);

        if_strict_ = declare_or_get_parameter<bool>(node, p + "if_strict", false);

        min_ground_points_per_cell_ = declare_or_get_parameter<int>(node, p + "min_ground_points_per_cell", 4);
        ground_percentile_ = declare_or_get_parameter<double>(node, p + "ground_percentile", 0.4);
        
        min_obstacle_height_ = declare_or_get_parameter<double>(node, p + "min_obstacle_height", 0.02);
        strong_obstacle_height_ = declare_or_get_parameter<double>(node, p + "strong_obstacle_height", 0.12);

        min_non_ground_count_ = declare_or_get_parameter<int>(node, p + "min_non_ground_count", 4);
        min_mean_height_ = declare_or_get_parameter<double>(node, p + "min_mean_height", 0.04);
        min_strong_count_ = declare_or_get_parameter<int>(node, p + "min_strong_count", 2);

        enable_ground_raycast_ = declare_or_get_parameter<bool>(node, p + "enable_ground_raycast", false);
        enable_non_ground_raycast_ = declare_or_get_parameter<bool>(node, p + "enable_non_ground_raycast", true);

        raycast_point_step_ = declare_or_get_parameter<int>(node, p + "raycast_point_step", 1);
        raycast_min_range_ = declare_or_get_parameter<double>(node, p + "raycast_min_range", 0.15);
        raycast_max_range_ = declare_or_get_parameter<double>(node, p + "raycast_max_range", 20.0);

        raycast_endpoint_margin_cells_ = declare_or_get_parameter<int>(node, p + "raycast_endpoint_margin_cells", 0);
        hit_logodds_ = declare_or_get_parameter<double>(node, p + "hit_logodds", 1.0);
        miss_logodds_ = declare_or_get_parameter<double>(node, p + "miss_logodds", 0.5);
        raycast_miss_logodds_ = declare_or_get_parameter<double>(node, p + "raycast_miss_logodds", 0.3);
        logodds_min_ = declare_or_get_parameter<double>(node, p + "logodds_min", -2.5);
        logodds_max_ = declare_or_get_parameter<double>(node, p + "logodds_max", 3.5);
        occ_enter_thresh_ = declare_or_get_parameter<double>(node, p + "occ_enter_thresh", 0.1);
        occ_exit_thresh_ = declare_or_get_parameter<double>(node, p + "occ_exit_thresh", -0.5);
        free_thresh_ = declare_or_get_parameter<double>(node, p + "free_thresh", -1.0);

        map_directory_ = declare_or_get_parameter<std::string>(node, p + "map_directory", "/tmp/trailblazer_maps");
        map_name_ = declare_or_get_parameter<std::string>(node, p + "map_name", "default");

        inflate_unknown_ = declare_or_get_parameter<bool>(node, p + "inflate_unknown", true);
        inflation_ring1_cost_ = declare_or_get_parameter<int>(node, p + "inflation_ring1_cost", 80);
        inflation_ring2_cost_ = declare_or_get_parameter<int>(node, p + "inflation_ring2_cost", 50);

        submap_min_points_per_cell_ = declare_or_get_parameter<int>(node, p + "submap_min_points_per_cell", 2);
        submap_inflate_ = declare_or_get_parameter<bool>(node, p + "submap_inflate", true);

        RCLCPP_INFO(
            node->get_logger(),
            "Params-loading accomplished!!!"
        );
    }

    // 注意，map_io_ 属于 configure-level resource；lifecycle_active_ 由 activate/deactivate 管
    void OrdinaryOccupancyMapping::create_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        const auto static_map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        const auto submap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        state_pub_ = node->create_publisher<trailblazer_map_interfaces::msg::MapStatus>(
            map_status_topic_,
            status_qos
        );

        occupancy_map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
            static_map_topic_,
            static_map_qos
        );

        occupancy_inflation_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
            inflation_map_topic_,
            static_map_qos
        );

        submap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
            submap_output_topic_,
            submap_qos
        );
    }

    void OrdinaryOccupancyMapping::reset_runtime_state()
    {
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);

            robot_x_ = 0.0;
            robot_y_ = 0.0;
            has_odom_ = false;
        }

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);

            pending_ground_.reset();
            pending_non_ground_.reset();
        }

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            state_ = MapState::NO_MAP;
            resetMapDataLocked();
        }
    }

    void OrdinaryOccupancyMapping::resetMapDataLocked()
    {
        const int cols = static_cast<int>(std::ceil(initial_width_ / resolution_));
        const int rows = static_cast<int>(std::ceil(initial_height_ / resolution_));

        const double origin_x = -0.5 * cols * resolution_;
        const double origin_y = -0.5 * rows * resolution_;

        geometry_.configure(resolution_, origin_x, origin_y, cols, rows);

        final_cells_.assign(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), FinalCell{});

        ground_cells_.clear();
        obstacle_cells_.clear();
        frame_grid_.clear();
        raycast_free_mask_.clear();

        frame_has_observation_ = false;
    }

    OrdinaryOccupancyMapping::DefaultMapLoadResult OrdinaryOccupancyMapping::tryLoadDefaultMap(std::string & message)
    {
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            state_ = MapState::LOADING;
        }

        // 没有文件
        if (!map_io_->hasAnyFile(map_name_)) 
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            state_ = MapState::NO_MAP;
            message = "No map files found for '" + map_name_ + "'.";
            return DefaultMapLoadResult::NOT_FOUND;
        }

        // 有文件缺失（损坏）
        if (!map_io_->exists(map_name_)) 
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            state_ = MapState::ERROR;
            message = "Static-map file set is incomplete for '" + map_name_ + "'.";
            return DefaultMapLoadResult::INVALID;
        }

        // 尝试加载文件内容
        StaticMapData loaded_data;
        const auto result = map_io_->load(map_name_, loaded_data);

        if (!result.success || !loaded_data.valid()) 
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            state_ = MapState::ERROR;
            message = result.message.empty() ? "Loaded static-map data is invalid." : result.message;
            return DefaultMapLoadResult::INVALID;
        }

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            if (!staticMapDataRestore(loaded_data)) 
            {
                state_ = MapState::ERROR;
                message = "Loaded map data invalid, restoration failed.";
                return DefaultMapLoadResult::INVALID;
            }
            state_ = MapState::MAP_READY;
        }

        message = result.message;
        return DefaultMapLoadResult::LOADED;
    }

    bool OrdinaryOccupancyMapping::staticMapDataRestore(StaticMapData & loaded_data)
    {
        if (!loaded_data.valid()) return false;
        
        frame_id_ = loaded_data.frame_id;
        geometry_.configure(loaded_data.resolution, loaded_data.origin_x, loaded_data.origin_y, loaded_data.cols, loaded_data.rows);
        final_cells_ = loaded_data.cells; 
        return true;
    }

    void OrdinaryOccupancyMapping::activate()
    {
        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        if (state_pub_)
            state_pub_->on_activate();

        if (occupancy_map_pub_)
            occupancy_map_pub_->on_activate();

        if (occupancy_inflation_pub_)
            occupancy_inflation_pub_->on_activate();

        if(submap_pub_)
            submap_pub_->on_activate();

        lifecycle_active_.store(true);

        create_subscriptions(node);
        create_timers(node);
        create_services(node);

        RCLCPP_INFO(
            node->get_logger(),
            "OrdinaryOccupancyMapping plugin '%s' activated.",
            plugin_name_.c_str());
    }

    void OrdinaryOccupancyMapping::create_subscriptions(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&OrdinaryOccupancyMapping::odom_callback, this, std::placeholders::_1)
        );

        // 子地图一直处理发布
        submap_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            submap_cloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&OrdinaryOccupancyMapping::submap_cloud_callback, this, std::placeholders::_1)
        );

        // ground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        //         ground_topic_,
        //         rclcpp::SensorDataQoS(),
        //         std::bind(&OrdinaryOccupancyMapping::groundCallback, this, std::placeholders::_1)
        //     );

        // non_ground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        //         non_ground_topic_,
        //         rclcpp::SensorDataQoS(),
        //         std::bind(&OrdinaryOccupancyMapping::nonGroundCallback, this, std::placeholders::_1)
        //     );
    }

    void OrdinaryOccupancyMapping::startMappingSubscriptions()
    {
        if (ground_sub_ || non_ground_sub_) 
            return;

        auto node = node_.lock();
        if (!node) 
        {
            throw std::runtime_error("OrdinaryOccupancy::startMappingSubscriptions() failed: parent node is expired.");
        }

        ground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                ground_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&OrdinaryOccupancyMapping::groundCallback, this, std::placeholders::_1)
            );

        non_ground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                non_ground_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&OrdinaryOccupancyMapping::nonGroundCallback, this, std::placeholders::_1)
            );

        RCLCPP_INFO(
            node->get_logger(),
            "Static mapping subscriptions started: ground=%s, non_ground=%s",
            ground_topic_.c_str(),
            non_ground_topic_.c_str());
    }

    // reset subs && reset pending_ground_/pending_non_ground_
    void OrdinaryOccupancyMapping::stopMappingSubscriptions()
    {
        ground_sub_.reset();
        non_ground_sub_.reset();

        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_ground_.reset();
        pending_non_ground_.reset();
    }

    void OrdinaryOccupancyMapping::create_timers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        state_timer_ = node->create_wall_timer(
            50ms,
            [this]() 
            {
                this->stateTimerCallback(); // 把 node 传进去
            }
        );

        // 计算周期
        auto period = std::chrono::duration<double>(1.0 / map_publish_frequency_);

        map_publish_timer_ = node->create_wall_timer(
            period,
            [this]() 
            {
                this->mapPublishTimerCallback(); // 把 node 传进去
            }
        );
    }

    void OrdinaryOccupancyMapping::create_services(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        start_mapping_server_ = node->create_service<std_srvs::srv::Trigger>(
            start_mapping_service_,
            std::bind(
                &OrdinaryOccupancyMapping::startMappingCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2)
        );

        stop_mapping_server_ =
            node->create_service<std_srvs::srv::Trigger>(
                stop_mapping_service_,
                std::bind(
                    &OrdinaryOccupancyMapping::stopMappingCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)
        );

        error_clearing_server_ =
            node->create_service<std_srvs::srv::Trigger>(
                error_clearing_service_,
                std::bind(
                    &OrdinaryOccupancyMapping::errorClearingCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)
        );

        start_incremental_server_ = node->create_service<std_srvs::srv::Trigger>(
            start_incremental_service_,
            std::bind(
                &OrdinaryOccupancyMapping::startIncrementalCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2)
        );
    }

    void OrdinaryOccupancyMapping::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        std::lock_guard<std::mutex> lock(odom_mutex_);

        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        has_odom_ = std::isfinite(robot_x_) && std::isfinite(robot_y_);
    }

    void OrdinaryOccupancyMapping::submap_cloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_)
            return;

        nav_msgs::msg::OccupancyGrid submap_msg;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            if (!geometry_.valid())
                return;

            submap_msg = buildEmptySubmapLocked(msg->header.stamp);

            // 即使当前没有障碍点，也发布空 layer，
            // 这样上一帧的动态障碍能够被清掉。
            if (msg->width == 0 || msg->height == 0)
            {
                // 后面退出锁以后发布空消息
            }
            else
            {
                std::vector<int> counts(submap_msg.data.size(), 0);

                sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
                {
                    const float x = *iter_x;
                    const float y = *iter_y;
                    const float z = *iter_z;

                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))  
                        continue;

                    int mx = 0;
                    int my = 0;

                    if (!geometry_.worldToMap(x, y, mx, my))
                        continue;
                    
                    const std::size_t idx = geometry_.toIndex(mx, my);

                    if (idx >= counts.size())
                        continue;

                    ++counts[idx];
                }

                for (std::size_t i = 0; i < counts.size(); ++i)
                {
                    if (counts[i] >= submap_min_points_per_cell_)
                        submap_msg.data[i] = 100;
                    
                }

                if (submap_inflate_)
                {
                    submap_msg = inflateObstacleLayer(submap_msg);
                }
            }
        }

        // 不拿着 map_mutex_ publish
        submap_pub_->publish(submap_msg);
    }

    void OrdinaryOccupancyMapping::stateTimerCallback()
    {
        if (!lifecycle_active_) 
            return;

        auto node = node_.lock();
        if (!node) 
            throw std::runtime_error("OrdinaryOccupancy::stateTimerCallback() failed: parent node is expired.");

        trailblazer_map_interfaces::msg::MapStatus status_msg;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            status_msg.header.frame_id = frame_id_;

            status_msg.header.stamp = node->now();

            status_msg.map_status = mapStateToString(state_);
        }

        state_pub_->publish(status_msg);        
    }

    void OrdinaryOccupancyMapping::mapPublishTimerCallback()
    {
        auto node = node_.lock();
        if (!node) 
            throw std::runtime_error("OrdinaryOccupancy::mapPublishTimerCallback() failed: parent node is expired.");

        nav_msgs::msg::OccupancyGrid occupancy_msg;
        nav_msgs::msg::OccupancyGrid inflation_map_msg;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            // 只有 MAP_READY 时定时发布地图
            if (state_ != MapState::MAP_READY)
                return;

            occupancy_msg = toOccupancyGrid(node->now());
            inflation_map_msg = inflateObstacleLayer(occupancy_msg);
        }

        occupancy_map_pub_->publish(occupancy_msg);
        occupancy_inflation_pub_->publish(inflation_map_msg);
    }

    // services callbacks

    void OrdinaryOccupancyMapping::startMappingCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (!lifecycle_active_) 
        {
            response->success = false;
            response->message = "MappingServer is not active.";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            // 只在 NO_MAP/MAP_READY 时允许新建地图
            if(state_ != MapState::NO_MAP && state_ != MapState::MAP_READY)
            {
                response->success = false;
                std::stringstream ss;
                ss << "Current state: " << mapStateToString(state_) << ". " 
                << "Building a new map is only available under NO_MAP/MAP_READY.";
                response->message = ss.str();
                return;
            }

            resetMapDataLocked();

            state_ = MapState::BUILDING_NEW_MAP;
        }

        // 不需要拿着 map_mutex_ 创建 ROS subscription
        startMappingSubscriptions();

        response->success = true;
        response->message = "Start building a new map...";
    }

    void OrdinaryOccupancyMapping::stopMappingCallback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        auto node = node_.lock();
        if (!node) 
            throw std::runtime_error("OrdinaryOccupancy::stopMappingCallback() failed: parent node is expired.");

        if (!lifecycle_active_) 
        {
            response->success = false;
            response->message = "MappingServer is not active.";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            // 只在 BUILDING_NEW_MAP/BUILDING_INCREMENTAL 时允许停止建图
            if(state_ != MapState::BUILDING_NEW_MAP && state_ != MapState::BUILDING_INCREMENTAL)
            {
                response->success = false;
                std::stringstream ss;
                ss << "Current state: " << mapStateToString(state_) << ". " 
                << "Stop mapping is only available under BUILDING_NEW_MAP/BUILDING_INCREMENTAL.";
                response->message = ss.str();
                return;
            }
        }
            
            
            stopMappingSubscriptions();             // 停止点云订阅

            StaticMapData snapshot_data;

            {
                std::lock_guard<std::mutex> lock(map_mutex_);

                snapshot_data = snapShotGetter();             // 获取一帧数据
                state_ = MapState::SAVING;
            }

            StaticMapIO::Result res = map_io_->save(snapshot_data, map_name_, true);              // 保存地图（覆盖原先）

            {
                std::lock_guard<std::mutex> lock(map_mutex_);

                if(!res.success)
                {
                    RCLCPP_ERROR(
                        node->get_logger(),
                        res.message.c_str()
                    );
                    state_ = MapState::ERROR;
                    response->success = false;
                    std::stringstream ss;
                    ss << "Map-saving failed, reason:" << res.message << ".";
                    response->message = ss.str();
                    return;
                }

                state_ = MapState::MAP_READY;           // 直接进入 MAP_READY 发布刚刚保存的地图
            }

            response->success = true;
            response->message = "Stop mapping, map has been automatically saved && loaded.";
            return;
        
    }

    void OrdinaryOccupancyMapping::errorClearingCallback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        auto node = node_.lock();
        if (!node) 
            throw std::runtime_error("OrdinaryOccupancy::errorClearingCallback() failed: parent node is expired.");

        if (!lifecycle_active_) 
        {
            response->success = false;
            response->message = "MappingServer is not active.";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            // 只在 ERROR 时允许 CLEARING
            if(state_ != MapState::ERROR)
            {
                response->success = false;
                std::stringstream ss;
                ss << "Current state: " << mapStateToString(state_) << ". " 
                << "CLEARING is only available under ERROR.";
                response->message = ss.str();
                return;
            }

            state_ = MapState::CLEARING;
        }

            StaticMapIO::Result res = map_io_->remove(map_name_);           // 移除原来的地图文件
            
        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            if(!res.success)
            {
                RCLCPP_ERROR(
                    node->get_logger(),
                    res.message.c_str()
                );
                state_ = MapState::ERROR;
                response->success = false;
                std::stringstream ss;
                ss << "Map-removing failed, reason:" << res.message << ".";
                response->message = ss.str();
                return;
            }

            state_ = MapState::NO_MAP;
        }

            response->success = true;
            response->message = "Error-clearing succeed.";
            return;
        
    }

    void OrdinaryOccupancyMapping::startIncrementalCallback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (!lifecycle_active_) 
        {
            response->success = false;
            response->message = "MappingServer is not active.";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            // 只在 MAP_READY 时允许 INCREMENTAL
            if(state_ != MapState::MAP_READY)
            {
                response->success = false;
                std::stringstream ss;
                ss << "Current state: " << mapStateToString(state_) << ". " 
                << "INCREMENTAL is only available under MAP_READY.";
                response->message = ss.str();
                return;
            }
            else
            {
                startMappingSubscriptions();

                state_ = MapState::BUILDING_INCREMENTAL;
                response->success = true;
                response->message = "Start building incremental map...";
                return;
            }
            
        }
    }

    void OrdinaryOccupancyMapping::groundCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            // 只在 BUILDING_NEW_MAP/BUILDING_INCREMENTAL 时启用
            if(state_ != MapState::BUILDING_NEW_MAP && state_ != MapState::BUILDING_INCREMENTAL)
                return;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr ground;
        sensor_msgs::msg::PointCloud2::SharedPtr non_ground;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            pending_ground_ = std::move(msg);

            if (!takeSynchronizedFrameLocked(ground, non_ground)) 
                return;
        }

        processFrame(ground, non_ground);
    }

    void OrdinaryOccupancyMapping::nonGroundCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            // 只在 BUILDING_NEW_MAP/BUILDING_INCREMENTAL 时启用
            if(state_ != MapState::BUILDING_NEW_MAP && state_ != MapState::BUILDING_INCREMENTAL)
                return;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr ground;
        sensor_msgs::msg::PointCloud2::SharedPtr non_ground;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            pending_non_ground_ = std::move(msg);

            if (!takeSynchronizedFrameLocked(ground, non_ground)) 
                return;
        }

        processFrame(ground, non_ground);
    }

    // 判断 ground_msg/non_ground_msg 的时间差，时间差过大返回 false，否则赋值给 ground/non_ground
    bool OrdinaryOccupancyMapping::takeSynchronizedFrameLocked(
        sensor_msgs::msg::PointCloud2::SharedPtr & ground, 
        sensor_msgs::msg::PointCloud2::SharedPtr & non_ground)
    {
        if (!pending_ground_ || !pending_non_ground_) 
            return false;

        auto node = node_.lock();
        if (!node) 
        {
            throw std::runtime_error("OrdinaryOccupancy::activate() failed: parent node is expired.");
        }

        const auto clock_type = node->get_clock()->get_clock_type();

        const rclcpp::Time ground_stamp(pending_ground_->header.stamp, clock_type);
        const rclcpp::Time non_ground_stamp(pending_non_ground_->header.stamp, clock_type);

        const double delta = std::abs((ground_stamp - non_ground_stamp).seconds());             // delta T

        if (delta > cloud_sync_tolerance_) 
        {
            if (ground_stamp < non_ground_stamp) 
                pending_ground_.reset();
            else 
                pending_non_ground_.reset();

            return false;
        }

        ground = std::move(pending_ground_);
        non_ground = std::move(pending_non_ground_);

        return true;
    }

    void OrdinaryOccupancyMapping::processFrame(
        const sensor_msgs::msg::PointCloud2::SharedPtr & ground, 
        const sensor_msgs::msg::PointCloud2::SharedPtr & non_ground)
    {
        if (!ground || !non_ground) 
            return;

        auto node = node_.lock();
        if (!node) 
        {
            throw std::runtime_error("OrdinaryOccupancy::activate() failed: parent node is expired.");
        }

        double sensor_x = 0.0;
        double sensor_y = 0.0;

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);

            if (!has_odom_) 
            {
                RCLCPP_WARN_THROTTLE
                (
                    node->get_logger(),
                    *node->get_clock(),
                    1000,
                    "[OrdinaryOccupancy]:Odometry is not ready, cannot process frame!!!"
                );

                return;
            }

            sensor_x = robot_x_;
            sensor_y = robot_y_;
        }

        nav_msgs::msg::OccupancyGrid occupancy_msg;
        nav_msgs::msg::OccupancyGrid inflation_msg;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            // 再检查一次。
            if (state_ != MapState::BUILDING_NEW_MAP && state_ != MapState::BUILDING_INCREMENTAL)
            {
                return;
            }

            if (!ensureFrameBounds(*ground, *non_ground))
                return;

            resetFrameBuffers();

            processGroundCloud(*ground);
            processNonGroundCloud(*non_ground);

            if (enable_ground_raycast_)
            {
                raycastCloudFreeSpace(*ground, sensor_x, sensor_y, true);
            }

            if (enable_non_ground_raycast_)
            {
                raycastCloudFreeSpace(*non_ground, sensor_x, sensor_y, false);
            }

            if (!frame_has_observation_)
                return;

            const rclcpp::Time stamp(ground->header.stamp, node->get_clock()->get_clock_type());

            fuseFrameToFinal(stamp);

            occupancy_msg = toOccupancyGrid(node->now());

            inflation_msg = inflateObstacleLayer(occupancy_msg);
        }

        // publish 不需要占着地图锁
        occupancy_map_pub_->publish(occupancy_msg);
        occupancy_inflation_pub_->publish(inflation_msg);
    }

    bool OrdinaryOccupancyMapping::ensureFrameBounds(const sensor_msgs::msg::PointCloud2 & ground, const sensor_msgs::msg::PointCloud2 & non_ground)
    {
        if(!whetherCurrentValid())
            return false;

        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        const bool has_ground = accumulateCloudBounds(ground, min_x, max_x, min_y, max_y);
        const bool has_non_ground = accumulateCloudBounds(non_ground, min_x, max_x, min_y, max_y);

        if (!has_ground && !has_non_ground)
            return false;

        return ensureBounds(min_x, max_x, min_y, max_y, expand_margin_);
    }

    bool OrdinaryOccupancyMapping::accumulateCloudBounds(
        const sensor_msgs::msg::PointCloud2 &msg, 
        double &min_x, double &max_x, double &min_y, double &max_y)
    {
        if (msg.width == 0 || msg.height == 0)
            return false;

        bool found_valid_point = false;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

        for(;iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            min_x = std::min(min_x, static_cast<double>(x));
            min_y = std::min(min_y, static_cast<double>(y));
            max_x = std::max(max_x, static_cast<double>(x));
            max_y = std::max(max_y, static_cast<double>(y));

            found_valid_point = true;
        }
        return found_valid_point;
    }

    bool OrdinaryOccupancyMapping::ensureBounds(double min_x, double max_x, double min_y, double max_y, double expand_margin)
    {
        // 当前边界足够大
        if(geometry_.minX() <= min_x && geometry_.minY() <= min_y && geometry_.maxX() >= max_x && geometry_.maxY() >= max_y)
            return true;

        double new_min_x = std::min(min_x, geometry_.minX()) - expand_margin;
        double new_min_y = std::min(min_y, geometry_.minY()) - expand_margin;
        double new_max_x = std::max(max_x, geometry_.maxX()) + expand_margin;
        double new_max_y = std::max(max_y, geometry_.maxY()) + expand_margin;

        int new_cols = static_cast<int>(std::ceil((new_max_x - new_min_x) / geometry_.resolution()));
        int new_rows = static_cast<int>(std::ceil((new_max_y - new_min_y) / geometry_.resolution()));

        // resize
        if(new_cols <= 0 || new_rows <= 0)
            return false;

        std::vector<FinalCell> new_final_cells(new_cols * new_rows);

        double wx, wy;
        int mx, my;
        for(int i = 0; i < geometry_.cols(); i++)
        {
            for(int j = 0; j < geometry_.rows(); j++)
            {
                // map 中的格子坐标
                size_t index = geometry_.toIndex(i, j);

                if(!final_cells_[index].ever_updated)
                    continue;

                // 转换为世界坐标
                geometry_.mapToWorld(i, j, wx, wy);

                // 转换为新的格子坐标
                mx = static_cast<int>(std::floor((wx - new_min_x) / geometry_.resolution()));
                my = static_cast<int>(std::floor((wy - new_min_y) / geometry_.resolution()));

                if(mx >= 0 && mx < new_cols && my >= 0 && my < new_rows)
                {
                    int new_index = static_cast<size_t>(mx) + static_cast<size_t>(my) * static_cast<size_t>(new_cols);
                    new_final_cells[new_index] = final_cells_[index];
                }
            }
        }

        // 更新 geometry 中的地图几何数据以及 final_cells
        geometry_.configure(geometry_.resolution(), new_min_x, new_min_y, new_cols, new_rows);
        final_cells_.swap(new_final_cells);
        return true;
    }

    void OrdinaryOccupancyMapping::resetFrameBuffers()
    {
        if(!whetherCurrentValid())
        {
            ground_cells_.clear();
            obstacle_cells_.clear();
            frame_grid_.clear();
            raycast_free_mask_.clear();
            frame_has_observation_ = false;
            return;
        }

        const std::size_t grid_size = final_cells_.size();

        ground_cells_.assign(grid_size, GroundCell{});
        obstacle_cells_.assign(grid_size, ObstacleCell{});
        frame_grid_.assign(grid_size, -1);
        raycast_free_mask_.assign(grid_size, 0);

        frame_has_observation_ = false;
    }

    void OrdinaryOccupancyMapping::processGroundCloud(const sensor_msgs::msg::PointCloud2 & msg)
    {
        std::vector<size_t> updated_indices;
        updated_indices.reserve(static_cast<std::size_t>(msg.width * msg.height));

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            int mx = 0;
            int my = 0;
            
            if (!geometry_.worldToMap(x, y, mx, my))
                continue;

            const size_t index = geometry_.toIndex(mx, my);
            auto &cell = ground_cells_[static_cast<std::size_t>(index)];

            cell.observed = true;
            cell.count += 1;
            cell.sum_z += z;
            cell.z_values.push_back(z);

            updated_indices.push_back(index);
        }

        if (!updated_indices.empty())
        {
            finalizeGroundCells(updated_indices);
        }
    }

    void OrdinaryOccupancyMapping::finalizeGroundCells(const std::vector<std::size_t> & updated_indices)
    {
        // 使用 unordered_set 去重
        std::unordered_set<int> unique_indices(updated_indices.begin(), updated_indices.end());

        for (const int idx : unique_indices)
        {
            if (idx < 0 || idx >= static_cast<int>(ground_cells_.size()))
                continue;

            auto &cell = ground_cells_[static_cast<std::size_t>(idx)];
            if (!cell.observed || cell.z_values.empty() || cell.count < min_ground_points_per_cell_)
                continue;

            cell.mean_z = cell.sum_z / static_cast<float>(cell.count);          // 格子内平均高度
            cell.ground_z = compute_percentile(cell.z_values, static_cast<float>(ground_percentile_));      // 估计格子地面高度

            // 第一步只做 ground mask：观测到地面则记为 free(0)，未观测仍为 unknown(-1)
            frame_grid_[static_cast<std::size_t>(idx)] = 0;

            frame_has_observation_ = true;
        }
    }

    // 根据格子（或周围）的地面高度，将高于 min_obstacle_height 的点填入 obstacle_cells_，之后 finalizeObstacleCells
    void OrdinaryOccupancyMapping::processNonGroundCloud(const sensor_msgs::msg::PointCloud2 & msg)
    {
        std::vector<size_t> updated_indices;
        updated_indices.reserve(static_cast<std::size_t>(msg.width * msg.height));

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

        for(;iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            int mx = 0;
            int my = 0;
            if (!geometry_.worldToMap(x, y, mx, my))
                continue;

            const int index = geometry_.toIndex(mx, my);

            float h;

            if(if_strict_)
            {
                // 查询附近地面格子高度，并使用相对高度来判断
                float ground_z;
                bool has_ground = queryGroundHeight(mx, my, ground_z);
                if(!has_ground)
                    continue;           // 找不到对应的地面格子高度，就抛弃这个点（保持格子为未知状态）

                h = z - ground_z;
            }
            else
            {
                // 使用绝对高度来判断
                h = z;
            }

            if (h < min_obstacle_height_)           // 点的高度太小就不算障碍物
                    continue;

            auto &cell = obstacle_cells_[static_cast<std::size_t>(index)];

            // 对应障碍物格子赋值
            cell.observed = true;
            cell.non_ground_count += 1;
            cell.sum_height += h;
            
            if(h >= strong_obstacle_height_)
            {
                cell.strong_count += 1;
            }
            updated_indices.push_back(index);
        }

        finalizeObstacleCells(updated_indices);
    }

    bool OrdinaryOccupancyMapping::queryGroundHeight(int mx, int my, float & ground_z) const
    {
        const int index = geometry_.toIndex(mx, my);
        auto &cell = ground_cells_[index];
        if (cell.observed && cell.count >= min_ground_points_per_cell_)      // 对应的地面格子存在
        {
            ground_z = cell.ground_z;
            return true;
        }
        else
        {
            int find_count = 0;
            float sum_ground_z = 0.0;
            // 本格未找到，在8邻域搜索(取周围邻居高度的平均值)
            for (int i = -1; i <= 1; ++i)
            { 
                for (int j = -1; j <= 1; ++j)
                {
                    if (i == 0 && j == 0) 
                        continue;               // 排除中心格子
                    if (mx+i < 0 || mx+i >= geometry_.cols() || my+j < 0 || my+j >= geometry_.rows())
                        continue;
                    const int index = geometry_.toIndex(mx + i, my + j);
                    auto &cell = ground_cells_[index];
                    if (cell.observed && cell.count >= min_ground_points_per_cell_)       // 对应的地面格子存在
                    {
                        sum_ground_z += cell.ground_z;
                        ++find_count;
                    }
                }
            }
            if(find_count == 0)
                return false;
            ground_z = sum_ground_z / static_cast<float>(find_count);
            return true;
        }
    }

    void OrdinaryOccupancyMapping::finalizeObstacleCells(const std::vector<std::size_t> & updated_indices)
    {
        std::unordered_set<int> unique_indices(updated_indices.begin(), updated_indices.end());
        for(const int idx : unique_indices)
        {
            if (idx < 0 || idx >= static_cast<int>(obstacle_cells_.size()))
                continue;

            auto &cell = obstacle_cells_[static_cast<std::size_t>(idx)];
            if(!cell.observed || cell.non_ground_count == 0)
                continue;

            cell.mean_height = cell.sum_height / static_cast<float>(cell.non_ground_count);
            
            if(cell.non_ground_count >= min_non_ground_count_ && cell.mean_height >= min_mean_height_)
            {
                // 格子内障碍物点数够多，且平均高度够大，认为是障碍物候选
                cell.is_candidate = true;
            }
            else if(cell.non_ground_count >= min_non_ground_count_ && cell.strong_count >= min_strong_count_)
            {
                // 格子中障碍物点数够多，且强证据点数够多，也认为是障碍物
                cell.is_candidate = true;
            }
            else
            {
                cell.is_candidate = false;
            }
            if (cell.is_candidate)
                frame_grid_[static_cast<std::size_t>(idx)] = 1;

            frame_has_observation_ = true;
        }
    }

    int OrdinaryOccupancyMapping::raycastCloudFreeSpace(
        const sensor_msgs::msg::PointCloud2 & msg, 
        double sensor_x, double sensor_y, bool include_endpoint)
    {
        int start_mx = 0;
        int start_my = 0;
        if (!geometry_.worldToMap(sensor_x, sensor_y, start_mx, start_my))
            return 0;

        int updated = 0;
        const int step = std::max(1, raycast_point_step_);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

        std::size_t point_id = 0;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_id)
        {
            if (static_cast<int>(point_id % static_cast<std::size_t>(step)) != 0)
                continue;

            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            // 点云距离机器人距离
            const double range_xy = std::hypot(static_cast<double>(x) - sensor_x, static_cast<double>(y) - sensor_y);

            if (range_xy < raycast_min_range_ || range_xy > raycast_max_range_)
                continue;

            int end_mx = 0;
            int end_my = 0;
            if (!geometry_.worldToMap(x, y, end_mx, end_my))
                continue;

            updated += raycastFreeCells(start_mx, start_my, end_mx, end_my, include_endpoint);
        }

        return updated;
    }

    int OrdinaryOccupancyMapping::raycastFreeCells(int start_mx, int start_my, int end_mx, int end_my, bool include_endpoint)
    {
        if (!geometry_.isValidIndex(start_mx, start_my) || !geometry_.isValidIndex(end_mx, end_my))
            return 0;

        int updated = 0;

        int x0 = start_mx;
        int y0 = start_my;
        const int x1 = end_mx;
        const int y1 = end_my;

        const int dx = std::abs(x1 - x0);
        const int dy = -std::abs(y1 - y0);

        // 步进方向
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;

        int err = dx + dy;

        bool first_cell = true;

        while (true)
        {
            const bool is_endpoint = (x0 == x1 && y0 == y1);

            if (!first_cell)
            {
                const int dist_to_endpoint = std::max(std::abs(x0 - x1), std::abs(y0 - y1));

                const bool too_close_to_endpoint = (!include_endpoint && dist_to_endpoint <= raycast_endpoint_margin_cells_);

                if (!too_close_to_endpoint && (!is_endpoint || include_endpoint))
                {
                    if (markRaycastFreeCell(x0, y0))
                        ++updated;
                }
            }

            if (is_endpoint)
                break;

            // bresenham
            const int e2 = 2 * err;

            if (e2 >= dy)
            {
                err += dy;
                x0 += sx;
            }

            if (e2 <= dx)
            {
                err += dx;
                y0 += sy;
            }

            first_cell = false;
        }

        return updated;
    }

    bool OrdinaryOccupancyMapping::markRaycastFreeCell(int mx, int my)
    {
        if (!geometry_.isValidIndex(mx, my))
            return false;

        const std::size_t idx = geometry_.toIndex(mx, my);

        // 不覆盖障碍物
        if (frame_grid_[static_cast<std::size_t>(idx)] == 1)
            return false;

        // 覆盖 unknown
        if (frame_grid_[static_cast<std::size_t>(idx)] == -1)
        {
            frame_grid_[static_cast<std::size_t>(idx)] = 0;
            raycast_free_mask_[static_cast<std::size_t>(idx)] = 1;
            frame_has_observation_ = true;
            return true;
        }

        return false;
    }

    void OrdinaryOccupancyMapping::fuseFrameToFinal(const rclcpp::Time & stamp)
    {
        for (size_t i = 0; i < final_cells_.size(); ++i)
        {
            const int8_t obs = frame_grid_[i];          // 获取当前帧的 map 数据 
            if (obs == -1) continue;        // 不处理未知情况

            auto& fc = final_cells_[i];

            if (obs == 1) 
            {
                fc.log_odds += hit_logodds_;
            }
            else if (obs == 0) 
            {
                // 是否是 raycast 出来的 free
                const bool raycast_free = (i < raycast_free_mask_.size() && raycast_free_mask_[i] != 0);

                fc.log_odds -= static_cast<float>(raycast_free ? raycast_miss_logodds_ : miss_logodds_);
            }

            fc.log_odds = std::clamp(fc.log_odds, (float)logodds_min_, (float)logodds_max_);
            fc.last_update = stamp;
            fc.ever_updated = true;
        }

        std::fill(raycast_free_mask_.begin(), raycast_free_mask_.end(), 0);
        frame_has_observation_ = false;
    }

    nav_msgs::msg::OccupancyGrid OrdinaryOccupancyMapping::toOccupancyGrid(const rclcpp::Time & stamp)
    {
        nav_msgs::msg::OccupancyGrid msg;
        if(!(geometry_.valid() && 
                (final_cells_.size() == static_cast<size_t>(geometry_.cols() * geometry_.rows()))))
            return msg;

        msg.header.frame_id = frame_id_;
        msg.header.stamp = stamp;
        msg.info.map_load_time = stamp;
        msg.info.resolution = static_cast<float>(geometry_.resolution());
        msg.info.width = static_cast<uint32_t>(geometry_.cols());
        msg.info.height = static_cast<uint32_t>(geometry_.rows());
        msg.info.origin.position.x = geometry_.originX();
        msg.info.origin.position.y = geometry_.originY();
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        msg.data.resize(final_cells_.size());
        for (std::size_t i = 0; i < final_cells_.size(); ++i) 
        {
            msg.data[i] = updateAndGetOccupancy(final_cells_[i]);
        }
        return msg;
    }

    /*
    * 根据占据概率判断格子 occupied 还是 free
    *       对于 free/unknown 格子：
    *           大于 enter_thresh 就 occupied，小于 free_thresh 就 free
    *       对于 occupied 格子：
    *           只有小于 exit_thresh 才改变其 occupied 状态（并不是小于 enter_thresh 就取消 occupied，
    *           这里为 occupied 类型添加了一个滞回，使得 occupied 一旦进入就没那么容易退出）
    */
    int8_t OrdinaryOccupancyMapping::updateAndGetOccupancy(FinalCell & cell)
    {
        if (!cell.ever_updated) return -1; // 未知

        // 滞回策略判断状态转移
        if (cell.state == FinalState::Occupied) 
        {
            if (cell.log_odds <= occ_exit_thresh_) 
            {
                if (cell.log_odds <= free_thresh_) 
                    cell.state = FinalState::Free;
                else 
                    cell.state = FinalState::Unknown;
            }
        } 
        else 
        {
            if (cell.log_odds >= occ_enter_thresh_) 
                cell.state = FinalState::Occupied;
            else if (cell.log_odds <= free_thresh_) 
                cell.state = FinalState::Free;
            else 
                cell.state = FinalState::Unknown;
        }

        if (cell.state == FinalState::Occupied) return 100;
        if (cell.state == FinalState::Free) return 0;
        return -1; // Unknown
    }
    
    StaticMapData OrdinaryOccupancyMapping::snapShotGetter()
    {
        StaticMapData data;
        data.frame_id = frame_id_;       

        data.resolution = geometry_.resolution();
        data.origin_x = geometry_.originX();
        data.origin_y = geometry_.originY();

        data.cols = geometry_.cols();
        data.rows = geometry_.rows();

        data.cells = final_cells_;

        return data;
    }
    
    void OrdinaryOccupancyMapping::inflateSingleCell(int occ_mx, int occ_my, nav_msgs::msg::OccupancyGrid &map_msg)
    {
        int center_idx = geometry_.toIndex(occ_mx, occ_my); 

        map_msg.data[center_idx] = 100;

        int nx, ny;
        for(int i = -1; i <= 1; i++)
        {
            for(int j = -1; j <= 1; j++)
            {
                if(i == 0 && j == 0)
                    continue;
                // 邻居坐标
                nx = occ_mx + i;
                ny = occ_my + j;

                if (!geometry_.isValidIndex(nx, ny))
                    continue;

                int nidx = geometry_.toIndex(nx, ny);

                if(geometry_.isValidIndex(nx, ny) && map_msg.data[nidx] != 100)
                {
                    if(map_msg.data[nidx] == -1)     // 是未知区域
                    {
                        if(inflate_unknown_)        // 允许在未知区域膨胀
                        {
                            map_msg.data[nidx] = inflation_ring1_cost_;
                        }
                    }
                    else
                    {
                        map_msg.data[nidx] = std::max(static_cast<int>(map_msg.data[nidx]), inflation_ring1_cost_);
                    }
                }

            }
        }
        
        // 第二圈膨胀
        for(int i = -2; i <= 2; i++)
        {
            for(int j = -2; j <= 2; j++)
            {
                if(i == 0 && j == 0)
                    continue;
                // 邻居坐标
                nx = occ_mx + i;
                ny = occ_my + j;

                if (!geometry_.isValidIndex(nx, ny))
                    continue;

                int nidx = geometry_.toIndex(nx, ny);

                if(geometry_.isValidIndex(nx, ny) && map_msg.data[nidx] != 100 && map_msg.data[nidx] != inflation_ring1_cost_)
                {
                    if(map_msg.data[nidx] == -1)     // 是未知区域
                    {
                        if(inflate_unknown_)        // 允许在未知区域膨胀
                        {
                            map_msg.data[nidx] = inflation_ring2_cost_;
                        }
                    }
                    else
                    {
                        map_msg.data[nidx] = std::max(static_cast<int>(map_msg.data[nidx]), inflation_ring2_cost_);
                    }
                }
                
            }
        }
    }

    nav_msgs::msg::OccupancyGrid OrdinaryOccupancyMapping::inflateObstacleLayer(nav_msgs::msg::OccupancyGrid &map_msg)
    {
        nav_msgs::msg::OccupancyGrid output_grid;
        output_grid = map_msg;

        // 记录下障碍物格子
        std::vector<int> obstacle_indices;
        obstacle_indices.reserve(map_msg.data.size());

        for (int my = 0; my < geometry_.rows(); ++my)
        {
            for (int mx = 0; mx < geometry_.cols(); ++mx)
            {
                const int idx = geometry_.toIndex(mx, my);
                if (map_msg.data[static_cast<std::size_t>(idx)] == 100)
                {
                    obstacle_indices.push_back(idx);
                }
            }
        }

        for (const int idx : obstacle_indices)
        {
            const int mx = idx % geometry_.cols();
            const int my = idx / geometry_.cols();
            inflateSingleCell(mx, my, output_grid);
        }

        return output_grid;
    }

    void OrdinaryOccupancyMapping::deactivate()
    {
        auto node = node_.lock();

        // 先挡住所有 callback
        lifecycle_active_.store(false);

        // 停止业务输入
        stopMappingSubscriptions();
        odom_sub_.reset();

        // 停止 timers
        if (state_timer_)
        {
            state_timer_->cancel();
            state_timer_.reset();
        }

        if (map_publish_timer_)
        {
            map_publish_timer_->cancel();
            map_publish_timer_.reset();
        }

        // services 是 activate 阶段创建的，所以 deactivate 时一起销毁
        start_mapping_server_.reset();
        stop_mapping_server_.reset();
        error_clearing_server_.reset();
        start_incremental_server_.reset();

        // 如果生命周期切换时恰好还处于建图状态，
        // 不要让重新 activate 后状态还显示 BUILDING，
        // 因为 subscriptions 已经不存在了。
        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            if (state_ == MapState::BUILDING_NEW_MAP || state_ == MapState::BUILDING_INCREMENTAL)
            {
                state_ = whetherCurrentValid() ? MapState::MAP_READY : MapState::NO_MAP;
            }
        }

        // deactivate publishers
        if (occupancy_inflation_pub_)
            occupancy_inflation_pub_->on_deactivate();

        if (occupancy_map_pub_)
            occupancy_map_pub_->on_deactivate();

        if (state_pub_)
            state_pub_->on_deactivate();

        if(submap_pub_)
            submap_pub_->on_deactivate();

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "OrdinaryOccupancyMapping plugin '%s' deactivated.",
                plugin_name_.c_str());
        }
    }

    void OrdinaryOccupancyMapping::cleanup()
    {
        auto node = node_.lock();

        lifecycle_active_.store(false);

        // 幂等保护：理论上 cleanup 前已经 deactivate，但再 reset 一次没有坏处
        stopMappingSubscriptions();

        odom_sub_.reset();

        if (state_timer_)
        {
            state_timer_->cancel();
            state_timer_.reset();
        }

        if (map_publish_timer_)
        {
            map_publish_timer_->cancel();
            map_publish_timer_.reset();
        }

        start_mapping_server_.reset();
        stop_mapping_server_.reset();
        error_clearing_server_.reset();
        start_incremental_server_.reset();

        // configure 阶段创建的 publishers 全部 reset
        state_pub_.reset();
        occupancy_map_pub_.reset();
        occupancy_inflation_pub_.reset();
        submap_pub_.reset();

        // 文件 IO 对象也是 configure resource
        map_io_.reset();

        {
            std::lock_guard<std::mutex> lock(map_mutex_);

            state_ = MapState::NO_MAP;

            geometry_ = GridGeometry{};

            ground_cells_.clear();
            obstacle_cells_.clear();

            raycast_free_mask_.clear();
            frame_grid_.clear();
            final_cells_.clear();

            frame_has_observation_ = false;
        }

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);

            pending_ground_.reset();
            pending_non_ground_.reset();
        }

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);

            robot_x_ = 0.0;
            robot_y_ = 0.0;
            has_odom_ = false;
        }

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "OrdinaryOccupancyMapping plugin '%s' cleaned up.",
                plugin_name_.c_str());
        }

        plugin_name_.clear();

        // 最后一件事再丢掉 node weak reference
        node_.reset();
    }

    nav_msgs::msg::OccupancyGrid OrdinaryOccupancyMapping::buildEmptySubmapLocked(const builtin_interfaces::msg::Time & stamp)
    {
        nav_msgs::msg::OccupancyGrid msg;

        if (!geometry_.valid())
            return msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;

        msg.info.map_load_time = stamp;

        msg.info.resolution = static_cast<float>(geometry_.resolution());
        msg.info.width = static_cast<std::uint32_t>(geometry_.cols());
        msg.info.height = static_cast<std::uint32_t>(geometry_.rows());

        msg.info.origin.position.x = geometry_.originX();
        msg.info.origin.position.y = geometry_.originY();
        msg.info.origin.position.z = 0.0;

        msg.info.origin.orientation.x = 0.0;
        msg.info.origin.orientation.y = 0.0;
        msg.info.origin.orientation.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        msg.data.assign(static_cast<std::size_t>(geometry_.cols() * geometry_.rows()), -1);

        return msg;
    }

    // =============================================== helpers ===============================================

    bool OrdinaryOccupancyMapping::whetherCurrentValid()
    {
        return geometry_.valid() && (final_cells_.size() == static_cast<size_t>(geometry_.cols() * geometry_.rows()));
    }

    float OrdinaryOccupancyMapping::compute_percentile(std::vector<float> values, float q) const
    {
        if (values.empty())
            return 0.0f;

        q = std::clamp(q, 0.0f, 1.0f);
        std::sort(values.begin(), values.end());

        const std::size_t idx = std::min(
            static_cast<std::size_t>(q * static_cast<float>(values.size() - 1)),
            values.size() - 1);

        return values[idx];
    }

    std::string OrdinaryOccupancyMapping::mapStateToString(MapState state) 
    {
        switch (state) 
        {
            case MapState::NO_MAP:            return "NO_MAP";
            case MapState::MAP_READY:         return "MAP_READY";
            case MapState::BUILDING_NEW_MAP:  return "BUILDING_NEW_MAP";
            case MapState::BUILDING_INCREMENTAL: return "BUILDING_INCREMENTAL";
            case MapState::SAVING:            return "SAVING";
            case MapState::LOADING:           return "LOADING";
            case MapState::CLEARING:          return "CLEARING";
            case MapState::ERROR:             return "ERROR";
            default:                          return "UNKNOWN_STATE";
        }
    }
}       // namespace occupancy_mapping

PLUGINLIB_EXPORT_CLASS(occupancy_mapping::OrdinaryOccupancyMapping, occupancy_mapping::OccupancyMappingBase)