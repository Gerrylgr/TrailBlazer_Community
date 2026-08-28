#include "global_planner/astar_esdf_global_planner.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <Eigen/Dense>
#include <queue>

using namespace std::chrono_literals;

namespace global_planner
{
    // std::pair<> 的输出流重载
    template <typename T1, typename T2>
    std::ostream& operator<<(std::ostream& os, const std::pair<T1, T2>& p)
    {
        os << "(" << p.first << ", " << p.second << ")";
        return os;
    }

    void AstarEsdfGlobalPlanning::configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name)
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
            "AstarEsdfGlobalPlanning plugin '%s' configured.",
            plugin_name_.c_str()
        );
    }

    void AstarEsdfGlobalPlanning::load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const std::string p = plugin_name_ + ".";

        if_silence_ = declare_or_get_parameter<bool>(node, p + "if_silence", false);

        global_path_topic_ = declare_or_get_parameter<std::string>(node, p + "global_path_topic", "/original_global_path");
        input_costmap_topic_ = declare_or_get_parameter<std::string>(node, p + "input_costmap_topic", "/inflation_occupancy_map");
        input_esdf_topic_ = declare_or_get_parameter<std::string>(node, p + "input_esdf_topic", "/esdf_map");
        input_odom_topic_ = declare_or_get_parameter<std::string>(node, p + "input_odom_topic", "/Odometry_centered");
        input_goal_topic_ = declare_or_get_parameter<std::string>(node, p + "input_goal_topic", "/goal_pose");

        if_keep_planning_ = declare_or_get_parameter<bool>(node, p + "if_keep_planning", true);

        min_dist_to_plan_ = declare_or_get_parameter<double>(node, p + "min_dist_to_plan", 0.2);
        treat_unknown_as_obstacle_ = declare_or_get_parameter<bool>(node, p + "treat_unknown_as_obstacle", false);
        obstacle_threshold_ = declare_or_get_parameter<int>(node, p + "obstacle_threshold", 50);
        min_esdf_value_for_goal_ = declare_or_get_parameter<double>(node, p + "min_esdf_value_for_goal", 0.2);

        unknown_cost_penalty_ = declare_or_get_parameter<int>(node, p + "unknown_cost_penalty", 50);
        cost_scale_ = declare_or_get_parameter<double>(node, p + "cost_scale", 2.0);
        // heuristic_tie_breaker_ = declare_or_get_parameter<double>(node, p + "heuristic_tie_breaker", 1.001);

        if_path_downsampling_ = declare_or_get_parameter<bool>(node, p + "if_path_downsampling", false);
        corner_deg_ = declare_or_get_parameter<int>(node, p + "corner_deg", 30);
        corner_dilate_ = declare_or_get_parameter<int>(node, p + "corner_dilate", 1);

        use_esdf_soft_cost_ = declare_or_get_parameter<bool>(node, p + "use_esdf_soft_cost", true);
        safe_clearance_ = declare_or_get_parameter<double>(node, p + "safe_clearance", 1.0);
        esdf_cost_scale_ = declare_or_get_parameter<double>(node, p + "esdf_cost_scale", 3.0);

        RCLCPP_INFO(
            node->get_logger(),
            "Params-loading accomplished!!!"
        );
    }

    void AstarEsdfGlobalPlanning::create_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
            global_path_topic_,
            path_qos
        );
    }

    void AstarEsdfGlobalPlanning::reset_runtime_state()
    {
        lifecycle_active_.store(false);

        esdf_buffer_.clear();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            robot_x_ = 0.0;
            robot_y_ = 0.0;
            has_odom_ = false;
            occupancy_map_msg_.reset();
            goal_.reset();
            next_goal_generation_ = 0;
        }

        grid_initialized_ = false;
        last_grid_height_ = 0;
        last_grid_width_ = 0;
    }

    void AstarEsdfGlobalPlanning::activate()
    {
        auto node = node_.lock();

        if (!node)
            throw std::runtime_error("Parent node expired.");

        if (path_pub_)
            path_pub_->on_activate();

        lifecycle_active_.store(true);

        create_subscriptions(node);
        create_timers(node);
        // create_services(node);

        RCLCPP_INFO(
            node->get_logger(),
            "AstarEsdfGlobalPlanning plugin '%s' activated.",
            plugin_name_.c_str());
    }

    void AstarEsdfGlobalPlanning::create_subscriptions(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        const auto esdf_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        occupancy_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            input_costmap_topic_,
            map_qos,
            std::bind(&AstarEsdfGlobalPlanning::occupancy_map_callback, this, std::placeholders::_1)
        );

        esdf_sub_ = node->create_subscription<trailblazer_map_interfaces::msg::EsdfMap>(
            input_esdf_topic_,
            esdf_qos,
            std::bind(&AstarEsdfGlobalPlanning::esdf_map_callback, this, std::placeholders::_1)
        );

        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&AstarEsdfGlobalPlanning::odom_callback, this, std::placeholders::_1)
        );

        goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_goal_topic_,
            10,
            std::bind(&AstarEsdfGlobalPlanning::goal_callback, this, std::placeholders::_1)
        );
    }

    void AstarEsdfGlobalPlanning::create_timers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        planning_timer = node->create_wall_timer(
            100ms,
            [this]() 
            {
                this->planningTimerCallback(); 
            }
        );
    }

    void AstarEsdfGlobalPlanning::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);

        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        has_odom_ = std::isfinite(robot_x_) && std::isfinite(robot_y_);
    }

    void AstarEsdfGlobalPlanning::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y))
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);

        goal_ = GoalState{
            msg->pose.position.x,
            msg->pose.position.y,
            ++next_goal_generation_,
            false,
            msg->header.frame_id
        };
    }

    void AstarEsdfGlobalPlanning::esdf_map_callback(const trailblazer_map_interfaces::msg::EsdfMap::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        esdf_buffer_.update(msg);
    }

    void AstarEsdfGlobalPlanning::occupancy_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (!msg || !lifecycle_active_) 
            return;

        auto node = node_.lock();
        if (!node)
            throw std::runtime_error("Parent node expired.");

        std::lock_guard<std::mutex> lock(state_mutex_);
        occupancy_map_msg_ = msg;
    }

    void AstarEsdfGlobalPlanning::planningTimerCallback()
    {
        if (!lifecycle_active_) 
            return;

        auto node = node_.lock();
        if (!node)
            throw std::runtime_error("Parent node expired.");

        // 尝试拿锁（如果有其他线程在跑 planningTimerCallback，则无法拿到）
        std::unique_lock<std::mutex> planning_lock(planning_mutex_, std::try_to_lock);
        if (!planning_lock.owns_lock())
        {
            // 上一次全局规划还没有结束
            return;
        }

        if (!lifecycle_active_.load(std::memory_order_acquire))
            return;

        const auto start_time = std::chrono::steady_clock::now();

        // ---------------- 先取出可能会有线程竞争的变量 ----------------
        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_msg;
        std::optional<GoalState> goal;
        double robot_x;
        double robot_y;
        bool has_odom;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            occupancy_map_msg = occupancy_map_msg_;
            goal = goal_;
            robot_x = robot_x_;
            robot_y = robot_y_;
            has_odom = has_odom_;
        }

        // ---------------- 是否需要取消规划 ----------------
        if (!goal.has_value())                 // 没有 goal
        {
            if (!if_silence_)
            {
                RCLCPP_INFO_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    1500,
                    "No goal yet, skip planning."
                );
            }
            return;
        }

        if (!has_odom)                 // 没有 odom
        {
            if (!if_silence_)
            {
                RCLCPP_INFO_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    1500,
                    "No odom yet, skip planning."
                );
            }
            return;
        }

        if(!if_keep_planning_)
        {
            if(goal->planned)
                return;             // 当前目标已经规划完成
        }

        // 是否距离目标过近
        const double distance_to_goal = std::hypot(robot_x - goal->x, robot_y - goal->y);
        if (distance_to_goal < min_dist_to_plan_)
        {
            if (!if_silence_)
            {
                RCLCPP_INFO_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    1500,
                    "Goal is already close enough, skip planning."
                );
            }
            return;
        }

        // ---------------- 解析栅格地图元数据，并与 ESDF 地图元数据比对 ----------------
        if (!occupancy_map_msg)
        {
            RCLCPP_INFO_THROTTLE(
                node->get_logger(), *node->get_clock(), 1500,
                "No occupancy map yet, skip planning.");
            return;
        }
        const int grid_height = static_cast<int>(occupancy_map_msg->info.height);
        const int grid_width  = static_cast<int>(occupancy_map_msg->info.width);
        const double resolution = occupancy_map_msg->info.resolution;
        const double originX = occupancy_map_msg->info.origin.position.x;
        const double originY = occupancy_map_msg->info.origin.position.y;

        if (grid_width == 0 || grid_height == 0 ||
            grid_width > std::numeric_limits<int>::max() ||
            grid_height > std::numeric_limits<int>::max() ||
            !std::isfinite(resolution) || resolution <= 0.0 ||
            !std::isfinite(originX) || !std::isfinite(originY))
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid occupancy map metadata.");
            return;
        }

        const auto esdf = esdf_buffer_.snapshot();              // 获取 EsdfMapSnapshot 对象
        if (!esdf)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "ESDF map is not ready."
            );
            return;
        }

        if (!esdf->geometry_matches(grid_width, grid_height, resolution, originX, originY))
        {
            RCLCPP_WARN(
                node->get_logger(),
                "ESDF geometry does not match costmap."
            );
            return;
        }

        // ---------------- 判断规划起点/终点是否合法（超出地图范围） ----------------
        const int robot_column = static_cast<int>(std::floor((robot_x - originX) / resolution));
        const int robot_row    = static_cast<int>(std::floor((robot_y - originY) / resolution));
        const int goal_column  = static_cast<int>(std::floor((goal->x - originX) / resolution));
        const int goal_row     = static_cast<int>(std::floor((goal->y - originY) / resolution));

        auto in_map = [&](int x, int y) -> bool
        {
            return x >= 0 && x < grid_width && y >= 0 && y < grid_height;
        };

        if (!in_map(robot_column, robot_row) || !in_map(goal_column, goal_row))
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Start or goal is out of map bounds. start=(%d,%d), goal=(%d,%d), map=(%d,%d)",
                robot_column, robot_row, goal_column, goal_row, grid_width, grid_height
            );
            return;
        }

        // ---------------- initialize/resize grid_cost_ ----------------
        if (!grid_initialized_ || grid_height != last_grid_height_ || grid_width != last_grid_width_)
        {
            grid_cost_.assign(grid_height, std::vector<int>(grid_width, 0));
            grid_initialized_ = true;
            last_grid_height_ = grid_height;
            last_grid_width_ = grid_width;

            if (!if_silence_)
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Grid cost map initialized to %dx%d",
                    grid_height, grid_width
                );
            }
        }

        for (int y = 0; y < grid_height; ++y)
        {
            for (int x = 0; x < grid_width; ++x)
            {
                // 为 grid_cost_ 赋值
                grid_cost_[y][x] = static_cast<int>(occupancy_map_msg->data[y * grid_width + x]);
            }
        }
        
        // ---------------- 检查起点/终点是否非法（位于障碍物上） ----------------
        const std::pair<int, int> grid_start = {robot_column, robot_row};
        const std::pair<int, int> grid_goal  = {goal_column, goal_row};

        if (!if_silence_)
        {
            const double grid_distance = std::hypot(grid_goal.first - grid_start.first, grid_goal.second - grid_start.second);

            RCLCPP_INFO_STREAM(
                node->get_logger(),
                "start: " << grid_start
                << ", goal: " << grid_goal
                << ", grid_distance: " << grid_distance
            );
        }

        auto cellBlocked = [&](int gx, int gy) -> bool
        {
            const int cost = grid_cost_[gy][gx];
            if (cost < 0)
                return treat_unknown_as_obstacle_;
            return cost >= obstacle_threshold_;
        };

        // 起点非法
        if (cellBlocked(grid_start.first, grid_start.second))
        {
            RCLCPP_WARN(node->get_logger(), "Path planning canceled: start is invalid or occupied.");
            nav_msgs::msg::Path empty_path;
            empty_path.header.frame_id = occupancy_map_msg->header.frame_id;
            empty_path.header.stamp = node->now();
            path_pub_->publish(empty_path);
            return;
        }

        // 终点非法
        float goal_esdf_distance;
        if(!esdf->distance_at_cell(grid_goal.first, grid_goal.second, goal_esdf_distance) || !std::isfinite(goal_esdf_distance))
        {
            RCLCPP_WARN(node->get_logger(), "Failed to query goal ESDF distance.");
            return;
        }
        if (cellBlocked(grid_goal.first, grid_goal.second) || goal_esdf_distance < min_esdf_value_for_goal_)
        {
            RCLCPP_WARN(node->get_logger(), "Path planning canceled: goal is invalid or occupied or too close to obstacles.");

            nav_msgs::msg::Path empty_path;
            empty_path.header.frame_id = occupancy_map_msg->header.frame_id;
            empty_path.header.stamp = node->now();
            path_pub_->publish(empty_path);
            return;
        }

        // ---------------- A* ----------------
        const auto path_int = astar_esdf(
            grid_start,
            grid_goal,
            grid_cost_,
            *esdf,
            obstacle_threshold_,
            treat_unknown_as_obstacle_,
            unknown_cost_penalty_,
            cost_scale_);

        if (path_int.empty())
        {
            RCLCPP_WARN(node->get_logger(), "ESDF A* returned empty path.");
            nav_msgs::msg::Path empty_path;
            empty_path.header.frame_id = occupancy_map_msg->header.frame_id;
            empty_path.header.stamp = node->now();
            path_pub_->publish(empty_path);
            return;
        }
        else
        {
            std::vector<Eigen::Vector2d> eigen_path;
            eigen_path.reserve(path_int.size());
            for (const auto &p : path_int)
            {
                eigen_path.emplace_back(static_cast<double>(p.first), static_cast<double>(p.second));
            }

            std::vector<Eigen::Vector2d> final_origin_path;
            if (if_path_downsampling_)
            {
                // if path-downsampling
                final_origin_path = simplify_path(eigen_path, corner_deg_, corner_dilate_);
            }
            else
            {
                final_origin_path = eigen_path;  
            }

            if (final_origin_path.empty())
            {
                RCLCPP_WARN(node->get_logger(), "Final path is empty after processing.");
                nav_msgs::msg::Path empty_path;
                empty_path.header.frame_id = occupancy_map_msg->header.frame_id;
                empty_path.header.stamp = node->now();
                path_pub_->publish(empty_path);
                return;
            }

            // ---------------- publish path ----------------
            nav_msgs::msg::Path origin_path;
            origin_path.header.frame_id = occupancy_map_msg->header.frame_id;
            origin_path.header.stamp = node->now();
            origin_path.poses.reserve(final_origin_path.size());

            // 生成的路径还包含机器人每一步的朝向
            for (size_t i = 0; i < final_origin_path.size(); ++i)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = origin_path.header;

                pose.pose.position.x = originX + (final_origin_path[i].x() + 0.5) * resolution;;
                pose.pose.position.y = originY + (final_origin_path[i].y() + 0.5) * resolution;
                pose.pose.position.z = 0.0;

                double yaw = 0.0;
                if (final_origin_path.size() >= 2)
                {
                    if (i + 1 < final_origin_path.size())       // 还没到最后一个点
                    {
                        const double dx = final_origin_path[i + 1].x() - final_origin_path[i].x();
                        const double dy = final_origin_path[i + 1].y() - final_origin_path[i].y();
                        yaw = std::atan2(dy, dx);
                    }
                    else
                    {
                        const double dx = final_origin_path[i].x() - final_origin_path[i - 1].x();
                        const double dy = final_origin_path[i].y() - final_origin_path[i - 1].y();
                        yaw = std::atan2(dy, dx);
                    }
                }

                pose.pose.orientation = yawToQuaternion(yaw);
                origin_path.poses.push_back(pose);
            }

            // 替换起点终点为真实世界坐标
            origin_path.poses.front().pose.position.x = robot_x;
            origin_path.poses.front().pose.position.y = robot_y;

            origin_path.poses.back().pose.position.x = goal->x;
            origin_path.poses.back().pose.position.y = goal->y;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);

                if (!goal_ || goal_->generation != goal->generation)
                {
                    // 路径处理期间又收到了新目标，丢弃旧结果
                    return;
                }

                path_pub_->publish(origin_path);

                if (!if_keep_planning_)
                {
                    goal_->planned = true;          // 路径正确计算出并发布才算是规划完成
                }
            }

            const auto time0 = std::chrono::steady_clock::now();
            double cost_ms = std::chrono::duration<double, std::milli>(time0 - start_time).count();

            if (!if_silence_)
            {
                RCLCPP_INFO_STREAM(
                    node->get_logger(),
                    (if_path_downsampling_ ? "A* + downsampling accomplished, cost: " : "A* accomplished, cost: ")
                    << cost_ms << " ms, path_size=" << origin_path.poses.size()
                );
            }
        }
    }

    // ======================================== A* ========================================
    // 返回格子索引
    inline int toIndex(int x, int y, int cols)
    {
        return y * cols + x;
    }

    // 格子索引是否合法
    inline bool inBounds(int x, int y, int cols, int rows)
    {
        return (x >= 0 && x < cols && y >= 0 && y < rows);
    }

    // 返回要走的总距离
    inline double octileHeuristic(int x1, int y1, int x2, int y2)
    {
        const double dx = std::abs(x1 - x2);
        const double dy = std::abs(y1 - y2);
        const double diag = std::min(dx, dy);       // 对角线走的步数
        const double straight = dx + dy - 2.0 * diag;       // 直线要走的距离（曼哈顿距离减去对角线走的距离）
        return std::sqrt(2.0) * diag + straight;
    }

    inline bool isBlocked(int cell_cost, int obstacle_threshold, bool treat_unknown_as_obstacle)
    {
        if (cell_cost < 0)
            return treat_unknown_as_obstacle;
        return cell_cost >= obstacle_threshold;
    }

    // 计算总代价权重
    inline double getTraversalWeight(
        int cell_cost,
        bool treat_unknown_as_obstacle,
        double unknown_cost_penalty,
        double cost_scale)
    {
        // 若 unknown 可通行，则赋予较高惩罚
        const double effective_cost =
            (cell_cost < 0) ? (treat_unknown_as_obstacle ? 1e9 : unknown_cost_penalty) : static_cast<double>(cell_cost);

        // 线性缩放：1.0 + cost_scale * normalized_cost
        // normalized_cost roughly in [0, 1] if cost is [0, 100]
        return 1.0 + cost_scale * (effective_cost / 100.0);         // 这里是基础代价，因此要加 1
    }

    std::vector<std::pair<int, int>> AstarEsdfGlobalPlanning::astar_esdf(
                const std::pair<int, int>& start,
                const std::pair<int, int>& goal,
                const std::vector<std::vector<int>>& grid,
                const field_map_builder::EsdfMapSnapshot& esdf,
                int obstacle_threshold,
                bool treat_unknown_as_obstacle,
                double unknown_cost_penalty,
                double cost_scale)
    {
        if (grid.empty() || grid[0].empty())
            return {};

        auto node = node_.lock();
        if (!node)
            throw std::runtime_error("Parent node expired.");

        const int rows = static_cast<int>(grid.size());
        const int cols = static_cast<int>(grid[0].size());

        if (!inBounds(start.first, start.second, cols, rows) || !inBounds(goal.first, goal.second, cols, rows))
            return {};

        if (isBlocked(grid[start.second][start.first], obstacle_threshold, treat_unknown_as_obstacle) ||
            isBlocked(grid[goal.second][goal.first], obstacle_threshold, treat_unknown_as_obstacle))
            return {};

        const int total_size = rows * cols;
        const double INF = std::numeric_limits<double>::infinity();

        std::vector<double> g_score(total_size, INF);       // 存储起点到这个格子的实际代价
        std::vector<int> parent(total_size, -1);        // 回溯用；全部初始化为-1
        std::vector<uint8_t> closed(total_size, 0);     // 到过的格子；0表示没有去过

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;

        const int start_idx = toIndex(start.first, start.second, cols);
        const int goal_idx = toIndex(goal.first, goal.second, cols);

        g_score[start_idx] = 0.0;
        const double h0 = octileHeuristic(start.first, start.second, goal.first, goal.second);
        open_set.push(Node{start.first, start.second, h0, h0});

        static const std::vector<std::pair<int, int>> dirs = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {-1, 1}, {-1, -1}, {1, -1}
        };

        while (!open_set.empty())
        {
            const Node current = open_set.top();
            open_set.pop();

            const int cur_idx = toIndex(current.x, current.y, cols);
            if (closed[cur_idx])        // 到过这个格子
                continue;
            closed[cur_idx] = 1;

            if (cur_idx == goal_idx)
            {
                std::vector<std::pair<int, int>> path;
                int trace = goal_idx;
                while (trace != -1)
                {
                    const int x = trace % cols;
                    const int y = trace / cols;
                    path.emplace_back(x, y);
                    trace = parent[trace];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto& d : dirs)
            {
                const int nx = current.x + d.first;
                const int ny = current.y + d.second;

                if (!inBounds(nx, ny, cols, rows))
                    continue;

                const int neighbor_cost = grid[ny][nx];
                if (isBlocked(neighbor_cost, obstacle_threshold, treat_unknown_as_obstacle))
                    continue;

                // 防止对角穿墙角
                if (d.first != 0 && d.second != 0)
                {
                    const int ax = current.x + d.first;
                    const int ay = current.y;
                    const int bx = current.x;
                    const int by = current.y + d.second;

                    if (!inBounds(ax, ay, cols, rows) || !inBounds(bx, by, cols, rows))
                        continue;

                    if (isBlocked(grid[ay][ax], obstacle_threshold, treat_unknown_as_obstacle) ||
                        isBlocked(grid[by][bx], obstacle_threshold, treat_unknown_as_obstacle))
                    {
                        continue;
                    }
                }

                float esdf_dist;
                if(!esdf.distance_at_cell(nx, ny, esdf_dist))      // ESDF 距离
                {
                    RCLCPP_WARN(node->get_logger(), "Failed to query goal ESDF distance.");
                    continue ;
                }

                const int neighbor_idx = toIndex(nx, ny, cols);
                if (closed[neighbor_idx])
                    continue;

                const double move_cost =
                    (d.first != 0 && d.second != 0) ? std::sqrt(2.0) : 1.0;

                const double base_weight = getTraversalWeight(
                    neighbor_cost,
                    treat_unknown_as_obstacle,
                    unknown_cost_penalty,
                    cost_scale);

                double clearance_penalty = 0.0;
                if (use_esdf_soft_cost_)
                {
                    clearance_penalty = compute_esdf_clearance_penalty(static_cast<double>(esdf_dist));          // ESDF 代价
                }

                // 下一步代价的总权重
                const double step_weight = base_weight + esdf_cost_scale_ * clearance_penalty;

                // 邻居格子的总代价
                const double tentative_g = g_score[cur_idx] + move_cost * step_weight;

                if (tentative_g < g_score[neighbor_idx])
                {
                    g_score[neighbor_idx] = tentative_g;
                    parent[neighbor_idx] = cur_idx;

                    const double h = octileHeuristic(nx, ny, goal.first, goal.second);
                    // const double f = tentative_g + h * heuristic_tie_breaker;
                    const double f = tentative_g + h;           // 比较运算符中已经会在 f 相同时用较小的 h 做第二排序

                    open_set.push(Node{nx, ny, f, h});
                }
            }
        }

        return {};
    }

    double AstarEsdfGlobalPlanning::compute_esdf_clearance_penalty(double esdf_dist) const
    {
        if(safe_clearance_ <= 1e-6)
            return 0.0;
        if(esdf_dist >= safe_clearance_)
        {
            return 0.0;
        }
        else
        {
            const double ratio = (safe_clearance_ - esdf_dist) / safe_clearance_;
            return ratio * ratio;
        }
    }

    void AstarEsdfGlobalPlanning::deactivate()
    {
        auto node = node_.lock();

        // 先挡住所有 callback
        lifecycle_active_.store(false, std::memory_order_release);

        odom_sub_.reset();
        occupancy_map_sub_.reset();
        esdf_sub_.reset();
        goal_sub_.reset();

        // 停止 timers
        if (planning_timer)
        {
            planning_timer->cancel();
            planning_timer.reset();
        }

        // 等待正在进行的 planningTimerCallback 退出
        {
            std::lock_guard<std::mutex> lock(planning_mutex_);
        }

        // deactivate publishers
        if (path_pub_)
            path_pub_->on_deactivate();

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "AstarEsdfGlobalPlanning plugin '%s' deactivated.",
                plugin_name_.c_str());
        }
    }

    void AstarEsdfGlobalPlanning::cleanup()
    {
        auto node = node_.lock();

        lifecycle_active_.store(false);

        odom_sub_.reset();
        occupancy_map_sub_.reset();
        esdf_sub_.reset();
        goal_sub_.reset();

        if (planning_timer)
        {
            planning_timer->cancel();
            planning_timer.reset();
        }

        // configure 阶段创建的 publishers 全部 reset
        path_pub_.reset();

        reset_runtime_state();

        if (node)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "AstarEsdfGlobalPlanning plugin '%s' cleaned up.",
                plugin_name_.c_str());
        }

        plugin_name_.clear();

        // 最后一件事再丢掉 node weak reference
        node_.reset();
    }

    // ============================== helpers ==============================
    geometry_msgs::msg::Quaternion AstarEsdfGlobalPlanning::yawToQuaternion(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.w = std::cos(yaw * 0.5);
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        return q;
    }

}       // namespace global_planner

PLUGINLIB_EXPORT_CLASS(global_planner::AstarEsdfGlobalPlanning, global_planner::GlobalPlannerBase)
