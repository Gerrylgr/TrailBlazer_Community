/*
* lifecycle 流程：
*   configure：
*       load_parameters
*       create_publishers
*       reset_runtime_state
*
*   activate：
*       (activate publishers)
*       create_subscriptions
*
*   deactivate：
*        (reset subscriptions)
*        (deactivate publishers)
*   cleanup：
*       (reset all pubs/subs)
*       (reset node/clear plugin name)
*/
/*
* 点云转换模块：
*       订阅 SLAM 模块的稠密建图点云，处理后发布：
*           /points_cropped：裁剪后的、多帧融合的滑动窗口合并点云（用于地面分割，可解决16线/32线雷达点云稀疏导致地面分割困难的问题）
*           /submap_points_cropped：经过粗糙裁剪的局部障碍物点云（可用于实时避障）
*           /points_cropped_single：单帧的裁剪后的点云（用于动态障碍物检测）
*/
#include "pointcloud_corrector/pointcloud_aligning.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <chrono>

namespace pointcloud_corrector
{
    void PointCloudAligner::configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name)
    {
        node_ = parent;                 // server 把自己的节点传给 PointCloudAligner，让其能够打印日志等
        plugin_name_ = plugin_name;

        auto node = node_.lock();
        if (!node) 
        {
            throw std::runtime_error("PointCloudAligner::configure() failed: parent node is expired.");
        }

        load_parameters(node);
        create_publishers(node);
        reset_runtime_state();

        RCLCPP_INFO(
            node->get_logger(),
            "PointCloudAligner plugin '%s' configured.",
            plugin_name_.c_str()
        );
    }

    void PointCloudAligner::load_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        const std::string p = plugin_name_ + ".";

        if_silence_ = declare_or_get_parameter<bool>(node, p + "if_silence", false);
        if_debug_ = declare_or_get_parameter<bool>(node, p + "if_debug", true);

        if_publish_single_frame_ = declare_or_get_parameter<bool>(node, p + "if_publish_single_frame", true);

        input_cloud_topic_ = declare_or_get_parameter<std::string>(node, p + "input_cloud_topic", "/cloud_registered");
        output_cloud_topic_ = declare_or_get_parameter<std::string>(node, p + "output_cloud_topic", "/points_cropped");
        output_submap_cloud_topic_ = declare_or_get_parameter<std::string>(node, p + "output_submap_cloud_topic", "/submap_points_cropped");
        output_single_topic_ = declare_or_get_parameter<std::string>(node, p + "output_single_topic", "/points_cropped_single");
        input_odom_topic_ = declare_or_get_parameter<std::string>(node, p + "input_odom_topic", "/Odometry");

        output_frame_ = declare_or_get_parameter<std::string>(node, p + "output_frame", "map");

        enable_translation_extrinsic_ = declare_or_get_parameter<bool>(node, p + "enable_translation_extrinsic", true);
        extrinsic_xyz_ = declare_or_get_parameter<std::vector<double>>(
            node,
            p + "extrinsic_xyz",
            std::vector<double>{0, 0, 0.1}
        );

        has_translation_extrinsic_ = false;
        if (!loadTranslationExtrinsic()) 
        {
            if (enable_translation_extrinsic_) 
            {
                throw std::runtime_error("Failed to load translation extrinsic, but enable_translation_extrinsic=true.");
            }

            RCLCPP_WARN(
                node->get_logger(),
                "Failed to load translation extrinsic. It will be ignored because enable_translation_extrinsic=false."
            );
        }

        sliding_window_size_ = declare_or_get_parameter<int>(node, p + "sliding_window_size", 3);

        if (sliding_window_size_ < 1) 
        {
            RCLCPP_WARN(
                node->get_logger(),
                "sliding_window_size must be >= 1. Reset to 1."
            );
            sliding_window_size_ = 1;
        }

        crop_range_x_ = declare_or_get_parameter<double>(node, p + "crop_range_x", 7.0);
        crop_range_y_ = declare_or_get_parameter<double>(node, p + "crop_range_y", 7.0);
        crop_min_z_ = declare_or_get_parameter<double>(node, p + "crop_min_z", -2.0);
        crop_max_z_ = declare_or_get_parameter<double>(node, p + "crop_max_z", 2.5);

        submap_crop_x_ = declare_or_get_parameter<double>(node, p + "submap_crop_x", 4.0);
        submap_crop_y_ = declare_or_get_parameter<double>(node, p + "submap_crop_y", 4.0);
        submap_crop_min_z_ = declare_or_get_parameter<double>(node, p + "submap_crop_min_z", 0.03);
        submap_crop_max_z_ = declare_or_get_parameter<double>(node, p + "submap_crop_max_z", 2.0);

        RCLCPP_INFO_STREAM(
            node->get_logger(),
            "PointCloudAligner parameters loaded."
            << " plugin_name=" << plugin_name_
            << ", input_cloud_topic=" << input_cloud_topic_
            << ", input_odom_topic=" << input_odom_topic_
            << ", output_cloud_topic=" << output_cloud_topic_
            << ", output_frame=" << output_frame_
            << ", sliding_window_size=" << sliding_window_size_
            << ", crop_range_x_half=" << crop_range_x_
            << ", crop_range_y_half=" << crop_range_y_
            << ", crop_min_z_rel=" << crop_min_z_
            << ", crop_max_z_rel=" << crop_max_z_
            << ", enable_translation_extrinsic="
            << (enable_translation_extrinsic_ ? "true" : "false")
        );
    }

    bool PointCloudAligner::loadTranslationExtrinsic()
    {
        if (extrinsic_xyz_.size() != 3)
            return false;
        
        t_extrinsic_.setValue(extrinsic_xyz_[0], extrinsic_xyz_[1], extrinsic_xyz_[2]);

        has_translation_extrinsic_ = true;
        return true;
    }

    void PointCloudAligner::create_publishers(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, rclcpp::SensorDataQoS());
        single_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(output_single_topic_, rclcpp::SensorDataQoS());
        submap_points_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(output_submap_cloud_topic_, rclcpp::SensorDataQoS());
    }

    void PointCloudAligner::reset_runtime_state()
    {
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            latest_odom_.reset();
        }

        cloud_buffer_.clear();
    }

    void PointCloudAligner::activate()
    {
        auto node = node_.lock();
        if (!node) 
        {
            throw std::runtime_error("PointCloudAligner::activate() failed: parent node is expired.");
        }

        // 激活发布者
        if (cloud_pub_) 
            cloud_pub_->on_activate();
        if (single_cloud_pub_) 
            single_cloud_pub_->on_activate();
        if (submap_points_pub_) 
            submap_points_pub_->on_activate();
        
        create_subscriptions(node);

        active_.store(true);

        RCLCPP_INFO(
            node->get_logger(),
            "PointCloudAligner plugin '%s' activated.",
            plugin_name_.c_str()
        );
    }

    void PointCloudAligner::create_subscriptions(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    {
        cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                        input_cloud_topic_,
                        rclcpp::SensorDataQoS(),
                        std::bind(&PointCloudAligner::pointcloud_callback, this, std::placeholders::_1)
                        );

        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
                    input_odom_topic_,
                    rclcpp::SensorDataQoS(),
                    std::bind(&PointCloudAligner::odom_callback, this, std::placeholders::_1)
                    );
    }

    void PointCloudAligner::deactivate()
    {
        auto node = node_.lock();

        active_.store(false);

        // 重置 subscription，停止处理高频点云
        cloud_sub_.reset();
        odom_sub_.reset();

        if (cloud_pub_) 
            cloud_pub_->on_deactivate();
        if (single_cloud_pub_) 
            single_cloud_pub_->on_deactivate();
        if (submap_points_pub_) 
            submap_points_pub_->on_deactivate();

        if (node) 
        {
            RCLCPP_INFO(
                node->get_logger(),
                "PointCloudAligner plugin '%s' deactivated.",
                plugin_name_.c_str()
            );
        }
    }

    void PointCloudAligner::cleanup()
    {
        active_.store(false);

        cloud_sub_.reset();
        odom_sub_.reset();

        cloud_pub_.reset();
        single_cloud_pub_.reset();
        submap_points_pub_.reset();

        reset_runtime_state();

        node_.reset();
        plugin_name_.clear();
    }

    void PointCloudAligner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!active_.load())                    // 只在插件 active 状态时执行回调
            return;

        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = msg;
    }

    void PointCloudAligner::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!active_.load()) 
            return;

        auto node = node_.lock();           // 需要转为 shared_ptr 才能使用 node
        if (!node) 
            return;

        const auto start_time = std::chrono::steady_clock::now();

        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_z = 0.0;

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);

            if (!latest_odom_)
            {
                RCLCPP_WARN_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    1000,
                    "Odometry msg is not ready yet. Skip current cloud."
                );
                return;
            }

            robot_x = latest_odom_->pose.pose.position.x;
            robot_y = latest_odom_->pose.pose.position.y;
            robot_z = latest_odom_->pose.pose.position.z;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr single_cloud = transformAndAddToBuffer(*msg);     // 单帧变换
        if (!single_cloud) 
        {
            return;
        }

        std::size_t total_points = 0;
        for (const auto & cloud : cloud_buffer_)
        {
            total_points += cloud->size();
        }

        pcl::PointCloud<pcl::PointXYZI> cloud_merged;
        cloud_merged.reserve(total_points);

        for (const auto & cloud : cloud_buffer_)
        {
            cloud_merged += *cloud;         // 获取合并后的点云     
        }

        if (cloud_merged.empty())
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "Merged cloud is empty. Skip publishing."
            );
            return;
        }

        // 裁剪点云：x/y 以机器人当前位置为中心，z 使用相对机器人高度。
        auto crop_cloud = [&](const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input_cloud, 
                                const double &crop_range_x, const double &crop_range_y,
                                const double &crop_min_z, const double &crop_max_z)
        {
            pcl::CropBox<pcl::PointXYZI> crop_box;
            crop_box.setInputCloud(input_cloud);

            crop_box.setMin(Eigen::Vector4f(
                static_cast<float>(robot_x - crop_range_x),
                static_cast<float>(robot_y - crop_range_y),
                static_cast<float>(robot_z + crop_min_z),
                1.0f));

            crop_box.setMax(Eigen::Vector4f(
                static_cast<float>(robot_x + crop_range_x),
                static_cast<float>(robot_y + crop_range_y),
                static_cast<float>(robot_z + crop_max_z),
                1.0f));

            crop_box.setNegative(false);

            pcl::PointCloud<pcl::PointXYZI> output_cloud;
            crop_box.filter(output_cloud);
            return output_cloud;
        };

        // 裁剪后的多帧点云
        pcl::PointCloud<pcl::PointXYZI> cropped_cloud = crop_cloud(cloud_merged.makeShared(), crop_range_x_, crop_range_y_, crop_min_z_, crop_max_z_);

        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::toROSMsg(cropped_cloud, cloud_out);
        cloud_out.header.frame_id = output_frame_;
        cloud_out.header.stamp = msg->header.stamp;

        cloud_pub_->publish(cloud_out);

        if(if_publish_single_frame_)
        {
            // 裁剪单帧点云
            pcl::PointCloud<pcl::PointXYZI> cropped_single_cloud = crop_cloud(single_cloud, crop_range_x_, crop_range_y_, crop_min_z_, crop_max_z_);

            pcl::toROSMsg(cropped_single_cloud, cloud_out);
            cloud_out.header.frame_id = output_frame_;
            cloud_out.header.stamp = msg->header.stamp;

            single_cloud_pub_->publish(cloud_out);

            // 裁剪子地图点云
            pcl::PointCloud<pcl::PointXYZI> submap_cloud = crop_cloud(single_cloud, submap_crop_x_, submap_crop_y_, submap_crop_min_z_, submap_crop_max_z_);
            pcl::toROSMsg(submap_cloud, cloud_out);
            cloud_out.header.frame_id = output_frame_;
            cloud_out.header.stamp = msg->header.stamp;

            submap_points_pub_->publish(cloud_out);
        }

        if (!if_silence_)
        {
            RCLCPP_INFO_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                2000,
                "Published cropped cloud. Buffer frames=%zu, merged points=%zu, cropped points=%zu",
                cloud_buffer_.size(),
                cloud_merged.size(),
                cropped_cloud.size()
            );
        }

        if (if_debug_)
        {
            const auto end_time = std::chrono::steady_clock::now();
            const double cost_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            RCLCPP_INFO(
                node->get_logger(),
                "Pointcloud cropper process cost: %.2f ms",
                cost_ms
            );
        }
    }

    /*
    * 检查node/点云/外参等合法性
    * 对单帧点云做外参变换，并推入 cloud_buffer_
    * 返回单帧的变换后点云 pcl_transformed
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudAligner::transformAndAddToBuffer(const sensor_msgs::msg::PointCloud2 & cloud_in)
    {
        auto node = node_.lock();
        if (!node) 
            return nullptr;

        if (!hasXYZfields(cloud_in))
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Input pointcloud does not have x/y/z fields."
            );
            return nullptr;
        }

        if (enable_translation_extrinsic_ && !has_translation_extrinsic_)
        {
            RCLCPP_ERROR_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "Translation extrinsic is enabled but not loaded. Skip current cloud."
            );
            return nullptr;
        }

        pcl::PointCloud<pcl::PointXYZI> pcl_input;
        pcl::fromROSMsg(cloud_in, pcl_input);

        if (pcl_input.empty())
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "Input pointcloud is empty after fromROSMsg."
            );
            return nullptr;
        }

        const bool has_intensity = hasField(cloud_in, "intensity");

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl_transformed->reserve(pcl_input.size());

        for (const auto & p_in : pcl_input.points)
        {
            if (!std::isfinite(p_in.x) || !std::isfinite(p_in.y) || !std::isfinite(p_in.z))
                continue;
            
            pcl::PointXYZI p_out;
            p_out.x = p_in.x;
            p_out.y = p_in.y;
            p_out.z = p_in.z;

            if (has_intensity && std::isfinite(p_in.intensity))
            {
                p_out.intensity = p_in.intensity;
            }
            else
            {
                p_out.intensity = 0.0f;
            }

            if (enable_translation_extrinsic_)
            {
                p_out.x += static_cast<float>(t_extrinsic_.x());
                p_out.y += static_cast<float>(t_extrinsic_.y());
                p_out.z += static_cast<float>(t_extrinsic_.z());
            }

            pcl_transformed->push_back(p_out);
        }

        if (pcl_transformed->empty())
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                1000,
                "All points are invalid after finite check.");
            return nullptr;
        }

        cloud_buffer_.push_back(pcl_transformed);

        while (cloud_buffer_.size() > static_cast<std::size_t>(sliding_window_size_))
        {
            cloud_buffer_.pop_front();
        }

        return pcl_transformed;
    }

    bool PointCloudAligner::hasField(const sensor_msgs::msg::PointCloud2 & cloud_in, const std::string & field_name) const
    {
        for (const auto & field : cloud_in.fields)
        {
            if (field.name == field_name)
            {
                return true;
            }
        }
        return false;
    }

    bool PointCloudAligner::hasXYZfields(const sensor_msgs::msg::PointCloud2 & cloud_in) const
    {
        return hasField(cloud_in, "x") && hasField(cloud_in, "y") && hasField(cloud_in, "z");
    }

}  // namespace pointcloud_corrector

PLUGINLIB_EXPORT_CLASS(pointcloud_corrector::PointCloudAligner, pointcloud_corrector::CorrectedPointCloudPublisherBase)