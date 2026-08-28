#ifndef POINTCLOUD_CORRECTOR__POINTCLOUD_ALIGNING_HPP_
#define POINTCLOUD_CORRECTOR__POINTCLOUD_ALIGNING_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Vector3.h"

#include "pointcloud_corrector/pointcloud_corrector_base.hpp"

namespace pointcloud_corrector
{

    class PointCloudAligner : public CorrectedPointCloudPublisherBase
    {
        public:
            PointCloudAligner() = default;
            ~PointCloudAligner() override = default;

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

            bool loadTranslationExtrinsic();

            bool hasField(const sensor_msgs::msg::PointCloud2 & cloud_in, const std::string & field_name) const;

            bool hasXYZfields(const sensor_msgs::msg::PointCloud2 & cloud_in) const;

            pcl::PointCloud<pcl::PointXYZI>::Ptr transformAndAddToBuffer(const sensor_msgs::msg::PointCloud2 & cloud_in);

            void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        private:
            LifecycleNodeWeakPtr node_;
            std::string plugin_name_;

            std::atomic_bool active_{false};            // 存储插件状态(active/inactive)

            // Parameters
            bool if_silence_{false};
            bool if_debug_{true};
            bool if_publish_single_frame_{true};

            std::string input_cloud_topic_;
            std::string output_cloud_topic_;
            std::string output_submap_cloud_topic_;
            std::string output_single_topic_;
            std::string input_odom_topic_;
            std::string output_frame_;

            bool enable_translation_extrinsic_{true};
            std::vector<double> extrinsic_xyz_;         // 外参参数
            bool has_translation_extrinsic_{false};
            tf2::Vector3 t_extrinsic_;                  // 存储外参

            int sliding_window_size_{15};

            double crop_range_x_{5.0};
            double crop_range_y_{5.0};
            double crop_min_z_{-1.5};
            double crop_max_z_{2.5};

            double submap_crop_x_{3.0};
            double submap_crop_y_{3.0};
            double submap_crop_min_z_{0.05};
            double submap_crop_max_z_{2.0};

            // Subscriptions
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

            // Lifecycle publishers
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;              // 多帧融合点云发布
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr single_cloud_pub_;           // 单帧点云发布
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_points_pub_;          // 局部障碍物发布

            // Runtime state
            std::mutex odom_mutex_;
            nav_msgs::msg::Odometry::SharedPtr latest_odom_;

            std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_buffer_;             // 存储滑动窗口的多帧点云
    };

}  // namespace pointcloud_corrector

#endif  // POINTCLOUD_CORRECTOR__POINTCLOUD_ALIGNING_HPP_