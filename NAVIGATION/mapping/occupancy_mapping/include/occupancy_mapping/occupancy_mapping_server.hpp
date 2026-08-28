#ifndef OCCUPANCY_MAPPING__OCCUPANCY_MAPPING_SERVER_HPP_
#define OCCUPANCY_MAPPING__OCCUPANCY_MAPPING_SERVER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "occupancy_mapping/occupancy_mapping_base.hpp"

namespace occupancy_mapping
{
    using PluginPtr = std::shared_ptr<OccupancyMappingBase>;

    class OccupancyMappingServer : public rclcpp_lifecycle::LifecycleNode
    {
        public:
            explicit OccupancyMappingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        protected:
            using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

            CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
            CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

        private:
            bool deactivate_plugins_best_effort(
                const std::vector<std::string> & plugin_ids,
                const char * context);

            bool cleanup_plugins_best_effort(const char * context);

            void clear_plugin_resources();

            template<typename T>
            T declare_or_get_parameter(const std::string & name, const T & default_value)
            {
                if (!this->has_parameter(name))                 // 没有这个参数就先声明
                {
                    return this->declare_parameter<T>(name, default_value);
                }

                return this->get_parameter(name).get_value<T>();
            }

        private:
            std::shared_ptr<pluginlib::ClassLoader<OccupancyMappingBase>> plugin_loader_;

            // Server 类持有（多个）插件指针（强引用）
            /*
            *   Server
            *    │
            *    │ owns(shared_ptr)
            *    ▼
            *   OrdinaryOccupancyMapping
            */
            std::unordered_map<std::string, PluginPtr> plugin_instances_;

            // 插件实例名
            std::vector<std::string> occupancy_mapping_plugin_ids_;

            // 已成功完成 activate() 的插件实例名，顺序与 YAML 一致
            std::vector<std::string> active_plugin_ids_;
    };

}  // namespace occupancy_mapping

#endif  // OCCUPANCY_MAPPING__OCCUPANCY_MAPPING_SERVER_HPP_