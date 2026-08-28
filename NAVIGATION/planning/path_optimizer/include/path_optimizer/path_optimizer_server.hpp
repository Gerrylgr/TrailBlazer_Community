#ifndef PATH_OPTIMIZER__PATH_OPTIMIZER_SERVER_HPP_
#define PATH_OPTIMIZER__PATH_OPTIMIZER_SERVER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "path_optimizer/path_optimizer_base.hpp"

namespace path_optimizer
{
    using PluginPtr = std::shared_ptr<PathOptimizerBase>;

    class PathOptimizerServer : public rclcpp_lifecycle::LifecycleNode
    {
        public:
            explicit PathOptimizerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
            std::shared_ptr<pluginlib::ClassLoader<PathOptimizerBase>> plugin_loader_;

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
            std::vector<std::string> path_optimizer_plugin_ids_;

            // 已成功完成 activate() 的插件实例名，顺序与 YAML 一致
            std::vector<std::string> active_plugin_ids_;
    };

}  // namespace path_optimizer

#endif  // PATH_OPTIMIZER__PATH_OPTIMIZER_SERVER_HPP_