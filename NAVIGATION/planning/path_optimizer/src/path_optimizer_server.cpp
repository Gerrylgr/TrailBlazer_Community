/*
 * Server 节点：按照 YAML 顺序管理 path_optimizer 插件的生命周期。
 */
#include "path_optimizer/path_optimizer_server.hpp"

#include <exception>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace path_optimizer
{

    PathOptimizerServer::PathOptimizerServer(const rclcpp::NodeOptions & options)
        : rclcpp_lifecycle::LifecycleNode("path_optimizer_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "PathOptimizerServer constructed.");
    }

    bool PathOptimizerServer::deactivate_plugins_best_effort(const std::vector<std::string> & plugin_ids, const char * context)
    {
        bool all_succeeded = true;

        for (auto it = plugin_ids.rbegin(); it != plugin_ids.rend(); ++it)
        {
            const auto plugin_it = plugin_instances_.find(*it);
            if (plugin_it == plugin_instances_.end() || !plugin_it->second)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Cannot deactivate plugin '%s' during %s: instance not found.",
                    it->c_str(),
                    context);
                all_succeeded = false;
                continue;
            }

            try
            {
                plugin_it->second->deactivate();
            }
            catch (const std::exception & ex)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to deactivate plugin '%s' during %s: %s",
                    it->c_str(),
                    context,
                    ex.what());
                all_succeeded = false;
            }
            catch (...)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to deactivate plugin '%s' during %s: unknown exception.",
                    it->c_str(),
                    context);
                all_succeeded = false;
            }
        }

        return all_succeeded;
    }

    bool PathOptimizerServer::cleanup_plugins_best_effort(const char * context)
    {
        bool all_succeeded = true;

        for (auto it = path_optimizer_plugin_ids_.rbegin(); it != path_optimizer_plugin_ids_.rend(); ++it)
        {
            const auto plugin_it = plugin_instances_.find(*it);
            if (plugin_it == plugin_instances_.end() || !plugin_it->second)
            {
                continue;
            }

            try
            {
                plugin_it->second->cleanup();
            }
            catch (const std::exception & ex)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to clean up plugin '%s' during %s: %s",
                    it->c_str(),
                    context,
                    ex.what());
                all_succeeded = false;
            }
            catch (...)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to clean up plugin '%s' during %s: unknown exception.",
                    it->c_str(),
                    context);
                all_succeeded = false;
            }
        }

        return all_succeeded;
    }

    void PathOptimizerServer::clear_plugin_resources()
    {
        // 必须先销毁插件实例，再销毁负责卸载动态库的 ClassLoader。
        plugin_instances_.clear();
        plugin_loader_.reset();
        active_plugin_ids_.clear();
        path_optimizer_plugin_ids_.clear();
    }

    PathOptimizerServer::CallbackReturn PathOptimizerServer::on_configure(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Configuring PathOptimizerServer...");

        // 正常 lifecycle 流程下这里本来就应为空。该保护用于处理此前失败后遗留的状态。
        if (!plugin_instances_.empty() || plugin_loader_)
        {
            RCLCPP_WARN(get_logger(), "Stale plugin resources found before configure; clearing them.");
            deactivate_plugins_best_effort(path_optimizer_plugin_ids_, "pre-configure recovery");
            cleanup_plugins_best_effort("pre-configure recovery");
            clear_plugin_resources();
        }

        try
        {
            path_optimizer_plugin_ids_ = declare_or_get_parameter<std::vector<std::string>>("path_optimizer_plugins", {"LbfgsBsplineOptimizer"});

            if (path_optimizer_plugin_ids_.empty())
            {
                throw std::runtime_error("Parameter 'path_optimizer_plugins' must not be empty.");
            }

            // 在创建任何插件之前完成 ID 校验，避免加载一半后才发现重复 ID。
            std::unordered_set<std::string> unique_ids;
            unique_ids.reserve(path_optimizer_plugin_ids_.size());

            for (const auto & plugin_id : path_optimizer_plugin_ids_)
            {
                if (plugin_id.empty())
                {
                    throw std::runtime_error("Plugin ID in 'path_optimizer_plugins' must not be empty.");
                }

                if (!unique_ids.insert(plugin_id).second)
                {
                    throw std::runtime_error("Duplicate plugin ID declared: '" + plugin_id + "'.");
                }
            }

            plugin_loader_ = std::make_shared<pluginlib::ClassLoader<PathOptimizerBase>>("path_optimizer", "path_optimizer::PathOptimizerBase");

            for (const auto & plugin_id : path_optimizer_plugin_ids_)
            {
                const std::string plugin_type = declare_or_get_parameter<std::string>(plugin_id + ".plugin", "path_optimizer::LbfgsBsplineOptimizing");

                RCLCPP_INFO(
                    get_logger(),
                    "Loading plugin. id='%s', type='%s'",
                    plugin_id.c_str(),
                    plugin_type.c_str());

                auto plugin = plugin_loader_->createSharedInstance(plugin_type);        // 子类实例化

                try
                {
                    /*
                    * 此处：（不会增加引用计数，不会造成循环引用）
                    * Plugin
                    *  │
                    *  │ weak references
                    *  ▼
                    * Server
                    */
                    plugin->configure(this->weak_from_this(), plugin_id);       // Server 将自己的 weak_ptr 传给插件使用（直接通过函数传递）
                }
                catch (...)
                {
                    // configure() 失败的当前实例还没有放入 map，单独尽力清理。
                    try
                    {
                        plugin->cleanup();
                    }
                    catch (...)
                    {
                    }
                    throw;
                }

                plugin_instances_.emplace(plugin_id, std::move(plugin));
            }

            RCLCPP_INFO(get_logger(), "Path optimizer plugins configured successfully.");
            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception & ex)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Exception while configuring path optimizer plugins: %s",
                ex.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Unknown exception while configuring path optimizer plugins.");
        }

        cleanup_plugins_best_effort("configure rollback");
        clear_plugin_resources();
        return CallbackReturn::FAILURE;
    }

    PathOptimizerServer::CallbackReturn PathOptimizerServer::on_activate(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Activating PathOptimizerServer...");

        if (plugin_instances_.empty())
        {
            RCLCPP_ERROR(get_logger(), "Cannot activate: no configured plugin instances.");
            return CallbackReturn::FAILURE;
        }

        if (!active_plugin_ids_.empty())
        {
            // 尝试恢复
            if (!deactivate_plugins_best_effort(active_plugin_ids_, "pre-activate recovery"))
            {
                return CallbackReturn::FAILURE;
            }
        }

        active_plugin_ids_.clear();

        for (const auto & plugin_id : path_optimizer_plugin_ids_)
        {
            const auto plugin_it = plugin_instances_.find(plugin_id);
            if (plugin_it == plugin_instances_.end() || !plugin_it->second)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Cannot activate plugin '%s': instance not found.",
                    plugin_id.c_str());

                deactivate_plugins_best_effort(active_plugin_ids_, "activate rollback");
                active_plugin_ids_.clear();
                return CallbackReturn::FAILURE;
            }

            try
            {
                active_plugin_ids_.push_back(plugin_id);
                try
                {
                    plugin_it->second->activate();
                    RCLCPP_INFO(get_logger(), "Activated plugin '%s'.", plugin_id.c_str());
                }
                catch (const std::exception & ex)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to activate plugin '%s': %s",
                        plugin_id.c_str(),
                        ex.what());

                    const bool rollback_succeeded = deactivate_plugins_best_effort(active_plugin_ids_, "activate rollback");

                    if (rollback_succeeded)
                    {
                        active_plugin_ids_.clear();
                    }
                    // 回滚失败则保留列表，下一次 activate/shutdown 还能重试

                    return CallbackReturn::FAILURE;
                }
                catch (...)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to activate plugin '%s': unknown exception.",
                        plugin_id.c_str());

                    const bool rollback_succeeded = deactivate_plugins_best_effort(active_plugin_ids_, "activate rollback");

                    if (rollback_succeeded)
                    {
                        active_plugin_ids_.clear();
                    }

                    return CallbackReturn::FAILURE;
                }

            }
            catch (const std::exception & ex)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to activate plugin '%s': %s",
                    plugin_id.c_str(),
                    ex.what());

                // 当前插件按接口契约应已自行回滚；此调用再提供一次幂等兜底。
                try
                {
                    plugin_it->second->deactivate();
                }
                catch (...)
                {
                }

                deactivate_plugins_best_effort(active_plugin_ids_, "activate rollback");
                active_plugin_ids_.clear();
                return CallbackReturn::FAILURE;
            }
            catch (...)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to activate plugin '%s': unknown exception.",
                    plugin_id.c_str());

                try
                {
                    plugin_it->second->deactivate();
                }
                catch (...)
                {
                }

                deactivate_plugins_best_effort(active_plugin_ids_, "activate rollback");
                active_plugin_ids_.clear();
                return CallbackReturn::FAILURE;
            }
        }

        return CallbackReturn::SUCCESS;
    }

    PathOptimizerServer::CallbackReturn PathOptimizerServer::on_deactivate(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Deactivating PathOptimizerServer...");

        const bool all_succeeded = deactivate_plugins_best_effort(active_plugin_ids_, "deactivate transition");

        if (!all_succeeded)
        {
            return CallbackReturn::FAILURE;
        }

        active_plugin_ids_.clear();

        RCLCPP_INFO(get_logger(), "PathOptimizerServer deactivated.");
        return CallbackReturn::SUCCESS;
    }

    PathOptimizerServer::CallbackReturn PathOptimizerServer::on_cleanup(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Cleaning up PathOptimizerServer...");

        const bool all_succeeded = cleanup_plugins_best_effort("cleanup transition");

        // 即使某个插件的 cleanup 抛出异常，也必须释放其余实例和 ClassLoader。
        clear_plugin_resources();

        if (!all_succeeded)
        {
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(get_logger(), "PathOptimizerServer cleaned up.");
        return CallbackReturn::SUCCESS;
    }

    PathOptimizerServer::CallbackReturn PathOptimizerServer::on_shutdown(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Shutting down PathOptimizerServer...");

        // shutdown 可能从 active、inactive 或错误状态进入；deactivate() 必须幂等。
        const bool deactivate_succeeded = deactivate_plugins_best_effort(path_optimizer_plugin_ids_, "shutdown");
        const bool cleanup_succeeded = cleanup_plugins_best_effort("shutdown");

        clear_plugin_resources();

        return (deactivate_succeeded && cleanup_succeeded) ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
    }

}  // namespace path_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(path_optimizer::PathOptimizerServer)