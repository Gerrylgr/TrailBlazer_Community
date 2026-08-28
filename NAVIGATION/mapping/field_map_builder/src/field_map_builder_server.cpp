/*
 * Server 节点：按照 YAML 顺序管理 field-map-builder 插件的生命周期。
 */
#include "field_map_builder/field_map_builder_server.hpp"

#include <exception>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace field_map_builder
{

    FieldMapBuilderServer::FieldMapBuilderServer(const rclcpp::NodeOptions & options)
        : rclcpp_lifecycle::LifecycleNode("field_map_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "FieldMapBuilderServer constructed.");
    }

    bool FieldMapBuilderServer::deactivate_plugins_best_effort(const std::vector<std::string> & plugin_ids, const char * context)
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

    bool FieldMapBuilderServer::cleanup_plugins_best_effort(const char * context)
    {
        bool all_succeeded = true;

        for (auto it = field_map_plugin_ids_.rbegin(); it != field_map_plugin_ids_.rend(); ++it)
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

    void FieldMapBuilderServer::clear_plugin_resources()
    {
        // 必须先销毁插件实例，再销毁负责卸载动态库的 ClassLoader。
        plugin_instances_.clear();
        plugin_loader_.reset();
        active_plugin_ids_.clear();
        field_map_plugin_ids_.clear();
    }

    FieldMapBuilderServer::CallbackReturn FieldMapBuilderServer::on_configure(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Configuring FieldMapBuilderServer...");

        // 正常 lifecycle 流程下这里本来就应为空。该保护用于处理此前失败后遗留的状态。
        if (!plugin_instances_.empty() || plugin_loader_)
        {
            RCLCPP_WARN(get_logger(), "Stale plugin resources found before configure; clearing them.");
            deactivate_plugins_best_effort(field_map_plugin_ids_, "pre-configure recovery");
            cleanup_plugins_best_effort("pre-configure recovery");
            clear_plugin_resources();
        }

        try
        {
            field_map_plugin_ids_ = declare_or_get_parameter<std::vector<std::string>>("field_map_plugins", {"EsdfMapBuilder"});

            if (field_map_plugin_ids_.empty())
            {
                throw std::runtime_error("Parameter 'field_map_plugins' must not be empty.");
            }

            // 在创建任何插件之前完成 ID 校验，避免加载一半后才发现重复 ID。
            std::unordered_set<std::string> unique_ids;
            unique_ids.reserve(field_map_plugin_ids_.size());

            for (const auto & plugin_id : field_map_plugin_ids_)
            {
                if (plugin_id.empty())
                {
                    throw std::runtime_error("Plugin ID in 'field_map_plugins' must not be empty.");
                }

                if (!unique_ids.insert(plugin_id).second)
                {
                    throw std::runtime_error("Duplicate plugin ID declared: '" + plugin_id + "'.");
                }
            }

            plugin_loader_ = std::make_shared<pluginlib::ClassLoader<FieldMapBuilderBase>>("field_map_builder", "field_map_builder::FieldMapBuilderBase");

            for (const auto & plugin_id : field_map_plugin_ids_)
            {
                const std::string plugin_type = declare_or_get_parameter<std::string>(plugin_id + ".plugin", "field_map_builder::EsdfMapBuilding");

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

            RCLCPP_INFO(get_logger(), "Field-map-builder plugins configured successfully.");
            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception & ex)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Exception while configuring field-map-builder plugins: %s",
                ex.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Unknown exception while configuring field-map-builder plugins.");
        }

        cleanup_plugins_best_effort("configure rollback");
        clear_plugin_resources();
        return CallbackReturn::FAILURE;
    }

    FieldMapBuilderServer::CallbackReturn FieldMapBuilderServer::on_activate(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Activating FieldMapBuilderServer...");

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

        for (const auto & plugin_id : field_map_plugin_ids_)
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

    FieldMapBuilderServer::CallbackReturn FieldMapBuilderServer::on_deactivate(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Deactivating FieldMapBuilderServer...");

        const bool all_succeeded = deactivate_plugins_best_effort(active_plugin_ids_, "deactivate transition");

        if (!all_succeeded)
        {
            return CallbackReturn::FAILURE;
        }

        active_plugin_ids_.clear();

        RCLCPP_INFO(get_logger(), "FieldMapBuilderServer deactivated.");
        return CallbackReturn::SUCCESS;
    }

    FieldMapBuilderServer::CallbackReturn FieldMapBuilderServer::on_cleanup(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Cleaning up FieldMapBuilderServer...");

        const bool all_succeeded = cleanup_plugins_best_effort("cleanup transition");

        // 即使某个插件的 cleanup 抛出异常，也必须释放其余实例和 ClassLoader。
        clear_plugin_resources();

        if (!all_succeeded)
        {
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(get_logger(), "FieldMapBuilderServer cleaned up.");
        return CallbackReturn::SUCCESS;
    }

    FieldMapBuilderServer::CallbackReturn FieldMapBuilderServer::on_shutdown(const rclcpp_lifecycle::State & state)
    {
        (void)state;

        RCLCPP_INFO(get_logger(), "Shutting down FieldMapBuilderServer...");

        // shutdown 可能从 active、inactive 或错误状态进入；deactivate() 必须幂等。
        const bool deactivate_succeeded = deactivate_plugins_best_effort(field_map_plugin_ids_, "shutdown");
        const bool cleanup_succeeded = cleanup_plugins_best_effort("shutdown");

        clear_plugin_resources();

        return (deactivate_succeeded && cleanup_succeeded) ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
    }

}  // namespace field_map_builder

RCLCPP_COMPONENTS_REGISTER_NODE(field_map_builder::FieldMapBuilderServer)