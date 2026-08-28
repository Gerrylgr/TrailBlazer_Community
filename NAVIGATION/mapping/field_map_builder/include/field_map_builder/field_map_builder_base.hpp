#ifndef FIELD_MAP_BUILDER__FIELD_MAP_BUILDER_BASE_HPP_
#define FIELD_MAP_BUILDER__FIELD_MAP_BUILDER_BASE_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace field_map_builder
{

  class FieldMapBuilderBase
  {
    public:
        using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
        using LifecycleNodeWeakPtr = rclcpp_lifecycle::LifecycleNode::WeakPtr;

        virtual ~FieldMapBuilderBase() = default;

        // 读取参数、创建 publisher、初始化内部状态
        virtual void configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name) = 0;

        // 激活 publisher、创建 subscription、开始处理数据
        virtual void activate() = 0;

        // 关闭 subscription、停用 publisher
        virtual void deactivate() = 0;

        // 释放 publisher、清空缓存、释放 node weak ptr
        virtual void cleanup() = 0;
  };

}  // namespace field_map_builder

#endif  // FIELD_MAP_BUILDER__FIELD_MAP_BUILDER_BASE_HPP_