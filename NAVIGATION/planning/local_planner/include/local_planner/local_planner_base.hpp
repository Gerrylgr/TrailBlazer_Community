#ifndef LOCAL_PLANNER__LOCAL_PLANNER_BASE_HPP_
#define LOCAL_PLANNER__LOCAL_PLANNER_BASE_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace local_planner
{
  class LocalPlannerBase
  {
    public:
        using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
        using LifecycleNodeWeakPtr = rclcpp_lifecycle::LifecycleNode::WeakPtr;

        virtual ~LocalPlannerBase() = default;

        // 读取参数、创建 publisher、初始化内部状态
        virtual void configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name) = 0;

        // 激活 publisher、创建 subscription、开始处理数据
        virtual void activate() = 0;

        // 关闭 subscription、停用 publisher
        virtual void deactivate() = 0;

        // 释放 publisher、清空缓存、释放 node weak ptr
        virtual void cleanup() = 0;
  };

}  // namespace local_planner

#endif      // LOCAL_PLANNER__LOCAL_PLANNER_BASE_HPP_