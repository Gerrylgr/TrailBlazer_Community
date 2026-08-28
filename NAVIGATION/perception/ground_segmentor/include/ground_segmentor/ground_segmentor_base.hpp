#ifndef GOUND_SEGMENTOR__GROUND_SEGMENTOR_BASE_HPP_
#define GOUND_SEGMENTOR__GROUND_SEGMENTOR_BASE_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ground_segmentor
{
    class GroundSegmentorBase 
    { 
      public:
        using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
        using LifecycleNodeWeakPtr = rclcpp_lifecycle::LifecycleNode::WeakPtr;

        virtual ~GroundSegmentorBase() = default;

        // 对应 Server 的 on_configure()
        // 这里读取参数、创建 publisher、初始化内部状态
        virtual void configure(const LifecycleNodeWeakPtr & parent, const std::string & plugin_name) = 0;

        // 对应 Server 的 on_activate()
        // 这里激活 publisher、创建 subscription、开始处理数据
        virtual void activate() = 0;

        // 对应 Server 的 on_deactivate()
        // 这里关闭 subscription、停用 publisher
        virtual void deactivate() = 0;

        // 对应 Server 的 on_cleanup()
        // 这里释放 publisher、清空缓存、释放 node weak ptr
        virtual void cleanup() = 0;
    };
        
}

#endif      // GOUND_SEGMENTOR__GROUND_SEGMENTOR_BASE_HPP_