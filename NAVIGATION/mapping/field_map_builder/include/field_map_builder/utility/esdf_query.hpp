
/*
*   ESDF 数据接口类，对外提供 ESDF 地图的数据查询等接口
*   思路：
*       原子操作更新 esdf msg/获取 EsdfMapSnapshot 对象，其余的梯度查询等处就无需上锁
*/
#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "trailblazer_map_interfaces/msg/esdf_map.hpp"

namespace field_map_builder
{
    struct EsdfQueryResult
    {
        float distance{0.0f};

        // ESDF 对世界坐标 x、y 的偏导数
        float gradient_x{0.0f};
        float gradient_y{0.0f};
    };

    class EsdfMapSnapshot final
    {
        public:
            static std::shared_ptr<const EsdfMapSnapshot> from_message(trailblazer_map_interfaces::msg::EsdfMap::ConstSharedPtr msg);

            bool contains_cell(int mx, int my) const noexcept;
            bool contains_world(double wx, double wy) const noexcept;

            bool world_to_cell(double wx, double wy, int & mx, int & my) const noexcept;
            bool cell_to_world(int mx, int my, double & wx, double & wy) const noexcept;

            // -------------------------- 查询 ESDF 距离 --------------------------
            bool distance_at_cell(int mx, int my, float & distance) const noexcept;             // 输入 map 坐标查询
            bool distance_nearest(double wx, double wy, float & distance) const noexcept;       // 输入 world 坐标查询

            // 
            bool distance_bilinear(double wx, double wy, float & distance) const noexcept;              // 双线性插值计算 ESDF 距离
            bool distance_and_gradient(double wx, double wy, EsdfQueryResult & result) const noexcept;      // 双线性插值计算 ESDF 距离和梯度

            // -------------------------- 获取地图的元数据 --------------------------
            int cols() const noexcept { return cols_; }
            int rows() const noexcept { return rows_; }
            double resolution() const noexcept {return resolution_;}
            double origin_x() const noexcept {return origin_x_;}
            double origin_y() const noexcept {return origin_y_;}
            float max_distance() const noexcept {return message_->max_distance;}
            bool unknown_as_obstacle() const noexcept {return message_->unknown_as_obstacle;}
            const std::string & frame_id() const noexcept {return message_->header.frame_id;}
            const builtin_interfaces::msg::Time & stamp() const noexcept {return message_->header.stamp;}

            // （输入 occupancy map 元数据）和 ESDF 地图元数据比较是否一致
            bool geometry_matches(
                int cols,
                int rows,
                double resolution,
                double origin_x,
                double origin_y,
                double tolerance = 1e-6) const noexcept;

        private:
            // 私有化构造函数，防止通过外部直接 new 来创建对象
            // 只允许通过内部接口 from_message 来创建对象
            explicit EsdfMapSnapshot(trailblazer_map_interfaces::msg::EsdfMap::ConstSharedPtr msg);

            struct BilinearStencil
            {
                // 双线性插值的四个顶点 X/Y 坐标
                int x0{0};
                int x1{0};
                int y0{0};
                int y1{0};

                // 插值权重
                float tx{0.0f};
                float ty{0.0f};

                // 若对应坐标被 clamp 到边缘，则该方向梯度设为 0
                float gradient_x_scale{1.0f};
                float gradient_y_scale{1.0f};
            };

            bool make_bilinear_stencil(double wx, double wy, BilinearStencil & stencil) const noexcept;

            std::size_t index_unchecked(int mx, int my) const noexcept
            {
                return static_cast<std::size_t>(my) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(mx);
            }

        private:
            // 直接持有收到的 const ROS 消息，不再复制 distances
            trailblazer_map_interfaces::msg::EsdfMap::ConstSharedPtr message_;

            int cols_{0};
            int rows_{0};

            double resolution_{0.0};
            double origin_x_{0.0};
            double origin_y_{0.0};
    };

    class EsdfMapBuffer final
    {
    public:

        EsdfMapBuffer() = default;

        // 禁止拷贝构造/赋值
        EsdfMapBuffer(const EsdfMapBuffer &) = delete;
        EsdfMapBuffer & operator=(const EsdfMapBuffer &) = delete;

        // 禁止移动构造/赋值
        EsdfMapBuffer(EsdfMapBuffer &&) = delete;
        EsdfMapBuffer & operator=(EsdfMapBuffer &&) = delete;

        // 更新 EsdfMapSnapshot
        bool update(trailblazer_map_interfaces::msg::EsdfMap::ConstSharedPtr msg)
        {
            std::shared_ptr<const EsdfMapSnapshot> next = EsdfMapSnapshot::from_message(std::move(msg));

            if (!next)
            {
                // 保留上一张有效地图，不把无效消息发布给查询线程。
                return false;
            }

            // 把新的地图指针 next 存入成员变量 latest_
            std::atomic_store_explicit(&latest_, std::move(next), std::memory_order_release);

            return true;
        }

        // 获取 EsdfMapSnapshot 变量来使用
        std::shared_ptr<const EsdfMapSnapshot> snapshot() const noexcept
        {
            return std::atomic_load_explicit(&latest_, std::memory_order_acquire);
        }

        // 清空 latest_（EsdfMapSnapshot 对象）
        void clear() noexcept
        {
            std::atomic_store_explicit(&latest_, std::shared_ptr<const EsdfMapSnapshot> {}, std::memory_order_release);
        }

    private:
        std::shared_ptr<const EsdfMapSnapshot> latest_;
    };
}   // namespace field_map_builder