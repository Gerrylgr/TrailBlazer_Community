#include "field_map_builder/utility/esdf_query.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace field_map_builder
{
    std::shared_ptr<const EsdfMapSnapshot> EsdfMapSnapshot::from_message(trailblazer_map_interfaces::msg::EsdfMap::ConstSharedPtr msg)
    {
        if (!msg)
           return nullptr;
        
        if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0 ||
            !std::isfinite(msg->info.resolution) || msg->max_distance <= 0.0f || 
            !std::isfinite(msg->max_distance))
            return nullptr;

        if (msg->info.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
            msg->info.height > static_cast<uint32_t>(std::numeric_limits<int>::max()))
            return nullptr;
        
        const std::size_t cols = static_cast<std::size_t>(msg->info.width);
        const std::size_t rows = static_cast<std::size_t>(msg->info.height);

        if (cols > std::numeric_limits<std::size_t>::max() / rows)
            return nullptr;

        if (msg->distances.size() != cols * rows)
            return nullptr;
        
        return std::shared_ptr<const EsdfMapSnapshot> (new EsdfMapSnapshot(std::move(msg)));
    }

    EsdfMapSnapshot::EsdfMapSnapshot(trailblazer_map_interfaces::msg::EsdfMap::ConstSharedPtr msg) : message_(std::move(msg))
    {
        cols_ = static_cast<int>(message_->info.width);
        rows_ = static_cast<int>(message_->info.height);

        resolution_ = static_cast<double>(message_->info.resolution);
        origin_x_ = message_->info.origin.position.x;
        origin_y_ = message_->info.origin.position.y;
    }

    bool EsdfMapSnapshot::contains_cell(int mx, int my) const noexcept
    {
        return mx >= 0 && mx < cols_ && my >= 0 && my < rows_;
    }

    bool EsdfMapSnapshot::contains_world(double wx, double wy) const noexcept
    {
        if (!std::isfinite(wx) || !std::isfinite(wy))
        {
            return false;
        }

        const double max_x = origin_x_ + static_cast<double>(cols_) * resolution_;
        const double max_y = origin_y_ + static_cast<double>(rows_) * resolution_;

        // 与 OccupancyGrid 的 floor 映射保持一致：左闭右开 [origin, max)
        return wx >= origin_x_ && wx < max_x && wy >= origin_y_ && wy < max_y;
    }

    bool EsdfMapSnapshot::world_to_cell(double wx, double wy, int & mx, int & my) const noexcept
    {
        if (!contains_world(wx, wy))
        {
            return false;
        }

        mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
        my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));

        return contains_cell(mx, my);
    }

    bool EsdfMapSnapshot::cell_to_world(int mx, int my, double & wx, double & wy) const noexcept
    {
        if (!contains_cell(mx, my))
        {
            return false;
        }

        wx = origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_;
        wy = origin_y_ + (static_cast<double>(my) + 0.5) * resolution_;

        return true;
    }

    bool EsdfMapSnapshot::distance_at_cell(int mx, int my, float & distance) const noexcept
    {
        if (!contains_cell(mx, my))
        {
            return false;
        }

        distance = message_->distances[index_unchecked(mx, my)];

        return true;
    }

    bool EsdfMapSnapshot::distance_nearest(double wx, double wy, float & distance) const noexcept
    {
        int mx = 0;
        int my = 0;

        if (!world_to_cell(wx, wy, mx, my))
        {
            return false;
        }

        return distance_at_cell(mx, my, distance);
    }

    bool EsdfMapSnapshot::make_bilinear_stencil(double wx, double wy, BilinearStencil & s) const noexcept
    {
        if (!contains_world(wx, wy))
        {
            return false;
        }

        // 格子中心坐标
        double gx = (wx - origin_x_) / resolution_ - 0.5;
        double gy = (wy - origin_y_) / resolution_ - 0.5;

        // 梯度系数
        s.gradient_x_scale = (cols_ > 1 && gx >= 0.0 && gx <= static_cast<double>(cols_ - 1)) ? 1.0f : 0.0f;
        s.gradient_y_scale = (rows_ > 1 && gy >= 0.0 && gy <= static_cast<double>(rows_ - 1)) ? 1.0f : 0.0f;

        // 地图边缘半个 cell 使用最近边缘值，避免访问 x+1/y+1 越界。
        gx = std::clamp(gx, 0.0, static_cast<double>(cols_ - 1));
        gy = std::clamp(gy, 0.0, static_cast<double>(rows_ - 1));

        s.x0 = static_cast<int>(std::floor(gx));
        s.y0 = static_cast<int>(std::floor(gy));

        s.x1 = std::min(s.x0 + 1, cols_ - 1);
        s.y1 = std::min(s.y0 + 1, rows_ - 1);

        s.tx = static_cast<float>(gx - s.x0);
        s.ty = static_cast<float>(gy - s.y0);

        return true;
    }

    bool EsdfMapSnapshot::distance_and_gradient(double wx, double wy, EsdfQueryResult & result) const noexcept
    {
        BilinearStencil s;
        if (!make_bilinear_stencil(wx, wy, s))
            return false;
        
        const auto & data = message_->distances;

        const float d00 = data[index_unchecked(s.x0, s.y0)];
        const float d10 = data[index_unchecked(s.x1, s.y0)];
        const float d01 = data[index_unchecked(s.x0, s.y1)];
        const float d11 = data[index_unchecked(s.x1, s.y1)];

        const float one_minus_tx = 1.0f - s.tx;
        const float one_minus_ty = 1.0f - s.ty;

        result.distance =                           // 双线性插值计算 ESDF 距离
            d00 * one_minus_tx * one_minus_ty +
            d10 * s.tx        * one_minus_ty +
            d01 * one_minus_tx * s.ty +
            d11 * s.tx         * s.ty;

        const float inv_resolution = 1.0f / static_cast<float>(resolution_);

        // 这里 d10 - d00 相当于格子底边的ESDF距离的“斜率”，d11 - d01 相当于格子顶边的ESDF距离的“斜率”（梯度）
        // 乘上 one_minus_ty/s.ty 是线性插值来计算目标点斜率
        result.gradient_x = ((d10 - d00) * one_minus_ty + (d11 - d01) * s.ty) * inv_resolution * s.gradient_x_scale;
        result.gradient_y = ((d01 - d00) * one_minus_tx + (d11 - d10) * s.tx) * inv_resolution * s.gradient_y_scale;

        return true;
    }

    bool EsdfMapSnapshot::distance_bilinear(double wx, double wy, float & distance) const noexcept
    {
        BilinearStencil s;
        if (!make_bilinear_stencil(wx, wy, s))
            return false;

        const auto & data = message_->distances;

        const float d00 = data[index_unchecked(s.x0, s.y0)];
        const float d10 = data[index_unchecked(s.x1, s.y0)];
        const float d01 = data[index_unchecked(s.x0, s.y1)];
        const float d11 = data[index_unchecked(s.x1, s.y1)];

        const float bottom = d00 + s.tx * (d10 - d00);
        const float top = d01 + s.tx * (d11 - d01);

        distance = bottom + s.ty * (top - bottom);

        return true;
    }

    bool EsdfMapSnapshot::geometry_matches(
        int cols, int rows, double resolution, double origin_x, double origin_y,
        double tolerance) const noexcept
    {
        if (cols_ != cols || rows_ != rows) 
            return false;

        if (std::abs(resolution_ - resolution) > tolerance) 
            return false;

        if (std::abs(origin_x_ - origin_x) > tolerance) 
            return false;

        if (std::abs(origin_y_ - origin_y) > tolerance) 
            return false;

        // 所有检查通过，几何属性匹配
        return true;
    }

}       // namespace field_map_builder