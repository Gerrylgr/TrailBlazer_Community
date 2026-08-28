#include "occupancy_mapping/grid_geometry.hpp"

#include <cmath>

namespace occupancy_mapping
{
    void GridGeometry::configure(double resolution, double origin_x, double origin_y, int cols, int rows)
    {
        resolution_ = resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        cols_ = cols;
        rows_ = rows;
    }

    // if resolution && cols && rows > 0.0
    bool GridGeometry::valid() const
    {
        return resolution_ > 0.0 && cols_ > 0 && rows_ > 0;         // 分辨率、长宽都大于0
    }

    bool GridGeometry::worldToMap(double wx, double wy, int & mx, int & my) const
    {
        mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
        my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));

        return isValidIndex(mx, my);
    }

    void GridGeometry::mapToWorld(int mx, int my, double & wx, double & wy) const
    {
        wx = (mx + 0.5) * resolution_ + origin_x_;
        wy = (my + 0.5) * resolution_ + origin_y_;
    }

    bool GridGeometry::isValidIndex(int mx, int my) const
    {
        return mx >= 0 && mx < cols_ && my >= 0 && my < rows_;
    }

    std::size_t GridGeometry::toIndex(int mx, int my) const
    {
        return static_cast<size_t>(mx) + static_cast<size_t>(my) * static_cast<size_t>(cols_);
    }

    // return origin_x_
    double GridGeometry::minX() const { return origin_x_; }
    // return origin_y_
    double GridGeometry::minY() const { return origin_y_; }

    double GridGeometry::maxX() const
    {
        return origin_x_ + resolution_ * cols_;
    }

    double GridGeometry::maxY() const
    {
        return origin_y_ + resolution_ * rows_;
    }

    // getter
    double GridGeometry::resolution() const { return resolution_; }
    double GridGeometry::originX() const { return origin_x_; }
    double GridGeometry::originY() const { return origin_y_; }
    int GridGeometry::cols() const { return cols_; }
    int GridGeometry::rows() const { return rows_; }

}   // namespace trailblazer_mapping
