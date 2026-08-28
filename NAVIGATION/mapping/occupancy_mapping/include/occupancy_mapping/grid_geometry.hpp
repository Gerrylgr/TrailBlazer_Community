/*
* 存储当下的地图信息，并提供一些有关地图的基本几何操作
*/
#ifndef TRAILBLAZER_MAPPING__GRID_GEOMETRY_HPP_
#define TRAILBLAZER_MAPPING__GRID_GEOMETRY_HPP_

#include <cstddef>

namespace occupancy_mapping
{
    class GridGeometry
    {
        public:
            void configure(double resolution, double origin_x, double origin_y, int cols, int rows);

            bool valid() const;

            bool worldToMap(double wx, double wy, int & mx, int & my) const;
            void mapToWorld(int mx, int my, double & wx, double & wy) const;

            bool isValidIndex(int mx, int my) const;

            std::size_t toIndex(int mx, int my) const;

            double minX() const;
            double minY() const;
            double maxX() const;
            double maxY() const;

            // getter
            double resolution() const;
            double originX() const;
            double originY() const;
            int cols() const;
            int rows() const;

        private:
            double resolution_{0.05};
            double origin_x_{0.0};
            double origin_y_{0.0};

            int cols_{0};
            int rows_{0};
    };
}   // namespace trailblazer_mapping

#endif