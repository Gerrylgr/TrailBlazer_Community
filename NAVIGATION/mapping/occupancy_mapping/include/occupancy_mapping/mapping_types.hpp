#ifndef OCCUPANCY_MAPPING__MAPPING_TYPES_HPP_
#define OCCUPANCY_MAPPING__MAPPING_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <limits>

#include "rclcpp/time.hpp"

namespace occupancy_mapping
{
    enum class MapState : std::uint8_t
    {
        NO_MAP = 0,
        MAP_READY = 1,
        BUILDING_NEW_MAP = 2,
        BUILDING_INCREMENTAL = 3,
        SAVING = 4,
        LOADING = 5,
        CLEARING = 6,
        ERROR = 255
    };

    enum class FinalState : std::uint8_t            // the state of final cells
    {
        Unknown = 0,
        Free = 1,
        Occupied = 2
    };

    struct FinalCell
    {
        float log_odds{0.0f};               // 置信度
        FinalState state{FinalState::Unknown};
        bool ever_updated{false};

        rclcpp::Time last_update{0, 0, RCL_ROS_TIME};           // 上一次更新时间，用于 fade
    };

    struct GroundCell
    {
        bool observed = false;          // 该格是否至少被 ground 点观测过一次（累积地图）
        int count = 0;                  // 累积落入该格的 ground 点数量

        float sum_z = 0.0f;

        float mean_z = 0.0f;            // 暂时没用上
        float ground_z = 0.0f;           // 该格的地面高度估计

        std::vector<float> z_values;    // 第一版直接累积所有 ground z

        void reset()
        {
            observed = false;
            count = 0;   
            sum_z = 0.0f;
            mean_z = 0.0f;
            ground_z = 0.0f; 
            z_values.clear();
        }
    };

    struct ObstacleCell
    {
        bool observed = false;

        int non_ground_count = 0;          // 总候选点数
        int strong_count = 0;              // 强障碍证据点数

        float mean_height = 0.0f;           // cell.mean_height = cell.sum_height / static_cast<float>(cell.non_ground_count);
        float sum_height = 0.0f;            

        bool is_candidate = false;         // 本帧/当前累计下是否认为是障碍候选

        void reset()
        {
            observed = false;
            non_ground_count = 0;
            strong_count = 0;
            sum_height = 0.0f;
            mean_height = 0.0f;
            is_candidate = false;
        }
    };

    // 静态地图数据
    struct StaticMapData
    {
        std::string frame_id{"map"};

        double resolution{0.05};
        double origin_x{0.0};
        double origin_y{0.0};

        int cols{0};
        int rows{0};

        std::vector<FinalCell> cells;

        bool valid() const
        {
            return cols > 0 &&
                rows > 0 &&
                resolution > 0.0 &&
                cells.size() ==
                    static_cast<std::size_t>(cols) *
                    static_cast<std::size_t>(rows);
        }
    };
}

#endif  // OCCUPANCY_MAPPING__MAPPING_TYPES_HPP_