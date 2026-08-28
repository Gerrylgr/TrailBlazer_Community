/*
*   地图文件的保存、删除、加载等操作：
*       save：保存地图文件（.meta 元数据文件、.data 地图数据文件）
*       load：输入地图名称，输出 StaticMapData 类型的单帧地图数据
*       remove：删除地图文件
*
*   之所以选择保存 地图元数据➕final_cells 而不是单纯的 occupancy map，
*   是为了实现增量式建图，如果只有 occupancy map，无法还原为 final_cells，
*   而 final_cells 中多出了很多信息，最重要的，就拿置信度来说，
*   如果有完整的 final_cells，可以在之前的置信度的基础上进一步构建，这样很明显更加合理准确
*/
#include "occupancy_mapping/static_map_io.hpp"

#include <cmath>
#include <iomanip>
#include <utility>

namespace occupancy_mapping
{
    StaticMapIO::StaticMapIO(std::string map_directory) : map_directory_(std::move(map_directory))
    {
        // 确保存储地图的根目录存在
        std::filesystem::create_directories(map_directory_);
    }

    std::filesystem::path StaticMapIO::metadataPath(const std::string & map_name) const
    {
        return map_directory_ / (map_name + ".meta");                   // 拼接路径
    }

    std::filesystem::path StaticMapIO::dataPath(const std::string & map_name) const
    {
        return map_directory_ / (map_name + ".data");
    }

    // 两个地图文件是否存在
    bool StaticMapIO::exists(const std::string & map_name) const
    {
        return std::filesystem::exists(metadataPath(map_name)) && std::filesystem::exists(dataPath(map_name));
    }

    // if any of data file/meta file exits
    bool StaticMapIO::hasAnyFile(const std::string & map_name) const
    {
        return std::filesystem::exists(metadataPath(map_name)) || std::filesystem::exists(dataPath(map_name));
    }

    StaticMapIO::Result StaticMapIO::save(const StaticMapData & data, const std::string & map_name, bool overwrite) const
    {
        Result res;
        if (!data.valid()) 
        {
            res.message = "Map data is invalid (check dimensions or cells size).";
            return res;
        }

        if (exists(map_name) && !overwrite) 
        {
            res.message = "Map already exists and overwrite is false.";
            return res;
        }

        auto meta_p = metadataPath(map_name);
        auto data_p = dataPath(map_name);

        // 保存元数据 (使用简单的文本格式)
        std::ofstream meta_file(meta_p);
        if (!meta_file.is_open()) 
        {
            res.message = "Failed to open metadata file for writing: " + meta_p.string();
            return res;
        }
        meta_file << "frame_id " << data.frame_id << "\n";
        meta_file << "resolution " << std::setprecision(10) << data.resolution << "\n";
        meta_file << "origin_x " << std::setprecision(10) << data.origin_x << "\n";
        meta_file << "origin_y " << std::setprecision(10) << data.origin_y << "\n";
        meta_file << "cols " << data.cols << "\n";
        meta_file << "rows " << data.rows << "\n";
        meta_file.close();

        // 保存栅格数据 (使用二进制格式提高效率)
        std::ofstream data_file(data_p, std::ios::binary);
        if (!data_file.is_open()) 
        {
            res.message = "Failed to open data file for writing: " + data_p.string();
            return res;
        }

        // 写入格子总数，用于加载时校验
        const std::size_t num_cells = data.cells.size();
        data_file.write(reinterpret_cast<const char *>(&num_cells), sizeof(std::size_t));

        // 逐个写入 FinalCell
        for (const auto & cell : data.cells) 
        {
            // 写入 log_odds
            data_file.write(reinterpret_cast<const char *>(&cell.log_odds), sizeof(float));
            
            // 写入 state
            uint8_t state_val = static_cast<uint8_t>(cell.state);
            data_file.write(reinterpret_cast<const char *>(&state_val), sizeof(uint8_t));
            
            // 写入 ever_updated
            uint8_t ever_updated_val = cell.ever_updated ? 1 : 0;
            data_file.write(reinterpret_cast<const char *>(&ever_updated_val), sizeof(uint8_t));

            // 处理 rclcpp::Time：分解为秒和纳秒存储
            int32_t sec = static_cast<int32_t>(cell.last_update.nanoseconds() / 1000000000);
            uint32_t nsec = static_cast<uint32_t>(cell.last_update.nanoseconds() % 1000000000);
            data_file.write(reinterpret_cast<const char *>(&sec), sizeof(int32_t));
            data_file.write(reinterpret_cast<const char *>(&nsec), sizeof(uint32_t));
        }
        data_file.close();

        res.success = true;
        res.message = "Map saved successfully.";
        res.metadata_path = meta_p.string();
        res.data_path = data_p.string();
        return res;
    }

    StaticMapIO::Result StaticMapIO::load(const std::string & map_name, StaticMapData & output) const
    {
        Result res;
        if (!exists(map_name)) 
        {
            res.message = "Map files do not exist for map: " + map_name;
            return res;
        }

        auto meta_p = metadataPath(map_name);
        auto data_p = dataPath(map_name);

        // 读取元数据
        std::ifstream meta_file(meta_p);
        if (!meta_file.is_open()) 
        {
            res.message = "Failed to open metadata file: " + meta_p.string();
            return res;
        }

        output = StaticMapData{};       // 创建输出对象
        std::string key;
        while (meta_file >> key) 
        {
            if (key == "frame_id") meta_file >> output.frame_id;
            else if (key == "resolution") meta_file >> output.resolution;
            else if (key == "origin_x") meta_file >> output.origin_x;
            else if (key == "origin_y") meta_file >> output.origin_y;
            else if (key == "cols") meta_file >> output.cols;
            else if (key == "rows") meta_file >> output.rows;
        }
        meta_file.close();

        // if (!output.valid()) 
        // {
        //     res.message = "Loaded metadata is invalid.";
        //     return res;
        // }

        // 这时候只加载了元数据，没有加载 cells 内容，所以不能用 output.valid()（cells 没有分配内存，size 不对）
        if (output.cols <= 0 || output.rows <= 0 || output.resolution <= 0.0 ||
            !std::isfinite(output.resolution) || !std::isfinite(output.origin_x) || !std::isfinite(output.origin_y))
        {
            res.message = "Loaded metadata geometry is invalid.";
            return res;
        }

        // 读取栅格数据
        std::ifstream data_file(data_p, std::ios::binary);
        if (!data_file.is_open()) 
        {
            res.message = "Failed to open data file: " + data_p.string();
            return res;
        }

        std::size_t num_cells = 0;
        data_file.read(reinterpret_cast<char *>(&num_cells), sizeof(std::size_t));

        if (num_cells != static_cast<std::size_t>(output.cols) * static_cast<std::size_t>(output.rows)) 
        {
            res.message = "Data size does not match metadata dimensions.";
            return res;
        }

        output.cells.resize(num_cells);
        for (std::size_t i = 0; i < num_cells; ++i) 
        {
            auto & cell = output.cells[i];
            
            // 读取 log_odds
            data_file.read(reinterpret_cast<char *>(&cell.log_odds), sizeof(float));
            
            // 读取 state
            uint8_t state_val = 0;
            data_file.read(reinterpret_cast<char *>(&state_val), sizeof(uint8_t));
            
            // 读取 ever_updated
            uint8_t ever_updated_val = 0;
            data_file.read(reinterpret_cast<char *>(&ever_updated_val), sizeof(uint8_t));
            
            // 读取时间戳
            int32_t sec = 0;
            uint32_t nsec = 0;
            data_file.read(reinterpret_cast<char *>(&sec), sizeof(int32_t));
            data_file.read(reinterpret_cast<char *>(&nsec), sizeof(uint32_t));

            // 在每次读取完一个 cell 后，立刻检查流状态
            // 如果流状态无效（可能是文件截断，也可能是磁盘错误），直接返回报错
            if (!data_file) 
            {
                res.message = "Unexpected end of map data file or read error occurred.";
                return res;
            }

            // 检查 state_val 的合法性
            // 如果 state_val 超过了最大定义值，说明数据损坏
            if (state_val > static_cast<uint8_t>(FinalState::Occupied)) 
            {
                res.message = "Invalid cell state encountered in data file.";
                return res;
            }
            cell.state = static_cast<FinalState>(state_val);

            // 处理其他字段
            cell.ever_updated = (ever_updated_val != 0);
            cell.last_update = rclcpp::Time(sec, nsec, RCL_ROS_TIME); 
        }

        data_file.close();

        if (!output.valid())
        {
            res.message = "Loaded map data is invalid.";
            return res;
        }

        res.success = true;
        res.message = "Map loaded successfully.";
        res.metadata_path = meta_p.string();
        res.data_path = data_p.string();
        return res;
    }

    StaticMapIO::Result StaticMapIO::remove(const std::string & map_name) const
    {
        Result res;
        auto meta_p = metadataPath(map_name);
        auto data_p = dataPath(map_name);

        bool removed_any = false;
        std::string error_msg;

        if (std::filesystem::exists(meta_p)) 
        {
            if (!std::filesystem::remove(meta_p)) 
            {
                error_msg += "Failed to remove metadata file. ";
            } 
            else 
            {
                removed_any = true;
            }
        }

        if (std::filesystem::exists(data_p)) 
        {
            if (!std::filesystem::remove(data_p)) 
            {
                error_msg += "Failed to remove data file. ";
            } 
            else 
            {
                removed_any = true;
            }
        }

        if (!error_msg.empty()) 
        {
            res.message = error_msg;
            return res;
        }

        if (removed_any) 
        {
            res.success = true;
            res.message = "Map files removed successfully.";
            res.metadata_path = meta_p.string();
            res.data_path = data_p.string();
        } 
        else 
        {
            res.success = true;
            res.message = "Map files not found, nothing to remove.";
        }

        return res;
    }

}   // namespace occupancy_mapping
