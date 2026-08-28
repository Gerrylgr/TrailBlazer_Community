#ifndef OCCUPANCY_MAPPING__STATIC_MAP_IO_HPP_
#define OCCUPANCY_MAPPING__STATIC_MAP_IO_HPP_

#include <string>
#include <filesystem>
#include <fstream>

#include "occupancy_mapping/mapping_types.hpp"

namespace occupancy_mapping
{
    class StaticMapIO
    {
        public:
            struct Result
            {
                bool success{false};
                std::string message;
                std::string metadata_path;
                std::string data_path;
            };

            explicit StaticMapIO(std::string map_directory);

            Result save(const StaticMapData & data, const std::string & map_name, bool overwrite) const;

            Result load(const std::string & map_name, StaticMapData & output) const;

            Result remove(const std::string & map_name) const;

            bool exists(const std::string & map_name) const;

            bool hasAnyFile(const std::string & map_name) const;

        private:
            std::filesystem::path metadataPath(const std::string & map_name) const;

            std::filesystem::path dataPath(const std::string & map_name) const;

        private:
            std::filesystem::path map_directory_;
    };
}   // namespace occupancy_mapping

#endif