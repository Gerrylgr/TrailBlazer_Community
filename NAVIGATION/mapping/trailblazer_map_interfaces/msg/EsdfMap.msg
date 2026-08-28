# ESDF msg for TrailBlazer
std_msgs/Header header
nav_msgs/MapMetaData info           # 地图基本信息
float32 max_distance                # ESDF 地图中超过 max_distance 距离的全部截断
bool unknown_as_obstacle            # ESDF 地图中是否将未知视为障碍物
float32[] distances                 # ESDF 地图数据