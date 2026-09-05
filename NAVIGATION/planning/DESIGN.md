## 规控模块
### global_planner
目前的实现是占据栅格+ESDF 双引导的 A*。
该实现亮点：
    使用更合理的 Octile 距离计算 h 代价；
    加入防对角穿墙角检查；
    重载优先队列的比较运算符，在 f 代价相同时选择 h 代价更小的节点，提升搜索效率；
    占据栅格+ESDF 双引导；

补充一些关于 A* 的理解/思考：
    首先 A* 的大体流程可概括为：
        每次从优先队列中 pop 出综合代价最小的点作为下一步的探索目标，然后遍历该点的8个邻居，计算所有邻居的g代价，并在g代价（比之前的记录的）更小时更新，此时加上h代价得到综合的f代价，并放入优先队列，如此循环直到找到目标点。

    该 A* 的代价函数为：f = g + h = （几何步长 + 栅格代价 * 系数 + ESDF 代价 * 系数） + octile 距离

    为什么优先队列每次都 pop，却仍可能在后边 pop 出相同的节点？/ closed 数组的存在意义？
    答：由于优先队列的“懒惰删除”机制，详细见 astar_esdf_global_planner.cpp 源码 680 行附近注释。

输入：
    input_costmap_topic：输入的栅格地图话题
    input_esdf_topic：输入的 ESDF 地图话题
    input_odom_topic：输入的里程计话题
    input_goal_topic：目标点话题
输出：
    global_path_topic：全局路径发布话题

重要参数解释：
    if_keep_planning：同一目标点是否持续规划；设为 true 则每一目标点只规划一次全局路径
    min_dist_to_plan：开启规划的最小距离
    treat_unknown_as_obstacle：是否将未知视作障碍物
    obstacle_threshold：大于等于这个数的代价会被当作障碍物
    min_esdf_value_for_goal：目标点能容忍的最大 ESDF 距离（小于则拒绝规划）
    unknown_cost_penalty：如果不将未知视作障碍物，那么它的代价是多少
    cost_scale：占据栅格代价权重
    use_esdf_soft_cost：是否启用 ESDF 代价
    safe_clearance：安全距离（距离小于这个值才会产生 ESDF 代价）
    esdf_cost_scale：ESDF 代价系数
    if_path_downsampling：是否对输出路径下采样（保留拐点）（若后续优化为 Bspline+LBFGS 则不建议开启）
    corner_deg：路径下采样角度阈值
    corner_dilate：路径下采样时拐点上采样数目

### path_optimizer
补充有关 B 样条的一些简单理解：
    对于均匀3次 B 样条，完整的曲线由一段段 3 次多项式首尾拼接而成，而每个 3 次多项式的函数表达式可通过 Cox-de boor 递推公式计算出（也就是基函数，实际上代表了每个点的权重系数），对于均匀3次 B 样条，基函数公式为：
    Ci​(s) = B0​(s)*Pi ​+ B1​(s)*Pi+1 ​+ B2​(s)*Pi+2 ​+ B3​(s)*Pi+3​，s ∈ [0,1]
    其中
        B0​(s) = (1−s)^3 / 6
        B1​(s) = (3s^3−6s^2+4) / 6
        B2​(s) = (−3s^3+3s^2+3s+1) / 6
        B3(s) = s^3 / 6
    Pi 代表四个用于计算曲线点的控制点。

    有关 B 样条参数化部分，见 uniform_bspline.cpp 中 parameterizeToBspline 注释，
    本实现是添加位置约束、起点/终点速度约束，通过最小二乘法求解得到控制点坐标，其中位置约束部分，也就是通过控制点计算曲线点时取端点，也就是s=0时刻，至于为什么是s=0而不是其他值，见 uniform_bspline.cpp 中112行附近注释。