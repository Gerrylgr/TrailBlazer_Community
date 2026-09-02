# TrailBlazer 感知模块设计

TrailBlazer 的感知模块负责将 SLAM 输出的原始点云转换为适合建图与导航使用的地面点云和非地面点云。目前主要包含以下两个模块：

- `pointcloud_corrector`：点云坐标变换、范围裁剪与多帧融合；
- `ground_segmentor`：基于局部 PCA 和 2.5D 网格的地面分割。

整体数据流如下：

```text
SLAM 原始点云
    → pointcloud_corrector
    → 坐标校正 / 范围裁剪 / 滑动窗口融合
    → ground_segmentor
    → 地面点云 + 非地面点云
```

---

## 1. `pointcloud_corrector`

### 1.1 模块职责

`pointcloud_corrector` 将原始 SLAM 点云变换到**车体在地面上的投影坐标系**下，并根据配置完成空间范围裁剪和多帧滑动窗口融合。

### 1.2 输入与输出

| 类型 | 参数 | 含义 |
| --- | --- | --- |
| 输入 | `input_cloud_topic` | SLAM 模块输出的点云，例如 `/cloud_registered` |
| 输入 | `input_odom_topic` | 使用的里程计话题，建议使用 IMU 坐标系下的 `/Odometry` |
| 输出 | `output_cloud_topic` | 经过坐标变换、范围裁剪和多帧滑动窗口融合的点云 |
| 输出 | `output_submap_cloud_topic` | 经过坐标变换和范围裁剪（尤其是高度粗裁剪）的单帧局部点云 |
| 输出 | `output_single_topic` | 经过坐标变换和范围裁剪的单帧点云 |

### 1.3 重要参数

| 参数 | 含义 |
| --- | --- |
| `extrinsic_xyz` | 激光雷达相对于车体地面投影坐标系的平移外参。正常情况下主要需要配置 `z` 方向偏移，因为点云不需要平移到车体投影中心 |

---

## 2. `ground_segmentor`

### 2.1 模块职责

`ground_segmentor` 基于**局部 PCA 平面拟合**和**多分辨率 2.5D 网格**完成地面分割。

PCA 的数学原理及完整计算示例见 [PCA_explain.md](./PCA_explain.md)。

### 2.2 算法定位与设计特点

#### 方法定位

从技术类别看，本实现属于**非学习型、几何模型驱动的局部多平面地面分割方法**。它没有直接复现某一篇论文或某一个开源仓库，而是将多种成熟思想重新组织为一条面向移动机器人导航的处理管线。

其中，PCA 平面拟合、低分位点选取、法向量和高度阈值、区域生长以及点到平面距离分类都是已有的经典方法。该实现更值得强调的地方，不是其中某一个数学算子的原创性，而是以下机制之间的组合与工程化衔接：

- 根据点到机器人的距离选择不同分辨率的笛卡尔网格，在近处保留细节、在远处提高单元内的点密度；
- 将 cell 区分为 clean cell 和 mixed cell，并使用常规 PCA、低分位 PCA 与 fallback PCA 组成分层拟合策略；
- 先以严格条件生成可靠地面种子，再通过区域生长、分组合并和 mixed-cell 恢复逐步提高召回率；
- 区域生长不仅比较平面法向量和中心高度，还检查相邻 cell 交界区域的低分位高度连续性；
- 对不直接相邻的地面组，使用距离门限、法向量差异和双向平面高度预测共同判断能否跨越小缺口；
- mixed cell 采用八邻域、宽邻域和传播式三阶段空间恢复；
- 最终不是直接按 cell 二值分类，而是根据点到平面的距离计算点级地面置信度。

> [!IMPORTANT]
> 更严谨的项目定位是“自主设计并实现的多阶段几何地面分割管线”或“面向导航场景的工程化组合与改进”，而不是“全新的地面分割算法”。在没有系统文献检索、公开数据集对比、消融实验和同行评审之前，现不宣称该组合是首创，也不宣称开源社区中不存在相似实现。

#### （AI 帮助检索总结的）与已有研究的联系和区别

| 本实现中的机制 | 可对照的已有研究 | 主要联系 | 本实现的区别 |
| --- | --- | --- | --- |
| 邻近低分位高度、低点种子与 PCA 拟合 | [Zermas 等人的 Ground Plane Fitting（ICRA 2017）](https://doi.org/10.1109/ICRA.2017.7989591) | 都利用较低点更可能属于地面的先验，并通过 PCA 估计平面 | 本实现先估计机器人附近的参考地面高度，又在每个 mixed cell 内独立提取低分位点；没有照搬 GPF 的全局迭代流程 |
| 距离越远、网格越粗的区域表达 | [Patchwork（RA-L 2021）](https://arxiv.org/abs/2108.05560) | 都针对 LiDAR 点密度随距离下降的问题采用非均匀区域划分 | Patchwork 使用同心区、ring 和 sector 构成的极坐标 CZM；本实现仍是笛卡尔网格，只根据距离 band 切换 cell 分辨率 |
| 法向朝向、高度与特征值形状判据 | [Patchwork 的 Ground Likelihood Estimation](https://arxiv.org/abs/2108.05560) | 都从 uprightness、elevation 和 flatness/planarity 角度判断局部平面是否像地面 | 本实现采用显式阈值和主地面高度，并加入区域生长与组级连续性；不是 Patchwork 的概率式 GLE |
| 根据法向量和平滑连续性进行区域扩张 | [Rabbani 等人的 smoothness-constrained region growing](https://research.utwente.nl/en/publications/segmentation-of-point-clouds-using-smoothness-constraints/) 与 [PCL RegionGrowing](https://pcl.readthedocs.io/projects/tutorials/en/master/region_growing_segmentation.html) | 都使用邻接关系、法向量相似性和平滑性约束形成连续表面 | 经典实现通常在点级 KNN 图上生长；本实现工作在 cell 图上，并额外使用边界点高度、主地面组和组间合并规则 |
| mixed-cell 恢复 | [Patchwork++（IROS 2022）](https://arxiv.org/abs/2207.11919) 所讨论的 under-segmentation 恢复问题 | 都试图找回因粗分割或局部异常而漏掉的真实地面 | Patchwork++ 的 TGR 使用跨帧历史状态；本实现是当前帧内的空间邻域恢复与传播，两者不应视为同一算法 |

#### 值得强调的工程思路

1. **先保证种子精度，再分阶段恢复召回率**：严格 seed、受约束 grow、组级 merge、mixed-cell recovery 形成由保守到宽松的 coarse-to-fine 判定过程。
2. **检查真实边界而非只比较 cell 中心**：提取相邻 cell 交界带内的点，并比较低分位高度，可减少仅凭中心高度造成的错误连接。
3. **使用双向平面外推判断跨缺口连续性**：分别把两个局部平面延伸到对方 cell 的位置，并取两侧预测误差的较大值，可避免单向外推偶然通过。
4. **区分 cell 级地面归属与点级最终分类**：即使一个 cell 整体属于地面，其中距离平面较远的点仍可被保留为障碍物；mixed cell 中的低层地面点也有机会被恢复。
5. **计算量受控**：PCA 最多均匀采样 48 个点，并通过距离分段调整网格大小，在局部几何表达与运行开销之间进行折中。

### 2.3 处理流程

#### 1. 基础过滤

1. 去除无效点；
2. 根据距离范围过滤点云；
3. 根据配置选择是否进行体素下采样；
4. 将保留的点转换为内部数据结构并存入 `processed_points_`。

#### 2. 构建 2.5D 网格地图

对于每一个点：

1. 根据点坐标确定其所在圈层；
2. 根据圈层选择对应的网格分辨率；
3. 计算点所属网格的索引坐标；
4. 将点写入 `grid_map_` 中对应的 cell。

#### 3. 局部平面拟合（plane fitting）

遍历所有 cell，并根据 cell 类型尝试拟合局部平面：

- **clean cell**：直接对采样点执行 PCA；
- **mixed cell**：选取指定高度百分位以下的点执行 PCA；
- **fallback PCA**：当前面的拟合策略失败时，使用回退策略再次尝试。

拟合成功且满足判定条件后，将结果写入 `cell.plane`。

#### 4. 生成地面种子（ground seeds）

首先根据车体周围点云的低分位高度估计主地面高度，并保存到 `estimated_ground_z_`。

当主地面存在时，一个 cell 只有同时满足以下条件，才会被选为地面种子：

- 已成功拟合平面；
- 平面法向量与期望地面法向量之间的夹角不能过大；
- cell 内部的高度起伏不能过大；
- cell 中心高度与估计主地面高度之间的差值不能过大。

#### 5. 地面区域生长与合并

##### 5.1 区域生长（grow ground groups）

以所有地面种子为起点遍历邻居。已经被确认属于地面的 cell 会继续向外扩展。

从当前 cell 生长到相邻 cell 时，需要满足以下条件：

- 相邻 cell 尚未被分组；
- 点数足够；
- 边界连续，即边界高度差不能过大；
- 若相邻 cell 已拟合平面，则两个 cell 的中心高度差和法向量夹角均不能过大。

##### 5.2 地面分组合并（merge ground groups）

首先选择距离机器人最近的 ground group 作为主地面组，随后尝试将其他 ground group 合并到其中。

两个 ground group 需要首先满足：

- 整体法向量夹角不能过大；
- 整体中心高度差不能过大。

随后根据两组之间是否存在相邻 cell，分别处理：

- **存在相邻 cell**：执行边界连续性检查；
- **不存在相邻 cell**：
  1. 查找两组中距离最近的一对 cell；
  2. 确认两组之间的距离不超过 `gap_tolerance_`；
  3. 检查最近 cell 的法向量偏差；
  4. 分别使用一个 cell 的平面预测另一个 cell 的高度；
  5. 当预测高度差足够小时，允许合并。

对于所有成功合并的 group ID，遍历 `grid_map_` 并将对应 cell 的 `is_ground` 设置为 `true`。

#### 6. mixed cell 恢复

恢复过程分为三轮：

1. **八邻域恢复**：在基础搜索半径内寻找参考地面平面；
2. **宽邻域恢复**：将搜索半径扩大为上一轮的两倍；
3. **传播式恢复**：允许使用上一轮刚恢复出的 mixed ground plane 作为新的参考平面。

待恢复的 mixed cell 需要满足：

- 点数足够；
- 内部高度起伏达到 mixed cell 的判定阈值；
- 至少存在一个已被判定为地面的邻居；
- 能够获得有效的参考平面。

恢复时还需要保证当前 cell 与参考平面之间的法向量偏差和高度差均在允许范围内。

#### 7. 点分类与发布

遍历所有 cell 中的点并计算地面置信度：

- **mixed cell**：使用幂函数建立置信度与点到平面距离之间的非线性关系；
- **ground cell**：
  - 没有有效平面时，赋予较低置信度；
  - 存在有效平面时，置信度与点到平面的距离呈线性关系。

最后根据 `confidence_thresh_` 将点划分为 ground points 和 non-ground points，并发布对应点云。

### 2.4 输入与输出

| 类型 | 参数 | 含义 |
| --- | --- | --- |
| 输入 | `input_cloud_topic` | 输入点云话题，支持 `map` 或 `body` 坐标系 |
| 输入 | `input_odom_topic` | 输入里程计话题 |
| 输出 | `output_ground_points_topic` | 地面点云话题 |
| 输出 | `output_non_ground_points_topic` | 非地面（障碍物）点云话题 |

### 2.5 调试点云

| 调试阶段 | 话题 |
| --- | --- |
| 2.5D 网格 | `/grid_map_output` |
| 局部平面拟合结果 | `/plane_fit_output` |
| 地面种子 cell | `/ground_seed_output` |
| 最终判定为地面的 cell | `/ground_group_output` |

---

## 3. 实现注意事项

### 3.1 插件内部使用 `weak_ptr` 避免循环引用

Server 节点通过 `shared_ptr` 持有插件实例。插件若再通过 `shared_ptr` 持有 Server 节点，就会形成循环引用，使双方的引用计数都无法归零。

| 持有关系 | 对引用计数的影响 | 销毁结果 |
| --- | --- | --- |
| Server `shared_ptr` → Plugin；Plugin `shared_ptr` → Server | 双方引用计数都会增加 | 形成循环引用，可能无法正常析构 |
| Server `shared_ptr` → Plugin；Plugin `weak_ptr` → Server | `weak_ptr` 不增加 Server 的强引用计数 | Server 可以正常析构，插件中的 `weak_ptr` 随后自动失效 |

因此，插件内部应保存节点的 `weak_ptr`，并在每次使用前调用 `.lock()` 获取临时的 `shared_ptr`：

```cpp
if (auto node = node_weak_ptr_.lock()) {
    // 在当前作用域内安全使用 node
}
```

> [!IMPORTANT]
> `.lock()` 可能返回空指针。插件在使用节点前必须检查结果，不能假设节点仍然存活。

### 3.2 循环引用示例

下面的两个对象通过 `shared_ptr` 相互持有。离开作用域后，外部变量 `a` 和 `b` 虽然被销毁，但两个对象的强引用计数都只会从 2 降到 1，因此析构函数不会执行。

```cpp
#include <iostream>
#include <memory>

struct Node {
    std::shared_ptr<Node> next;

    ~Node() {
        std::cout << "~Node\n";
    }
};

{
    auto a = std::make_shared<Node>();  // A: strong_count = 1
    auto b = std::make_shared<Node>();  // B: strong_count = 1

    a->next = b;  // B: strong_count = 2
    b->next = a;  // A: strong_count = 2
}  // a 和 b 离开作用域，A、B 的 strong_count 均由 2 降为 1
```

这里不能指望 `a->next` 或 `b->next` 自动打破循环：成员变量只有在所属对象开始析构后才会被析构，而对象又只有在强引用计数归零时才会开始析构。

（a 析构 → A 的计数 2→1，不为 0，A 不会被销毁。成员 a->next 并不会被析构——它住在 NodeA 对象内部，只有 ~NodeA() 运行才会析构它，而 ~NodeA() 又只有计数归零才运行。）