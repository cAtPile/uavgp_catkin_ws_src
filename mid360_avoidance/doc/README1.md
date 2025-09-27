### 工程算法与结构分析

该工程是一个局部路径规划器（`LocalPlanner`），主要用于无人机等移动机器人的避障路径规划，基于直方图（Histogram）和星形搜索树（Star Planner）实现。以下从整体结构、核心算法、函数任务及依赖关系展开分析。


### 一、工程结构
```
local_planner/         # 核心模块
test/                  # 单元测试
avoidance/             # 避障相关工具类
src/                   # 源代码
  nodes/               # 节点实现（主逻辑）
    local_planner.cpp  # 局部规划器主逻辑
    star_planner.cpp   # 星形搜索树实现
    planner_functions.cpp  # 规划辅助函数
    ...
  common.cpp           # 通用工具函数
```


### 二、核心算法概述
1. **直方图（Histogram）表示障碍物**  
   将三维空间的障碍物点云转换为极坐标下的二维直方图（方位角+仰角），每个单元格存储障碍物距离，用于快速计算避障成本。

2. **成本矩阵（Cost Matrix）计算**  
   基于直方图和目标位置，为每个方向（极坐标角度）计算成本，综合障碍物距离、目标方向一致性等因素。

3. **星形搜索树（Star Planner）**  
   构建搜索树生成候选路径，通过启发式函数（目标距离）和成本矩阵选择最优路径，实现局部避障。


### 三、关键函数任务与实现原理

#### 1. `LocalPlanner::determineStrategy()`（local_planner.cpp）
- **任务**：规划主入口，协调障碍物处理、成本计算和路径搜索。
- **流程**：
  1. 清空并初始化成本图像数据。
  2. 生成二维障碍物表示（`create2DObstacleRepresentation`）。
  3. 计算车辆在“前目标-当前目标”连线上的投影点（`closest_pt_`），用于路径约束。
  4. 若存在障碍物直方图，调用`getCostMatrix`计算成本矩阵，并通过`star_planner_`构建搜索树。


#### 2. 点云处理与直方图生成（planner_functions.cpp）
- **`processPointcloud()`**：
  - 任务：处理原始点云（去噪、裁剪、与历史数据融合），输出用于规划的点云（`final_cloud`）。
  - 原理：
    - 按极坐标网格划分点云，每个网格保留足够数量的点（过滤噪声）。
    - 融合历史点云（未过期且不在当前视野内的点），维持环境记忆。

- **`generateNewHistogram()`**：
  - 任务：将处理后的点云转换为极坐标直方图（`polar_histogram`）。
  - 原理：统计每个极坐标网格（方位角`z`、仰角`e`）内障碍物的平均距离，存储于直方图中。

- **`compressHistogramElevation()`**：
  - 任务：压缩直方图的仰角维度，仅保留每个方位角下最近的障碍物距离（简化计算）。


#### 3. 成本矩阵计算（planner_functions.cpp）
- **`getCostMatrix()`**：
  - 任务：为每个极坐标方向计算成本，输出成本矩阵（`cost_matrix`）和可视化数据。
  - 原理：
    1. 检查目标方向是否存在障碍物（`is_obstacle_facing_goal`）。
    2. 对每个极坐标网格，通过`costFunction`计算成本（综合障碍物距离、目标方向偏差等）。
    3. 对成本矩阵进行平滑处理（`smoothPolarMatrix`），避免局部极小值。


#### 4. 星形搜索树（star_planner.cpp）
- **`StarPlanner::buildLookAheadTree()`**：
  - 任务：构建搜索树生成避障路径。
  - 原理：
    1. 以当前位置为根节点初始化树，通过成本矩阵选择低-cost方向生成子节点。
    2. 每个节点的成本由父节点成本、当前方向成本和启发式函数（到目标的距离）组成。
    3. 扩展节点至最大数量（`n_expanded_nodes_`），选择最深层节点回溯生成路径。

- **`treeHeuristicFunction()`**：
  - 任务：计算启发式成本（引导搜索向目标方向进行），值为节点到目标的距离乘以权重。


#### 5. 辅助函数（common.cpp & planner_functions.h）
- **`removeNaNAndGetMaxima()`**：过滤点云中的无效点（NaN），保留各轴极值点（用于视野计算）。
- **`costFunction()`**：计算单个方向的成本，考虑：
  - 障碍物距离（越近成本越高）。
  - 与目标方向的偏差（偏差越大成本越高）。
  - 车辆速度方向一致性（平滑运动优先）。


### 四、模块依赖关系
1. **数据流向**：
   ```
   原始点云 → processPointcloud() → 处理后的点云 → generateNewHistogram() → 直方图
   直方图 + 目标位置 → getCostMatrix() → 成本矩阵 → StarPlanner → 最优路径
   ```

2. **关键依赖**：
   - `LocalPlanner` 依赖 `StarPlanner` 生成路径，依赖 `planner_functions` 处理点云和计算成本。
   - `StarPlanner` 依赖成本矩阵（`getCostMatrix`）和启发式函数规划路径。
   - 可视化模块（`LocalPlannerVisualization`）依赖所有核心数据（点云、路径、成本矩阵）进行可视化。


### 五、总结
该工程通过点云处理构建环境的直方图表示，结合成本矩阵评估各方向避障代价，最终使用星形搜索树生成局部最优路径。核心是将三维空间问题转换为极坐标下的二维成本计算，平衡了避障安全性和向目标运动的效率。各模块分工明确：点云处理提供环境感知，直方图简化环境表示，成本矩阵量化避障代价，星形搜索树实现路径搜索。